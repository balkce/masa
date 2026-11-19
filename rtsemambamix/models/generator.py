from __future__ import annotations

import torch
import torch.nn as nn
from einops import rearrange

from .mamba_block import MLPWithNorm, TFMambaBlock
from .transformer_block import TFTransformerBlock
from .codec_module import DenseEncoder, MagDecoder, MapMagDecoder, PhaseDecoder


class SEMamba(nn.Module):
    """Speech enhancement network using Jamba-style Mamba + Transformer blocks."""

    def __init__(self, cfg):
        super(SEMamba, self).__init__()
        self.cfg = cfg
        self.num_tscblocks = cfg["model_cfg"].get("num_tfmamba") or 4
        self.mapping = bool(cfg["model_cfg"].get("mapping", False))
        self._streaming_enabled = False
        self._transformer_cache_limit = None
        self._transformer_cache_seconds = cfg["model_cfg"].get("transformer_cache_seconds")

        self.dense_encoder = DenseEncoder(cfg)

        # Jamba stack: alternating Mamba and occasional Transformer blocks with MLPs
        # assign unique layer_idx for Mamba selective scan cache
        layer_offset = 0
        self.TSMamba = nn.ModuleList()
        for _ in range(self.num_tscblocks):
            block = TFMambaBlock(cfg, layer_offset=layer_offset)
            layer_offset += block.total_layers
            self.TSMamba.append(block)
        self.mlp_after_mamba = nn.ModuleList(MLPWithNorm(cfg) for _ in range(self.num_tscblocks))

        # Optional Transformer blocks interleaved into the Mamba stack.
        # With the default stride/offset no position matches for the KD1/KD8
        # depths, so both released models are pure-Mamba stacks.
        transformer_positions = cfg["model_cfg"].get("transformer_positions")
        if transformer_positions is None:
            stride = cfg["model_cfg"].get("transformer_stride", 40)
            offset = cfg["model_cfg"].get("transformer_offset", 20)
            transformer_positions = [idx for idx in range(self.num_tscblocks) if idx % stride == offset]
        #    print(f"Transformer positions: {transformer_positions} (stride={stride}, offset={offset})")
        #else:
        #    print(f"Transformer positions: {transformer_positions} (from config)")
        self.transformer_positions = transformer_positions
        self.transformer_blocks = nn.ModuleList(TFTransformerBlock(cfg) for _ in self.transformer_positions)
        self.mlp_after_transformer = nn.ModuleList(MLPWithNorm(cfg) for _ in self.transformer_positions)

        if self.mapping:
            self.mask_decoder = MapMagDecoder(cfg)
        else:
            self.mask_decoder = MagDecoder(cfg)
        self.phase_decoder = PhaseDecoder(cfg)
        self._transformer_cache = [None for _ in self.transformer_blocks]

    def set_streaming(self, enabled: bool, cache_seconds: float | None = None) -> None:
        self._streaming_enabled = enabled
        if cache_seconds is None:
            cache_seconds = self._transformer_cache_seconds
        if cache_seconds is not None:
            hop_size = self.cfg["stft_cfg"]["hop_size"]
            sampling_rate = self.cfg["stft_cfg"]["sampling_rate"]
            frames = int(round(cache_seconds * sampling_rate / hop_size))
            self._transformer_cache_limit = max(frames, 1)
        else:
            self._transformer_cache_limit = None
        self.dense_encoder.set_streaming(enabled)
        for block in self.TSMamba:
            block.set_streaming(enabled)
        self.mask_decoder.set_streaming(enabled)
        self.phase_decoder.set_streaming(enabled)
        if enabled:
            self.reset_streaming_state()

    def reset_streaming_state(self) -> None:
        self.dense_encoder.reset_streaming()
        for block in self.TSMamba:
            block.reset_streaming_state()
        self.mask_decoder.reset_streaming()
        self.phase_decoder.reset_streaming()
        self._transformer_cache = [None for _ in self.transformer_blocks]

    def _streaming_transformer_forward(self, idx: int, x: torch.Tensor) -> torch.Tensor:
        cache = self._transformer_cache[idx]
        if cache is None:
            cache = x
        else:
            cache = torch.cat([cache, x], dim=2)
        if self._transformer_cache_limit is not None and cache.shape[2] > self._transformer_cache_limit:
            cache = cache[:, :, -self._transformer_cache_limit :, :]
        x_full = self.transformer_blocks[idx](cache)
        x_last = x_full[:, :, -1:, :]
        if not self.training:
            cache = cache.detach()
        self._transformer_cache[idx] = cache
        return x_last

    def forward(self, noisy_mag, noisy_pha, noise_mag, noise_pha, inference_params=None, return_features: bool = False):
        noisy_mag = rearrange(noisy_mag, "b f t -> b t f").unsqueeze(1)
        noisy_pha = rearrange(noisy_pha, "b f t -> b t f").unsqueeze(1)
        noise_mag = rearrange(noise_mag, "b f t -> b t f").unsqueeze(1)
        noise_pha = rearrange(noise_pha, "b f t -> b t f").unsqueeze(1)

        x = torch.cat((noisy_mag, noisy_pha, noise_mag, noise_pha), dim=1)
        x = self.dense_encoder(x)

        streaming = self._streaming_enabled and x.shape[2] == 1
        transformer_idx = 0
        tfmamba_block_outputs = [] if return_features else None
        for i in range(self.num_tscblocks):
            x = self.TSMamba[i](x, inference_params=inference_params)
            x = self.mlp_after_mamba[i](x)
            if return_features:
                tfmamba_block_outputs.append(x)

            if i in self.transformer_positions:
                if streaming:
                    x = self._streaming_transformer_forward(transformer_idx, x)
                else:
                    x = self.transformer_blocks[transformer_idx](x)
                x = self.mlp_after_transformer[transformer_idx](x)
                transformer_idx += 1

        mag_out = self.mask_decoder(x)
        if self.mapping:
            denoised_mag = rearrange(mag_out, "b 1 t f -> b f t")
        else:
            denoised_mag = rearrange(mag_out * noisy_mag, "b 1 t f -> b f t")
        denoised_pha = rearrange(self.phase_decoder(x), "b c t f -> b f t c").squeeze(-1)

        denoised_com = torch.stack(
            (denoised_mag * torch.cos(denoised_pha), denoised_mag * torch.sin(denoised_pha)),
            dim=-1,
        )

        if not return_features:
            return denoised_mag, denoised_pha, denoised_com

        features = {
            "tfmamba_blocks": tfmamba_block_outputs,
            "pre_decoder": x,
        }
        return denoised_mag, denoised_pha, denoised_com, features
