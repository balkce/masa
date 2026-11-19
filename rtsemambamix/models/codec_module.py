from __future__ import annotations

# Reference: https://github.com/yxlu-0102/MP-SENet/blob/main/models/generator.py

import torch
import torch.nn as nn
import torch.nn.functional as F
from einops import rearrange

from .lsigmoid import LearnableSigmoid2D


def get_padding(kernel_size, dilation=1):
    """Calculate padding for 1D convolutions."""

    return int((kernel_size * dilation - dilation) / 2)


def get_padding_2d(kernel_size, dilation=(1, 1)):
    """Calculate padding for 2D convolutions."""

    return (
        int((kernel_size[0] * dilation[0] - dilation[0]) / 2),
        int((kernel_size[1] * dilation[1] - dilation[1]) / 2),
    )


class DenseBlock_old(nn.Module):
    """Legacy dense block retained for reference compatibility."""

    def __init__(self, cfg, kernel_size=(3, 3), depth=4):
        super(DenseBlock_old, self).__init__()
        self.cfg = cfg
        self.depth = depth
        self.dense_block = nn.ModuleList()
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        for i in range(depth):
            dil = 2 ** i
            dense_conv = nn.Sequential(
                nn.Conv2d(
                    self.hid_feature * (i + 1),
                    self.hid_feature,
                    kernel_size,
                    dilation=(dil, 1),
                    padding=get_padding_2d(kernel_size, (dil, 1)),
                ),
                ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
                nn.PReLU(self.hid_feature),
            )
            self.dense_block.append(dense_conv)

    def forward(self, x):
        skip = x
        for i in range(self.depth):
            x = self.dense_block[i](skip)
            skip = torch.cat([x, skip], dim=1)
        return x


class ChannelLayerNorm2d(nn.Module):
    """Channel-only LayerNorm over time-frequency positions (causal friendly)."""

    def __init__(self, num_channels, eps=1e-5):
        super(ChannelLayerNorm2d, self).__init__()
        self.norm = nn.LayerNorm(num_channels, eps=eps)

    def forward(self, x):
        b, c, t, f = x.shape
        x = x.permute(0, 2, 3, 1)
        x = self.norm(x)
        return x.permute(0, 3, 1, 2)


def _extract_conv_norm_act(seq: nn.Sequential) -> tuple[nn.Conv2d, nn.Module, nn.Module]:
    conv = None
    norm = None
    act = None
    for module in seq:
        if isinstance(module, nn.Conv2d):
            conv = module
        elif isinstance(
            module,
            (ChannelLayerNorm2d, nn.InstanceNorm2d, nn.LayerNorm, nn.BatchNorm2d),
        ):
            norm = module
        elif isinstance(module, nn.PReLU):
            act = module
    if conv is None or norm is None or act is None:
        raise ValueError("Dense block expects Conv2d + Norm + PReLU in each layer.")
    return conv, norm, act


def _get_causal_padding_2d(kernel_size, dilation=(1, 1)):
    pad_t = kernel_size[0] * dilation[0] - dilation[0]
    pad_f = (kernel_size[1] * dilation[1] - dilation[1]) // 2
    return (pad_f, pad_f, pad_t, 0)


class StreamingCausalConv2d(nn.Module):
    """Causal Conv2d wrapper for frame-by-frame streaming."""

    def __init__(self, conv: nn.Conv2d):
        super().__init__()
        self.__dict__["conv"] = conv
        kt, kf = conv.kernel_size
        dt, df = conv.dilation
        self.cache_len = max((kt - 1) * dt, 0)
        self.register_buffer("cache", None, persistent=False)
        self.freq_pad = _get_causal_padding_2d((1, kf), (dt, df))

    def reset(self):
        self.cache = None

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # Expect x: [B, C, 1, F] for streaming.
        if self.cache_len == 0:
            return self.conv(F.pad(x, self.freq_pad, "constant", 0))

        b, c, _, f = x.shape
        if self.cache is None:
            self.cache = torch.zeros(
                b, c, self.cache_len, f, device=x.device, dtype=x.dtype
            )

        x_cat = torch.cat([self.cache, x], dim=2)
        y = self.conv(F.pad(x_cat, self.freq_pad, "constant", 0))
        self.cache = x_cat[:, :, -self.cache_len :, :]
        return y


class DenseBlock(nn.Module):
    """Dense block with causal padding along time axis."""

    def __init__(self, cfg, kernel_size=(2, 3), depth=4):
        super(DenseBlock, self).__init__()
        self.cfg = cfg
        self.depth = depth
        self.dense_block = nn.ModuleList()
        self._streaming_enabled = False
        self._streaming_convs = nn.ModuleList()
        self._streaming_norms = []
        self._streaming_acts = []
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        for i in range(self.depth):
            dilation = 2 ** i
            pad_length = dilation
            dense_conv = nn.Sequential(
                nn.ConstantPad2d((1, 1, pad_length, 0), value=0.0),
                nn.Conv2d(self.hid_feature * (i + 1), self.hid_feature, kernel_size, dilation=(dilation, 1)),
                ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
                nn.PReLU(self.hid_feature),
            )
            self.dense_block.append(dense_conv)
            conv, norm, act = _extract_conv_norm_act(dense_conv)
            self._streaming_convs.append(StreamingCausalConv2d(conv))
            self._streaming_norms.append(norm)
            self._streaming_acts.append(act)

    def set_streaming(self, enabled: bool) -> None:
        self._streaming_enabled = enabled
        if enabled:
            self.reset_streaming()

    def reset_streaming(self) -> None:
        for conv in self._streaming_convs:
            conv.reset()

    def forward(self, x):
        if self._streaming_enabled and x.shape[2] == 1:
            skip = x
            y = None
            for idx, conv in enumerate(self._streaming_convs):
                y = conv(skip)
                y = self._streaming_norms[idx](y)
                y = self._streaming_acts[idx](y)
                skip = torch.cat([y, skip], dim=1)
            return y
        skip = x
        for i in range(self.depth):
            x = self.dense_block[i](skip)
            skip = torch.cat([x, skip], dim=1)
        return x


class DenseEncoder_old(nn.Module):
    """Legacy dense encoder retained for compatibility."""

    def __init__(self, cfg):
        super(DenseEncoder_old, self).__init__()
        self.cfg = cfg
        self.input_channel = cfg["model_cfg"]["input_channel"]
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        self.dense_conv_1 = nn.Sequential(
            nn.Conv2d(self.input_channel, self.hid_feature, (1, 1)),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
        )

        self.dense_block = DenseBlock(cfg, depth=4)

        self.dense_conv_2 = nn.Sequential(
            nn.Conv2d(self.hid_feature, self.hid_feature, (1, 3), stride=(1, 2)),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
        )

    def forward(self, x):
        x = self.dense_conv_1(x)
        x = self.dense_block(x)
        x = self.dense_conv_2(x)
        return x


class DenseEncoder(nn.Module):
    """Primary dense encoder used by the generator."""

    def __init__(self, cfg):
        super(DenseEncoder, self).__init__()
        self.cfg = cfg
        self.input_channel = cfg["model_cfg"]["input_channel"]
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        self.dense_conv_1 = nn.Sequential(
            nn.Conv2d(self.input_channel, self.hid_feature, (1, 1)),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
        )

        self.dense_block = DenseBlock(cfg, depth=4)

        self.dense_conv_2 = nn.Sequential(
            nn.Conv2d(self.hid_feature, self.hid_feature, (1, 3), (1, 2), padding=(0, 1)),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
        )

    def forward(self, x):
        x = self.dense_conv_1(x)
        x = self.dense_block(x)
        x = self.dense_conv_2(x)
        return x

    def set_streaming(self, enabled: bool) -> None:
        self.dense_block.set_streaming(enabled)

    def reset_streaming(self) -> None:
        self.dense_block.reset_streaming()


class MagDecoder_old(nn.Module):
    """Legacy magnitude decoder retained for compatibility."""

    def __init__(self, cfg):
        super(MagDecoder_old, self).__init__()
        self.dense_block = DenseBlock(cfg, depth=4)
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.output_channel = cfg["model_cfg"]["output_channel"]
        self.n_fft = cfg["stft_cfg"]["n_fft"]
        self.beta = cfg["model_cfg"]["beta"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        self.mask_conv = nn.Sequential(
            nn.ConvTranspose2d(self.hid_feature, self.hid_feature, (1, 3), stride=(1, 2)),
            nn.Conv2d(self.hid_feature, self.output_channel, (1, 1)),
            ChannelLayerNorm2d(self.output_channel, eps=self.norm_epsilon),
            nn.PReLU(self.output_channel),
            nn.Conv2d(self.output_channel, self.output_channel, (1, 1)),
        )
        self.lsigmoid = LearnableSigmoid2D(self.n_fft // 2 + 1, beta=self.beta)

    def forward(self, x):
        x = self.dense_block(x)
        x = self.mask_conv(x)
        x = rearrange(x, "b c t f -> b f t c").squeeze(-1)
        x = self.lsigmoid(x)
        x = rearrange(x, "b f t -> b t f").unsqueeze(1)
        return x


class PhaseDecoder_old(nn.Module):
    """Legacy phase decoder retained for compatibility."""

    def __init__(self, cfg):
        super(PhaseDecoder_old, self).__init__()
        self.dense_block = DenseBlock(cfg, depth=4)
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.output_channel = cfg["model_cfg"]["output_channel"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        self.phase_conv = nn.Sequential(
            nn.ConvTranspose2d(self.hid_feature, self.hid_feature, (1, 3), stride=(1, 2)),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
        )

        self.phase_conv_r = nn.Conv2d(self.hid_feature, self.output_channel, (1, 1))
        self.phase_conv_i = nn.Conv2d(self.hid_feature, self.output_channel, (1, 1))

    def forward(self, x):
        x = self.dense_block(x)
        x = self.phase_conv(x)
        x_r = self.phase_conv_r(x)
        x_i = self.phase_conv_i(x)
        x = torch.atan2(x_i, x_r)
        return x


class SPConvTranspose2d(nn.Module):
    """Sub-pixel transposed convolution used for frequency upsampling."""

    def __init__(self, in_channels, out_channels, kernel_size, r=1):
        super(SPConvTranspose2d, self).__init__()
        self.pad1 = nn.ConstantPad2d((1, 1, 0, 0), value=0.0)
        self.out_channels = out_channels
        self.conv = nn.Conv2d(in_channels, out_channels * r, kernel_size=kernel_size, stride=(1, 1))
        self.r = r

    def forward(self, x):
        x = self.pad1(x)
        out = self.conv(x)
        batch_size, nchannels, h, w = out.shape
        out = out.view((batch_size, self.r, nchannels // self.r, h, w))
        out = out.permute(0, 2, 3, 4, 1)
        out = out.contiguous().view((batch_size, nchannels // self.r, h, -1))
        return out


class MagDecoder(nn.Module):
    """Mask-based magnitude decoder."""

    def __init__(self, cfg):
        super(MagDecoder, self).__init__()
        self.dense_block = DenseBlock(cfg, depth=4)
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.output_channel = cfg["model_cfg"]["output_channel"]
        self.n_fft = cfg["stft_cfg"]["n_fft"]
        self.beta = cfg["model_cfg"]["beta"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        self.mask_conv = nn.Sequential(
            SPConvTranspose2d(self.hid_feature, self.hid_feature, (1, 3), 2),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
            nn.Conv2d(self.hid_feature, self.output_channel, (1, 2)),
        )
        self.lsigmoid = LearnableSigmoid2D(self.n_fft // 2 + 1, beta=self.beta)

    def forward(self, x):
        x = self.dense_block(x)
        x = self.mask_conv(x)
        x = x.permute(0, 3, 2, 1).squeeze(-1)
        x = self.lsigmoid(x)
        x = x.permute(0, 2, 1).unsqueeze(1)
        return x

    def set_streaming(self, enabled: bool) -> None:
        self.dense_block.set_streaming(enabled)

    def reset_streaming(self) -> None:
        self.dense_block.reset_streaming()


class MapMagDecoder(nn.Module):
    """Mapping-based magnitude decoder with Softplus output."""

    def __init__(self, cfg):
        super(MapMagDecoder, self).__init__()
        self.dense_block = DenseBlock(cfg, depth=4)
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.output_channel = cfg["model_cfg"]["output_channel"]
        self.n_fft = cfg["stft_cfg"]["n_fft"]
        self.beta = cfg["model_cfg"]["beta"]
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]

        self.mask_conv = nn.Sequential(
            SPConvTranspose2d(self.hid_feature, self.hid_feature, (1, 3), 2),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
            nn.Conv2d(self.hid_feature, self.output_channel, (1, 2)),
        )

        self.lsigmoid = nn.Softplus(beta=1, threshold=20)

    def forward(self, x):
        x = self.dense_block(x)
        x = self.mask_conv(x)
        x = x.permute(0, 3, 2, 1).squeeze(-1)
        x = self.lsigmoid(x)
        x = x.permute(0, 2, 1).unsqueeze(1)
        return x

    def set_streaming(self, enabled: bool) -> None:
        self.dense_block.set_streaming(enabled)

    def reset_streaming(self) -> None:
        self.dense_block.reset_streaming()


class PhaseDecoder(nn.Module):
    """Phase decoder that reconstructs phase via atan2."""

    def __init__(self, cfg):
        super(PhaseDecoder, self).__init__()
        self.hid_feature = cfg["model_cfg"]["hid_feature"]
        self.output_channel = cfg["model_cfg"]["output_channel"]
        self.dense_block = DenseBlock(cfg, depth=4)
        self.norm_epsilon = cfg["model_cfg"]["norm_epsilon"]
        self.phase_conv = nn.Sequential(
            SPConvTranspose2d(self.hid_feature, self.hid_feature, (1, 3), 2),
            ChannelLayerNorm2d(self.hid_feature, eps=self.norm_epsilon),
            nn.PReLU(self.hid_feature),
        )
        self.phase_conv_r = nn.Conv2d(self.hid_feature, self.output_channel, (1, 2))
        self.phase_conv_i = nn.Conv2d(self.hid_feature, self.output_channel, (1, 2))

    def forward(self, x):
        x = self.dense_block(x)
        x = self.phase_conv(x)
        x_r = self.phase_conv_r(x)
        x_i = self.phase_conv_i(x)
        x = torch.atan2(x_i, x_r)
        return x

    def set_streaming(self, enabled: bool) -> None:
        self.dense_block.set_streaming(enabled)

    def reset_streaming(self) -> None:
        self.dense_block.reset_streaming()
