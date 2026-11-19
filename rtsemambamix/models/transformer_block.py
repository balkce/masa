"""Transformer blocks adapted for 2D time-frequency feature maps with rotary position embeddings."""

from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F


def _rotate_half(x: torch.Tensor) -> torch.Tensor:
    """Rotate every other feature dimension."""

    x1 = x[..., ::2]
    x2 = x[..., 1::2]
    return torch.stack((-x2, x1), dim=-1).flatten(-2)


class RotaryEmbedding(nn.Module):
    """Generate rotary positional embeddings on-the-fly."""

    def __init__(self, dim: int, base: float = 10000.0) -> None:
        super().__init__()
        if dim % 2 != 0:
            raise ValueError("Rotary embedding dimension must be even.")

        inv_freq = 1.0 / (base ** (torch.arange(0, dim, 2).float() / dim))
        self.register_buffer("inv_freq", inv_freq, persistent=False)

    def get_cos_sin(self, seq_len: int, device: torch.device, dtype: torch.dtype) -> tuple[torch.Tensor, torch.Tensor]:
        t = torch.arange(seq_len, device=device, dtype=self.inv_freq.dtype)
        freqs = torch.einsum("i,j->ij", t, self.inv_freq)
        emb = torch.cat((freqs, freqs), dim=-1)
        cos = emb.cos().to(dtype)
        sin = emb.sin().to(dtype)
        return cos, sin


def apply_rotary_pos_emb(x: torch.Tensor, cos: torch.Tensor, sin: torch.Tensor) -> torch.Tensor:
    """Apply rotary position embedding to query/key tensors shaped as (B, H, S, D)."""

    cos = cos.unsqueeze(0).unsqueeze(0).to(dtype=x.dtype)
    sin = sin.unsqueeze(0).unsqueeze(0).to(dtype=x.dtype)
    return (x * cos) + (_rotate_half(x) * sin)


class RotarySelfAttention(nn.Module):
    """Multi-head self-attention with RoPE applied to queries and keys."""

    def __init__(self, d_model: int, nhead: int, dropout: float) -> None:
        super().__init__()
        if d_model % nhead != 0:
            raise ValueError("d_model must be divisible by nhead.")

        self.nhead = nhead
        self.head_dim = d_model // nhead
        self.scale = self.head_dim ** -0.5

        self.q_proj = nn.Linear(d_model, d_model, bias=True)
        self.k_proj = nn.Linear(d_model, d_model, bias=True)
        self.v_proj = nn.Linear(d_model, d_model, bias=True)
        self.out_proj = nn.Linear(d_model, d_model, bias=True)

        self.attn_dropout = nn.Dropout(dropout)
        self.out_dropout = nn.Dropout(dropout)

        self.rotary_emb = RotaryEmbedding(self.head_dim)

    def forward(self, x: torch.Tensor, attn_mask: torch.Tensor | None = None) -> torch.Tensor:
        bsz, seq_len, dim = x.size()
        q = self.q_proj(x)
        k = self.k_proj(x)
        v = self.v_proj(x)

        q = q.view(bsz, seq_len, self.nhead, self.head_dim).transpose(1, 2)
        k = k.view(bsz, seq_len, self.nhead, self.head_dim).transpose(1, 2)
        v = v.view(bsz, seq_len, self.nhead, self.head_dim).transpose(1, 2)

        cos, sin = self.rotary_emb.get_cos_sin(seq_len, x.device, x.dtype)
        q = apply_rotary_pos_emb(q, cos, sin)
        k = apply_rotary_pos_emb(k, cos, sin)

        attn_scores = torch.matmul(q, k.transpose(-2, -1)) * self.scale
        if attn_mask is not None:
            attn_scores = attn_scores.masked_fill(~attn_mask, float("-inf"))
        attn_probs = F.softmax(attn_scores, dim=-1)
        attn_probs = self.attn_dropout(attn_probs)

        attn_output = torch.matmul(attn_probs, v)
        attn_output = attn_output.transpose(1, 2).contiguous().view(bsz, seq_len, dim)

        attn_output = self.out_proj(attn_output)
        attn_output = self.out_dropout(attn_output)
        return attn_output


class RotaryTransformerLayer(nn.Module):
    """Transformer encoder layer with RoPE-enhanced self-attention."""

    def __init__(self, d_model: int, nhead: int, dim_feedforward: int, dropout: float) -> None:
        super().__init__()
        self.self_attn = RotarySelfAttention(d_model, nhead, dropout)
        self.linear1 = nn.Linear(d_model, dim_feedforward)
        self.linear2 = nn.Linear(dim_feedforward, d_model)

        self.norm1 = nn.LayerNorm(d_model)
        self.norm2 = nn.LayerNorm(d_model)

        self.dropout = nn.Dropout(dropout)
        self.dropout1 = nn.Dropout(dropout)
        self.dropout2 = nn.Dropout(dropout)

        self.activation = nn.GELU()

    def forward(self, src: torch.Tensor, attn_mask: torch.Tensor | None = None) -> torch.Tensor:
        src2 = self.self_attn(self.norm1(src), attn_mask=attn_mask)
        src = src + self.dropout1(src2)

        src2 = self.linear2(self.dropout(self.activation(self.linear1(self.norm2(src)))))
        src = src + self.dropout2(src2)
        return src


class RotaryTransformerEncoder(nn.Module):
    """Stack of rotary-aware Transformer layers."""

    def __init__(
        self,
        d_model: int,
        nhead: int,
        dim_feedforward: int,
        dropout: float,
        num_layers: int,
    ) -> None:
        super().__init__()
        self.layers = nn.ModuleList(
            RotaryTransformerLayer(d_model, nhead, dim_feedforward, dropout) for _ in range(num_layers)
        )

    def forward(self, src: torch.Tensor, attn_mask: torch.Tensor | None = None) -> torch.Tensor:
        output = src
        for mod in self.layers:
            output = mod(output, attn_mask=attn_mask)
        return output


class TFTransformerBlock(nn.Module):
    """Transformer block that attends across time and frequency axes sequentially."""

    def __init__(self, cfg):
        super().__init__()
        model_cfg = cfg["model_cfg"]
        hid_feature = model_cfg["hid_feature"]

        num_heads = model_cfg.get("transformer_heads", 4)
        dropout = model_cfg.get("transformer_dropout", 0.1)
        num_layers = model_cfg.get("transformer_layers", 1)
        dim_feedforward = model_cfg.get("transformer_ffn_dim", hid_feature * 4)

        self.time_encoder = RotaryTransformerEncoder(
            hid_feature,
            num_heads,
            dim_feedforward,
            dropout,
            num_layers,
        )
        self.freq_encoder = RotaryTransformerEncoder(
            hid_feature,
            num_heads,
            dim_feedforward,
            dropout,
            num_layers,
        )

        self.time_out = nn.Linear(hid_feature, hid_feature)
        self.freq_out = nn.Linear(hid_feature, hid_feature)

        self.time_norm = nn.LayerNorm(hid_feature)
        self.freq_norm = nn.LayerNorm(hid_feature)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """Apply transformer attention along time then frequency with residuals."""

        b, c, t, f = x.shape

        # --- Time attention ---
        x_time = x.permute(0, 3, 2, 1).contiguous().view(b * f, t, c)
        res_time = x_time
        causal_mask = torch.tril(torch.ones((t, t), device=x.device, dtype=torch.bool))
        x_time = self.time_encoder(x_time, attn_mask=causal_mask)
        x_time = self.time_out(x_time)
        x_time = self.time_norm(x_time + res_time)

        # --- Frequency attention ---
        x_freq = x_time.view(b, f, t, c).permute(0, 2, 1, 3).contiguous().view(b * t, f, c)
        res_freq = x_freq
        x_freq = self.freq_encoder(x_freq)
        x_freq = self.freq_out(x_freq)
        x_freq = self.freq_norm(x_freq + res_freq)

        x_freq = x_freq.view(b, t, f, c).permute(0, 3, 1, 2)

        return x + x_freq
