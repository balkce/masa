from __future__ import annotations

import math
from typing import Optional, Tuple

import torch


class StreamingSTFT:
    """Incremental STFT with center=False and hop-based framing."""

    def __init__(self, n_fft: int, hop_size: int, win_size: int, compress_factor: float = 1.0, device: Optional[torch.device] = None):
        self.n_fft = n_fft
        self.hop_size = hop_size
        self.win_size = win_size
        self.compress_factor = compress_factor
        self.device = device if device is not None else torch.device("cpu")
        self.eps = 1e-10
        #self.window = torch.hann_window(win_size, device=self.device) + 1e-3
        self.window = torch.sqrt(torch.hann_window(win_size, device=self.device) + 1e-3)
        self.tail = torch.zeros(0, device=self.device)

    def process(self, audio_chunk: torch.Tensor) -> Optional[Tuple[torch.Tensor, torch.Tensor, torch.Tensor]]:
        """
        Consume a chunk of audio samples and return STFT features for any full frames that fit.

        Args:
            audio_chunk: 1D tensor of shape (samples,) or (1, samples)
        Returns:
            mag, pha, com tensors shaped (1, F, T_chunk) for the newly available frames, or None if not enough samples yet.
        """
        if audio_chunk.dim() == 2:
            audio_chunk = audio_chunk.squeeze(0)
        audio_chunk = audio_chunk.to(self.device)
        data = torch.cat([self.tail, audio_chunk], dim=-1)

        if data.numel() < self.win_size:
            self.tail = data
            return None

        num_frames = (data.numel() - self.win_size) // self.hop_size + 1
        frames = data.unfold(0, self.win_size, self.hop_size)[:num_frames]
        self.tail = data[num_frames * self.hop_size :]

        windowed = frames * self.window
        spec = torch.fft.rfft(windowed, n=self.n_fft, dim=-1)
        mag = torch.abs(spec)
        pha = torch.angle(spec)
        mag = torch.pow(torch.clamp(mag, min=self.eps), self.compress_factor)
        com = torch.stack((mag * torch.cos(pha), mag * torch.sin(pha)), dim=-1)

        # reshape to (1, F, T)
        mag = mag.transpose(0, 1).unsqueeze(0)
        pha = pha.transpose(0, 1).unsqueeze(0)
        com = com.permute(1, 0, 2).unsqueeze(0)
        return mag, pha, com

    def flush(self) -> Optional[Tuple[torch.Tensor, torch.Tensor, torch.Tensor]]:
        """
        Process any remaining samples in the tail buffer by zero-padding to win_size.
        This ensures all input samples are processed, preventing clicks from lost samples.
        
        Returns:
            mag, pha, com tensors shaped (1, F, T) for the final frame, or None if tail is empty.
        """
        if self.tail.numel() == 0:
            return None
        
        # Zero-pad tail to win_size to create a final frame
        padding_needed = self.win_size - self.tail.numel()
        if padding_needed > 0:
            padded_tail = torch.cat([self.tail, torch.zeros(padding_needed, device=self.device)], dim=-1)
        else:
            padded_tail = self.tail[:self.win_size]
        
        # Process the final frame
        windowed = padded_tail * self.window
        spec = torch.fft.rfft(windowed.unsqueeze(0), n=self.n_fft, dim=-1)
        mag = torch.abs(spec)
        pha = torch.angle(spec)
        mag = torch.pow(torch.clamp(mag, min=self.eps), self.compress_factor)
        com = torch.stack((mag * torch.cos(pha), mag * torch.sin(pha)), dim=-1)
        
        # Clear tail after processing
        self.tail = torch.zeros(0, device=self.device)
        
        # reshape to (1, F, T)
        mag = mag.transpose(0, 1).unsqueeze(0)
        pha = pha.transpose(0, 1).unsqueeze(0)
        com = com.permute(1, 0, 2).unsqueeze(0)
        return mag, pha, com


class StreamingISTFT:
    """Overlap-add ISTFT that emits samples incrementally."""

    def __init__(
        self,
        n_fft: int,
        hop_size: int,
        win_size: int,
        compress_factor: float = 1.0,
        device: Optional[torch.device] = None,
        fade_in: bool = False,
        min_win_sum: float = 1e-8,
    ):
        self.n_fft = n_fft
        self.hop_size = hop_size
        self.win_size = win_size
        self.compress_factor = compress_factor
        self.device = device if device is not None else torch.device("cpu")
        self.fade_in = fade_in
        self.min_win_sum = min_win_sum
        self.eps = 1e-10
        #self.window = torch.hann_window(win_size, device=self.device) + 1e-3
        self.window = torch.sqrt(torch.hann_window(win_size, device=self.device) + 1e-3)
        self.window_sq = self.window * self.window
        self.buffer = torch.zeros(0, device=self.device)
        self.win_sum = torch.zeros(0, device=self.device)
        self.processed_frames = 0

    def _ensure_capacity(self, length: int):
        if self.buffer.numel() >= length:
            return
        new_len = max(length, int(self.buffer.numel() * 1.5) + 1)
        pad = new_len - self.buffer.numel()
        self.buffer = torch.cat([self.buffer, torch.zeros(pad, device=self.device)], dim=0)
        self.win_sum = torch.cat([self.win_sum, torch.zeros(pad, device=self.device)], dim=0)

    def add_frames(self, mag: torch.Tensor, pha: torch.Tensor):
        """
        Add a batch of frames to the output buffer.
        Args:
            mag: (1, F, T_new)
            pha: (1, F, T_new)
        """
        mag = mag.to(self.device)
        pha = pha.to(self.device)
        mag = torch.pow(torch.clamp(mag, min=self.eps), 1.0 / self.compress_factor)
        com = torch.complex(mag * torch.cos(pha), mag * torch.sin(pha))
        # reshape to (T_new, F)
        com_tf = com.squeeze(0).permute(1, 0)
        frames = torch.fft.irfft(com_tf, n=self.n_fft, dim=-1)  # (T_new, n_fft)
        frames = frames[:, : self.win_size] * self.window

        for idx, frame in enumerate(frames):
            start = (self.processed_frames + idx) * self.hop_size
            end = start + self.win_size
            self._ensure_capacity(end)
            self.buffer[start:end] += frame
            self.win_sum[start:end] += self.window_sq
        self.processed_frames += frames.size(0)

    def finalize(self, length: int | None = None) -> torch.Tensor:
        """
        Return reconstructed waveform normalized by window sum.
        
        Args:
            length: Desired output length. If None, returns full buffer. If specified,
                   truncates to this length. For lengths shorter than buffer, applies
                   a gentle fade-out to prevent clicks from abrupt truncation.
        """
        if self.win_sum.numel() == 0:
            return torch.zeros(0, device=self.device)
        
        # Normalize by window sum
        out = self.buffer / torch.clamp(self.win_sum, min=self.min_win_sum)

        if self.fade_in and out.numel() > 0:
            fade_length = min(self.win_size, out.numel())
            if length is not None:
                fade_length = min(fade_length, length)
            if fade_length > 0:
                fade = torch.linspace(0.0, 1.0, fade_length, device=self.device)
                out[:fade_length] *= fade
        
        if length is not None:
            if length < out.numel():
                # Apply fade-out to the last samples before truncation to prevent clicks
                # Fade over the last min(win_size, length) samples
                fade_length = min(self.win_size, length)
                if fade_length > 0:
                    fade_start = length - fade_length
                    fade = torch.linspace(1.0, 0.0, fade_length, device=self.device)
                    out[fade_start:length] *= fade
                out = out[:length]
            elif length > out.numel():
                # Pad with zeros if requested length is longer
                pad_length = length - out.numel()
                out = torch.cat([out, torch.zeros(pad_length, device=self.device)], dim=0)
        
        return out
