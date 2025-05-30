# Copyright (c) Facebook, Inc. and its affiliates.
# All rights reserved.
#
# This source code is licensed under the license found in the
# LICENSE file in the root directory of this source tree.
# author: adefossez, balkce

import math
import time

import torch as th
from torch import nn
from torch.nn import functional as F

from .audio import extract_interf, combine_interf
from .resample import downsample2, upsample2
from .utils import capture_init

class BLSTM(nn.Module):
    def __init__(self, dim, layers=2, bi=True, mix=False):
        super().__init__()
        klass = nn.LSTM
        self.lstm = klass(bidirectional=bi, num_layers=layers, hidden_size=dim, input_size=dim)
        self.linear = None
        if bi:
            self.linear = nn.Linear(2 * dim, dim)

    def forward(self, x, hidden=None):
        x, hidden = self.lstm(x, hidden)
        if self.linear:
            x = self.linear(x)
        return x, hidden


def rescale_conv(conv, reference):
    std = conv.weight.std().detach()
    scale = (std / reference)**0.5
    conv.weight.data /= scale
    if conv.bias is not None:
        conv.bias.data /= scale


def rescale_module(module, reference):
    for sub in module.modules():
        if isinstance(sub, (nn.Conv1d, nn.ConvTranspose1d)):
            rescale_conv(sub, reference)


class Demucs(nn.Module):
    """
    Demucs speech enhancement model.
    Args:
        - chin (int): number of input channels.
        - chout (int): number of output channels.
        - hidden (int): number of initial hidden channels.
        - lstm_depth (int): number of BLSTM layers.
        - depth (int): number of layers.
        - kernel_size (int): kernel size for each layer.
        - stride (int): stride for each layer.
        - causal (bool): if false, uses BiLSTM instead of LSTM.
        - resample (int): amount of resampling to apply to the input/output.
            Can be one of 1, 2 or 4.
        - growth (float): number of channels is multiplied by this for every layer.
        - max_hidden (int): maximum number of channels. Can be useful to
            control the size/speed of the model.
        - normalize (bool): if true, normalize the input.
        - glu (bool): if true uses GLU instead of ReLU in 1x1 convolutions.
        - rescale (float): controls custom weight initialization.
            See https://arxiv.org/abs/1911.13254.
        - floor (float): stability flooring when normalizing.
        - sample_rate (float): sample_rate used for training the model.

    """
    @capture_init
    def __init__(self,
                 chin=1,
                 chout=1,
                 hidden=48,
                 lstm_depth=2,
                 depth=5,
                 kernel_size=8,
                 stride=4,
                 causal=True,
                 resample=4,
                 growth=2,
                 max_hidden=10_000,
                 normalize=True,
                 glu=True,
                 rescale=0.1,
                 floor=1e-3,
                 sample_rate=16_000):

        super().__init__()
        if resample not in [1, 2, 4]:
            raise ValueError("Resample should be 1, 2 or 4.")

        self.chin = chin
        self.chout = chout
        self.hidden = hidden
        self.depth = depth
        self.lstm_depth = lstm_depth
        self.kernel_size = kernel_size
        self.stride = stride
        self.causal = causal
        self.floor = floor
        self.resample = resample
        self.normalize = normalize
        self.sample_rate = sample_rate

        self.encoder = nn.ModuleList()
        self.encoder_int = nn.ModuleList()
        self.decoder = nn.ModuleList()
        self.decoder_int = nn.ModuleList()
        activation = nn.GLU(1) if glu else nn.ReLU()
        ch_scale = 2 if glu else 1

        for index in range(depth):
            encode = []
            encode += [
                nn.Conv1d(chin, hidden, kernel_size, stride),
                nn.ReLU(),
                nn.Conv1d(hidden, hidden * ch_scale, 1), activation,
            ]
            self.encoder.append(nn.Sequential(*encode))
            
            encode_int = []
            encode_int += [
                nn.Conv1d(chin, hidden, kernel_size, stride),
                nn.ReLU(),
                nn.Conv1d(hidden, hidden * ch_scale, 1), activation,
            ]
            self.encoder_int.append(nn.Sequential(*encode_int))
            
            decode = []
            decode += [
                nn.Conv1d(hidden, ch_scale * hidden, 1), activation,
                nn.ConvTranspose1d(hidden, chout, kernel_size, stride),
            ]
            if index > 0:
                decode.append(nn.ReLU())
            self.decoder.insert(0, nn.Sequential(*decode))
            
            decode_int = []
            decode_int += [
                nn.Conv1d(hidden, ch_scale * hidden, 1), activation,
                nn.ConvTranspose1d(hidden, chout, kernel_size, stride),
            ]
            if index > 0:
                decode_int.append(nn.ReLU())
            self.decoder_int.insert(0, nn.Sequential(*decode_int))
            
            chout = hidden
            chin = hidden
            hidden = min(int(growth * hidden), max_hidden)

        self.lstm_can = BLSTM(chin*2, layers=lstm_depth, bi=not causal)
        if rescale:
            rescale_module(self, reference=rescale)

    def valid_length(self, length):
        """
        Return the nearest valid length to use with the model so that
        there is no time steps left over in a convolutions, e.g. for all
        layers, size of the input - kernel_size % stride = 0.

        If the mixture has a valid length, the estimated sources
        will have exactly the same length.
        """
        length = math.ceil(length * self.resample)
        for idx in range(self.depth):
            length = math.ceil((length - self.kernel_size) / self.stride) + 1
            length = max(length, 1)
        for idx in range(self.depth):
            length = (length - 1) * self.stride + self.kernel_size
        length = int(math.ceil(length / self.resample))
        return int(length)

    @property
    def total_stride(self):
        return self.stride ** self.depth // self.resample

    def forward(self, mix_interf):
        mix, interf = extract_interf(mix_interf)
        #print(str(mix_interf.shape)+" -> "+str(interf.shape)+", "+str(mix.shape))
        
        if mix.dim() == 2:
            mix = mix.unsqueeze(1)
        if interf.dim() == 2:
            interf = interf.unsqueeze(1)

        if self.normalize:
            mono = mix.mean(dim=1, keepdim=True)
            std = mono.std(dim=-1, keepdim=True)
            mix = mix / (self.floor + std)
            
            mono = interf.mean(dim=1, keepdim=True)
            std_i = mono.std(dim=-1, keepdim=True)
            interf = interf / (self.floor + std_i)
        else:
            std = 1
            std_i = 1
        
        length = mix.shape[-1]
        
        x = mix
        i = interf
        #print(" -> "+str(x.shape))
        
        x = F.pad(x, (0, self.valid_length(length) - length))
        i = F.pad(i, (0, self.valid_length(length) - length))
        
        if self.resample == 2:
            x = upsample2(x)
            i = upsample2(i)
        elif self.resample == 4:
            x = upsample2(x)
            x = upsample2(x)
            i = upsample2(i)
            i = upsample2(i)
        
        skips_x = []
        for encode in self.encoder:
            x = encode(x)
            skips_x.append(x)
        skips_i = []
        for encode_int in self.encoder_int:
            i = encode_int(i)
            skips_i.append(i)
        
        x = x.permute(2, 0, 1)
        length_precat = x.shape[2]
        
        i = i.permute(2, 0, 1)
        
        #cancelling i from x
        xi, _ = self.lstm_can(th.cat((x,i),2))
        (x, i) = th.split(xi,length_precat,dim=2)
        
        x = x.permute(1, 2, 0)
        i = i.permute(1, 2, 0)
        
        for decode in self.decoder:
            skip_x = skips_x.pop(-1)
            x = x + skip_x[..., :x.shape[-1]]
            x = decode(x)
        for decode_int in self.decoder_int:
            skip_i = skips_i.pop(-1)
            i = i + skip_i[..., :i.shape[-1]]
            i = decode_int(i)
        
        if self.resample == 2:
            x = downsample2(x)
            i = downsample2(i)
        elif self.resample == 4:
            x = downsample2(x)
            x = downsample2(x)
            i = downsample2(i)
            i = downsample2(i)
        
        x = x[..., :length]
        i = i[..., :length]
        return combine_interf(std * x, std_i * i)


def fast_conv(conv, x):
    """
    Faster convolution evaluation if either kernel size is 1
    or length of sequence is 1.
    """
    batch, chin, length = x.shape
    chout, chin, kernel = conv.weight.shape
    assert batch == 1
    if kernel == 1:
        x = x.view(chin, length)
        out = th.addmm(conv.bias.view(-1, 1),
                       conv.weight.view(chout, chin), x)
    elif length == kernel:
        x = x.view(chin * kernel, 1)
        out = th.addmm(conv.bias.view(-1, 1),
                       conv.weight.view(chout, chin * kernel), x)
    else:
        out = conv(x)
    return out.view(batch, chout, -1)


class DemucsStreamer:
    """
    Streaming implementation for Demucs. It supports being fed with any amount
    of audio at a time. You will get back as much audio as possible at that
    point.

    Args:
        - demucs (Demucs): Demucs model.
        - mic_dist (float): distance between microphones.
        - dry (float): amount of dry (e.g. input) signal to keep. 0 is maximum
            noise removal, 1 just returns the input signal. Small values > 0
            allows to limit distortions.
        - num_frames (int): number of frames to process at once. Higher values
            will increase overall latency but improve the real time factor.
        - resample_lookahead (int): extra lookahead used for the resampling.
        - resample_buffer (int): size of the buffer of previous inputs/outputs
            kept for resampling.
    """
    def __init__(self, demucs,
                 dry=0,
                 num_frames=1,
                 resample_lookahead=64,
                 resample_buffer=256):
        device = next(iter(demucs.parameters())).device
        self.demucs = demucs
        self.device = device
        
        self.lstm_state_can = None
        self.conv_state_x = None
        self.conv_state_i = None
        self.dry = dry
        self.resample_lookahead = resample_lookahead
        resample_buffer = min(demucs.total_stride, resample_buffer)
        self.resample_buffer = resample_buffer
        self.frame_length = demucs.valid_length(1) + demucs.total_stride * (num_frames - 1)
        self.total_length = self.frame_length + self.resample_lookahead
        self.stride = demucs.total_stride * num_frames
        self.resample_in = th.zeros(demucs.chin*2, resample_buffer, device=device)
        self.resample_out = th.zeros(demucs.chin, resample_buffer, device=device)

        self.frames = 0
        self.total_time = 0
        self.variance = 0
        self.pending = th.zeros(demucs.chin*2, 0, device=device) #making it stereo

        bias = demucs.decoder[0][2].bias
        weight = demucs.decoder[0][2].weight
        chin, chout, kernel = weight.shape
        self._bias = bias.view(-1, 1).repeat(1, kernel).view(-1, 1)
        self._weight = weight.permute(1, 2, 0).contiguous()
        
        self.phase_threshold = 30

    def do_phasebeamform(self,stereo_frame):
        mic1_fft = th.fft.fft(stereo_frame[0,:])
        mic2_fft = th.fft.fft(stereo_frame[1,:])
        
        angle_diffs = th.abs(th.angle(mic1_fft) - th.angle(mic2_fft))
        angle_diffs *= 180/3.14159
        angle_diffs[angle_diffs > 180] = th.abs(360-angle_diffs[angle_diffs > 180])
        
        mic1_fft[angle_diffs > self.phase_threshold] = 0.0
        mic2_fft[angle_diffs <= self.phase_threshold] = 0.0
        
        stereo_frame[0,:] = th.real(th.fft.ifft(mic1_fft))
        stereo_frame[1,:] = th.real(th.fft.ifft(mic2_fft))
        
        return stereo_frame

    def reset_time_per_frame(self):
        self.total_time = 0
        self.frames = 0

    @property
    def time_per_frame(self):
        return self.total_time / self.frames

    def flush(self):
        """
        Flush remaining audio by padding it with zero and initialize the previous
        status. Call this when you have no more input and want to get back the last
        chunk of audio.
        """
        self.lstm_state_can = None
        self.conv_state_x = None
        self.conv_state_i = None
        pending_length = self.pending.shape[1]
        padding = th.zeros(self.demucs.chin, self.total_length, device=self.pending.device)
        out = self.feed(padding)
        return out[:, :pending_length]

    def feed(self, wav):
        """
        Apply the model to mix using true real time evaluation.
        Normalization is done online as is the resampling.
        Assume a stereo input (wav.shape == [2,:])
        """
        begin = time.time()
        demucs = self.demucs
        resample_buffer = self.resample_buffer
        stride = self.stride
        resample = demucs.resample

        if wav.dim() != 2:
            raise ValueError("input wav should be two dimensional.")
        chin, _ = wav.shape
        if chin != 2*demucs.chin:
            raise ValueError(f"Expected {demucs.chin} channels, got {chin}")

        self.pending = th.cat([self.pending, wav], dim=1)
        outs = []
        while self.pending.shape[1] >= self.total_length:
            self.frames += 1
            frame = self.pending[:, :self.total_length]
            dry_signal = frame[0, :stride]
            if demucs.normalize:
                mono = frame.mean(0)
                variance = (mono**2).mean()
                self.variance = variance / self.frames + (1 - 1 / self.frames) * self.variance
                frame = frame / (demucs.floor + math.sqrt(self.variance))
            
            frame_orig = th.Tensor(frame)
            frame = self.do_phasebeamform(frame) # carry out phase beamform
            
            padded_frame = th.cat([self.resample_in, frame], dim=-1)
            self.resample_in[:] = frame[:, stride - resample_buffer:stride]
            frame = padded_frame

            if resample == 4:
                frame = upsample2(upsample2(frame))
            elif resample == 2:
                frame = upsample2(frame)
            frame = frame[:, resample * resample_buffer:]  # remove pre sampling buffer
            frame = frame[:, :resample * self.frame_length]  # remove extra samples after window
            frame_interf = combine_interf(frame[0,:][None][None],frame[1,:][None][None])

            out, extra = self._separate_frame(frame_interf)
            padded_out = th.cat([self.resample_out, out, extra], 1)
            self.resample_out[:] = out[:, -resample_buffer:]
            if resample == 4:
                out = downsample2(downsample2(padded_out))
            elif resample == 2:
                out = downsample2(padded_out)
            else:
                out = padded_out

            out = out[:, resample_buffer // resample:]
            out = out[:, :stride]

            if demucs.normalize:
                out *= math.sqrt(self.variance)
            out = self.dry * dry_signal + (1 - self.dry) * out
            outs.append(out)
            self.pending = self.pending[:, stride:]

        self.total_time += time.time() - begin
        if outs:
            out = th.cat(outs, 1)
        else:
            out = th.zeros(chin, 0, device=wav.device)
        return out

    def _separate_frame(self, signal_w_interf):
        frame, interf = extract_interf (signal_w_interf)
        frame = frame.squeeze(0)
        interf = interf.squeeze(0)
        demucs = self.demucs
        skips_x = []
        skips_i = []
        next_state_x = []
        next_state_i = []
        first = self.conv_state_x is None
        stride = self.stride * demucs.resample
        
        x = frame[None]
        for idx, encode in enumerate(demucs.encoder):
            stride //= demucs.stride
            length = x.shape[2]
            if idx == demucs.depth - 1:
                # This is sligthly faster for the last conv
                x = fast_conv(encode[0], x)
                x = encode[1](x)
                x = fast_conv(encode[2], x)
                x = encode[3](x)
            else:
                if not first:
                    prev_x = self.conv_state_x.pop(0)
                    prev_x = prev_x[..., stride:]
                    tgt = (length - demucs.kernel_size) // demucs.stride + 1
                    missing = tgt - prev_x.shape[-1]
                    offset = length - demucs.kernel_size - demucs.stride * (missing - 1)
                    x = x[..., offset:]
                x = encode[1](encode[0](x))
                x = fast_conv(encode[2], x)
                x = encode[3](x)
                if not first:
                    x = th.cat([prev_x, x], -1)
                next_state_x.append(x)
            skips_x.append(x)
        
        i = interf[None]
        for idx, encode_int in enumerate(demucs.encoder_int):
            stride //= demucs.stride
            length = i.shape[2]
            if idx == demucs.depth - 1:
                # This is sligthly faster for the last conv
                i = fast_conv(encode_int[0], i)
                i = encode_int[1](i)
                i = fast_conv(encode_int[2], i)
                i = encode_int[3](i)
            else:
                if not first:
                    prev_i = self.conv_state_i.pop(0)
                    prev_i = prev_i[..., stride:]
                    tgt = (length - demucs.kernel_size) // demucs.stride + 1
                    missing = tgt - prev_i.shape[-1]
                    offset = length - demucs.kernel_size - demucs.stride * (missing - 1)
                    i = i[..., offset:]
                i = encode_int[1](encode_int[0](i))
                i = fast_conv(encode_int[2], i)
                i = encode_int[3](i)
                if not first:
                    i = th.cat([prev_i, i], -1)
                next_state_i.append(i)
            skips_i.append(i)
        
        #print("  > "+str(x.shape))
        
        x = x.permute(2, 0, 1)
        
        i = i.permute(2, 0, 1)
        
        xi, self.lstm_state_can = demucs.lstm_can(th.cat((x,i),2), self.lstm_state_can)
        (x, i) = th.split(xi,length,dim=2)
        
        x = x.permute(1, 2, 0)
        i = i.permute(1, 2, 0)
        
        # In the following, x contains only correct samples, i.e. the one
        # for which each time position is covered by two window of the upper layer.
        # extra contains extra samples to the right, and is used only as a
        # better padding for the online resampling.
        extra_x = None
        extra_i = None
        for idx, decode in enumerate(demucs.decoder):
            decode_int = demucs.decoder_int[idx]
            
            skip_x = skips_x.pop(-1)
            x += skip_x[..., :x.shape[-1]]
            x = fast_conv(decode[0], x)
            x = decode[1](x)
            
            skip_i = skips_i.pop(-1)
            i += skip_i[..., :i.shape[-1]]
            i = fast_conv(decode_int[0], i)
            i = decode_int[1](i)
            
            if extra_x is not None:
                skip_ex = skip_x[..., x.shape[-1]:]
                extra_x += skip_ex[..., :extra_x.shape[-1]]
                extra_x = decode[2](decode[1](decode[0](extra_x)))
            x = decode[2](x)
            next_state_x.append(x[..., -demucs.stride:] - decode[2].bias.view(-1, 1))
            if extra_x is None:
                extra_x = x[..., -demucs.stride:]
            else:
                extra_x[..., :demucs.stride] += next_state_x[-1]
            
            if extra_i is not None:
                skip_ei = skip_i[..., i.shape[-1]:]
                extra_i += skip_ei[..., :extra_i.shape[-1]]
                extra_i = decode_int[2](decode_int[1](decode_int[0](extra_i)))
            i = decode_int[2](i)
            next_state_i.append(i[..., -demucs.stride:] - decode_int[2].bias.view(-1, 1))
            if extra_i is None:
                extra_i = i[..., -demucs.stride:]
            else:
                extra_i[..., :demucs.stride] += next_state_i[-1]
            
            x = x[..., :-demucs.stride]
            i = i[..., :-demucs.stride]

            if not first:
                prev_x = self.conv_state_x.pop(0)
                x[..., :demucs.stride] += prev_x
                prev_i = self.conv_state_i.pop(0)
                i[..., :demucs.stride] += prev_i
            if idx != demucs.depth - 1:
                x = decode[3](x)
                extra_x = decode[3](extra_x)
                i = decode_int[3](i)
                extra_i = decode_int[3](extra_i)
        self.conv_state_x = next_state_x
        self.conv_state_i = next_state_i
        return x[0], extra_x[0]


def test():
    import argparse
    parser = argparse.ArgumentParser(
        "denoiser.demucs",
        description="Benchmark the streaming Demucs implementation, "
                    "as well as checking the delta with the offline implementation.")
    parser.add_argument("--depth", default=5, type=int)
    parser.add_argument("--resample", default=4, type=int)
    parser.add_argument("--hidden", default=48, type=int)
    parser.add_argument("--sample_rate", default=16000, type=float)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("-t", "--num_threads", type=int)
    parser.add_argument("-f", "--num_frames", type=int, default=1)
    args = parser.parse_args()
    if args.num_threads:
        th.set_num_threads(args.num_threads)
    sr = args.sample_rate
    sr_ms = sr / 1000
    demucs = Demucs(depth=args.depth, hidden=args.hidden, resample=args.resample).to(args.device)
    x = th.randn(1, int(sr * 4)).to(args.device)
    out = demucs(x[None])[0]
    streamer = DemucsStreamer(demucs, num_frames=args.num_frames)
    out_rt = []
    frame_size = streamer.total_length
    with th.no_grad():
        while x.shape[1] > 0:
            out_rt.append(streamer.feed(x[:, :frame_size]))
            x = x[:, frame_size:]
            frame_size = streamer.demucs.total_stride
    out_rt.append(streamer.flush())
    out_rt = th.cat(out_rt, 1)
    model_size = sum(p.numel() for p in demucs.parameters()) * 4 / 2**20
    initial_lag = streamer.total_length / sr_ms
    tpf = 1000 * streamer.time_per_frame
    print(f"model size: {model_size:.1f}MB, ", end='')
    print(f"delta batch/streaming: {th.norm(out - out_rt) / th.norm(out):.2%}")
    print(f"initial lag: {initial_lag:.1f}ms, ", end='')
    print(f"stride: {streamer.stride * args.num_frames / sr_ms:.1f}ms")
    print(f"time per frame: {tpf:.1f}ms, ", end='')
    print(f"RTF: {((1000 * streamer.time_per_frame) / (streamer.stride / sr_ms)):.2f}")
    print(f"Total lag with computation: {initial_lag + tpf:.1f}ms")


if __name__ == "__main__":
    test()
