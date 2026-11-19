import rclpy
from rclpy.node import Node

from jack_msgs.msg import JackAudio

import os
import math
import time
import torch
import numpy as np
import sys

import soundfile as sf

import yaml

from threading import Thread
from collections import deque

from ament_index_python.packages import get_package_share_directory

class RTSEMAMBAROSAudio(Node):
  def __init__(self):
    super().__init__('rtsemamba')
    
    self.device = "cuda"
    self.publisher = self.create_publisher(JackAudio, '/jackaudio_filtered', 1000)
    
    self.declare_parameter('input_length', 0.512)
    self.input_length = self.get_parameter('input_length').get_parameter_value().double_value
    self.declare_parameter('n_ctx', 1024)
    self.n_ctx = self.get_parameter('n_ctx').get_parameter_value().integer_value
    if self.n_ctx < 0:
      print("Invalid n_ctx value ("+str(self.n_ctx)+"). Should greater than 0. Defaulting to 1024.")
      self.n_ctx = 1024
    
    this_share_directory = get_package_share_directory('rtsemamba')
    this_base_directory = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(this_share_directory))))
    this_src_directory = os.path.join(this_base_directory,"src","rtsemamba")
    
    self.rtsemamba_modelpath = os.path.join(this_src_directory,"pretrained_model","best.pth")
    self.rtsemamba_configpath = os.path.join(this_src_directory,"pretrained_model","config.yaml")
    
    print("Using the following pretrained model : "+self.rtsemamba_modelpath)
    print("Using the following pretrained config: "+self.rtsemamba_configpath)
    
    with open(self.rtsemamba_configpath, 'r') as config_file:
        cfg=yaml.safe_load(config_file)
    
    self.n_fft, self.hop_size, self.win_size = cfg['stft_cfg']['n_fft'], cfg['stft_cfg']['hop_size'], cfg['stft_cfg']['win_size']
    self.compress_factor = cfg['model_cfg']['compress_factor']
    self.samplerate = cfg['stft_cfg']['sampling_rate']  #16000
    
    self._out_read_pos = 0
    
    torch.manual_seed(cfg['env_setting']['seed'])
    torch.cuda.manual_seed(cfg['env_setting']['seed'])
    if not torch.cuda.is_available():
      print(f"CUDA NOT AVAILABLE!")
      exit()
    
    sys.path.append(this_src_directory)
    from models.generator import SEMamba
    from models.streaming import StreamingSTFT, StreamingISTFT
    self.stft = StreamingSTFT(self.n_fft, self.hop_size, self.win_size, self.compress_factor, self.device)
    self.istft = StreamingISTFT(self.n_fft, self.hop_size, self.win_size, self.compress_factor, self.device, fade_in=True)
    
    self.rtsemamba = SEMamba(cfg).to(self.device)
    assert os.path.isfile(self.rtsemamba_modelpath)
    print("Loading '{}'".format(self.rtsemamba_modelpath))
    state_dict = torch.load(self.rtsemamba_modelpath, map_location=self.device)
    print("Complete.")
    self.rtsemamba.load_state_dict(state_dict['generator'])
    self.rtsemamba.eval()
    #self.rtsemamba.set_streaming(True, cache_seconds=2.0) #using the default cache lengt in seconds
    #self.rtsemamba.reset_streaming_state()
    self.rtsemamba.set_streaming(False)
    
    self.hann_window = torch.hann_window(self.win_size).to(self.device)
    
    self.jack_win_size = 1024 #BIG assumption
    
    max_segment_samples = int(2 * self.samplerate)
    
    # Rolling context buffer: (1, F, n_ctx) tensors, None until first segment
    self._ctx_mag: torch.Tensor | None = None
    self._ctx_pha: torch.Tensor | None = None
    
    self.segment_size = int((self.input_length*self.samplerate))
    
    if self.segment_size > max_segment_samples:
      print(f"WARNING: segment_size ({self.segment_size}) > 2 seconds. "
            f"Capping to {max_segment_samples} samples.")
      self.segment_size = max_segment_samples
    # Validate segment size compatibility
    if self.segment_size < self.win_size:
      print(f"WARNING: segment_size ({self.segment_size}) is smaller than win_size ({self.win_size}). "
            f"This may cause issues with STFT processing. Recommended: segment_size >= {self.win_size}")
    
    #frames = (self.segment_size // self.hop_size) + 1
    #if frames % 4 != 0:
    #  frames += 4 - (frames % 4)
    #  self.segment_size = (frames - 1) * self.hop_size
    
    print(f"input_length  : {self.input_length} seconds")
    print(f"segment_size  : {self.segment_size} samples")
    print(f"jack_win_size : {self.jack_win_size} samples")
    print(f"sample rate   : {self.samplerate} samples/second")
    print(f"n_ctx         : {self.n_ctx} context frames")
    
    self.rtsemamba_ring_in = deque([])
    self.rtsemamba_ring_out = deque([])
    
    self.sf_file_in = sf.SoundFile("out_rtsemamba_in.wav",mode="w",samplerate=self.samplerate,channels=1,subtype='PCM_16')
    self.sf_file_out = sf.SoundFile("out_rtsemamba_out.wav",mode="w",samplerate=self.samplerate,channels=1,subtype='PCM_16')
    
    print("doing initial model test to allocate memory")
    self.rtsemamba_allocate = True
    self.rtsemamba_thread = Thread(target=self.rtsemamba_callback)
    self.rtsemamba_ring_in.extend([0.0]*(self.segment_size))
    self.rtsemamba_thread.start()
    self.rtsemamba_thread.join()
    print("...done")
    
    self.rtsemamba_allocate = False
    self.rtsemamba_thread = Thread(target=self.rtsemamba_callback)
    self.past_start = time.time()
    
    print("starting to capture...")
    self.subscription = self.create_subscription(JackAudio, '/jackaudio', self.jackaudio_callback,1000)
    self.subscription  # prevent unused variable warning
  
  def rtsemamba_callback(self):
    #if not self.rtsemamba_allocate:
    #  capt_time = time.time() - self.past_start
    #  print(f"capture time : {capt_time}")
    #self.past_start = time.time()
    #start_time = time.time()
    
    #grabbing audio data from jackaudio
    input_win = torch.tensor([self.rtsemamba_ring_in.popleft() for _i in range(self.segment_size)],device=self.device).unsqueeze(0)
    
    self.sf_file_in.write(input_win.squeeze().cpu().detach().numpy())
    
    stft_out = self.stft.process(input_win)
    if stft_out is None:
      self.rtsemamba_ring_out.extend(np.zeros(0, dtype=np.float32))
    
    mag, pha, _ = stft_out          # (1, F, T_new)
    T_new = mag.shape[2]
    
    # ---- Build context + new input ---------------------------------- #
    if self._ctx_mag is not None:
      full_mag = torch.cat([self._ctx_mag, mag], dim=2)
      full_pha = torch.cat([self._ctx_pha, pha], dim=2)
    else:
      full_mag, full_pha = mag, pha
    T_full = full_mag.shape[2]
    
    # ---- Single model call on the full window ----------------------- #
    with torch.no_grad():
        enh_mag_full, enh_pha_full, _ = self.rtsemamba(full_mag, full_pha)
    
    # ---- Extract only the new frames' output ----------------------- #
    enh_mag = enh_mag_full[:, :, T_full - T_new:]
    enh_pha = enh_pha_full[:, :, T_full - T_new:]
    
    # ---- Update rolling context (keep last n_ctx frames) ----------- #
    self._ctx_mag = full_mag[:, :, -self.n_ctx:].detach()
    self._ctx_pha = full_pha[:, :, -self.n_ctx:].detach()
    
    # ---- OLA reconstruction ---------------------------------------- #
    self.istft.add_frames(enh_mag, enh_pha)
    
    # After N total frames, samples [0 .. N×hop_size − 1] are complete
    readable_end = min(
      self.istft.processed_frames * self.hop_size,
      self.istft.buffer.numel(),
    )
    if readable_end <= self._out_read_pos:
      self.rtsemamba_ring_out.extend(np.zeros(0, dtype=np.float32))
    
    buf = self.istft.buffer
    ws  = self.istft.win_sum
    new_samples = (
      buf[self._out_read_pos : readable_end]
      #/ torch.clamp(ws[self._out_read_pos : readable_end], min=1e-8)
    ).cpu().numpy().astype(np.float32)
    
    #if not self.rtsemamba_allocate:
    #  exec_time = time.time() - start_time
    #  print(f"execution time : {exec_time}")
    
    self.sf_file_out.write(new_samples)
    
    self.rtsemamba_ring_out.extend(new_samples.tolist())
    
    self._out_read_pos = readable_end
  
  def jackaudio_callback(self, msg):
    self.rtsemamba_ring_in.extend(msg.data)
    
    if len(self.rtsemamba_ring_in) >= self.segment_size:
      #print(f"{len(self.rtsemamba_ring_in)=}")
      if self.rtsemamba_thread.is_alive():
        #print("waiting for rtsemamba to finish last segment")
        self.rtsemamba_thread.join()
      #print("doing rtsemamba")
      self.rtsemamba_thread = Thread(target=self.rtsemamba_callback)
      self.rtsemamba_thread.start()
    
    #print(f"{len(self.rtsemamba_ring_out)=}")
    if len(self.rtsemamba_ring_out) >= self.jack_win_size:
      filt_win = [self.rtsemamba_ring_out.popleft() for _i in range(self.jack_win_size)]
    else:
      #print("publishing zero'ed jack window")
      filt_win = [0.0]*(self.jack_win_size)
    
    msg_filt = JackAudio()
    msg_filt.size = len(filt_win)
    msg_filt.header.stamp = self.get_clock().now().to_msg()
    msg_filt.data = filt_win
    self.publisher.publish(msg_filt)


def main(args=None):
  rclpy.init(args=args)
  rtsemambarosaudio = RTSEMAMBAROSAudio()
  rclpy.spin(rtsemambarosaudio)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  rtsemambarosaudio.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
