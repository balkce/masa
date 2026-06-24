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

import json

from threading import Thread
from collections import deque

from ament_index_python.packages import get_package_share_directory

class AttrDict(dict):
    def __init__(self, *args, **kwargs):
        super(AttrDict, self).__init__(*args, **kwargs)
        self.__dict__ = self

class MuseMVDRROSAudio(Node):
  def __init__(self):
    super().__init__('muse')
    
    self.device = "cuda"
    self.subscription = self.create_subscription(JackAudio, '/jackaudio', self.jackaudio_callback,1000)
    self.subscription  # prevent unused variable warning
    self.publisher = self.create_publisher(JackAudio, '/jackaudio_filtered', 1000)
    
    self.declare_parameter('input_length', 0.512)
    self.input_length = self.get_parameter('input_length').get_parameter_value().double_value
    
    this_share_directory = get_package_share_directory('muse')
    this_base_directory = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(this_share_directory))))
    this_src_directory = os.path.join(this_base_directory,"src","muse")
    self.muse_modelpath = os.path.join(this_src_directory,"pretrained_model","g_best")
    self.muse_configpath = os.path.join(this_src_directory,"pretrained_model","config.json")
    print("Using the following pretrained model : "+self.muse_modelpath)
    print("Using the following pretrained config: "+self.muse_configpath)
    
    with open(self.muse_configpath) as f:
        data = f.read()
    
    json_config = json.loads(data)
    h = AttrDict(json_config)
    
    self.muse_segment_size = h.segment_size
    self.n_fft = h.n_fft
    self.hop_size = h.hop_size
    self.win_size = h.win_size
    self.compress_factor = h.compress_factor
    self.samplerate = h.sampling_rate #16000
    
    torch.manual_seed(h.seed)
    torch.cuda.manual_seed(h.seed)
    if not torch.cuda.is_available():
      print(f"CUDA NOT AVAILABLE!")
      exit()
    
    sys.path.append(this_src_directory)
    from models.generator import MUSE
    
    self.muse = MUSE(h).to(self.device)
    assert os.path.isfile(self.muse_modelpath)
    print("Loading '{}'".format(self.muse_modelpath))
    state_dict = torch.load(self.muse_modelpath, map_location=self.device)
    print("Complete.")
    self.muse.load_state_dict(state_dict['generator'])
    self.muse.eval()
    
    self.hann_window = torch.hann_window(self.win_size).to(self.device)
    
    self.jack_win_size = 1024 #BIG assumption
    
    
    self.muse_win_num = int((self.input_length*self.samplerate)/self.jack_win_size)
    self.segment_size = int(self.muse_win_num*self.jack_win_size)
    self.padded_zeros = torch.zeros(1, self.muse_segment_size - self.segment_size).to(self.device)
    
    print(f"input_length     : {self.input_length} seconds")
    print(f"segment_size     : {self.segment_size} samples")
    print(f"jack_win_size: {self.jack_win_size} samples")
    print(f"sample rate  : {self.samplerate} samples/second")
    
    print(f"muse_win_num: {self.muse_win_num} windows")
    
    self.muse_ring_in = deque([])
    self.muse_ring_out = deque([])
    self.muse_in = [0.0]*(self.muse_win_num*self.jack_win_size)
    self.muse_out = [0.0]*(self.muse_win_num*self.jack_win_size)
    self.muse_in_win_i = 0
    self.muse_out_win_i = 0
    self.READY_TO_CLONE_OUT = True
    
    self.sf_file_in = sf.SoundFile("out_muse_in.wav",mode="w",samplerate=h.sampling_rate,channels=1,subtype='PCM_16')
    self.sf_file_out = sf.SoundFile("out_muse_out.wav",mode="w",samplerate=h.sampling_rate,channels=1,subtype='PCM_16')
    
    print("doing initial model test to allocate memory")
    self.muse_allocate = True
    self.muse_thread = Thread(target=self.muse_callback)
    self.muse_ring_in.extend([0.0]*(self.segment_size))
    self.muse_thread.start()
    self.muse_thread.join()
    print("...done")
    
    self.muse_allocate = False
    self.muse_thread = Thread(target=self.muse_callback)
    self.past_start = time.time()
    
    self.past_rms = []
    self.past_rms_len = 10
  
  
  def muse_callback(self):
    if not self.muse_allocate:
      capt_time = time.time() - self.past_start
      print(f"capture time : {capt_time}")
    self.past_start = time.time()
    
    start_time = time.time()
    
    #input_win = torch.tensor(self.muse_in,device=self.device).unsqueeze(0)
    
    input_win = torch.tensor([self.muse_ring_in.popleft() for _i in range(self.segment_size)],device=self.device).unsqueeze(0)
    
    if not self.muse_allocate:
      #this_rms = torch.sqrt(len(input_win) / torch.sum(input_win ** 2.0)).item()
      print(f"{len(self.past_rms)=}")
      if len(self.past_rms) >= self.past_rms_len:
        del self.past_rms[0]
      
      this_rms = torch.sqrt(torch.mean(input_win ** 2.0)).item()
      self.past_rms.append(this_rms)
      
      norm_factor = torch.sqrt(torch.mean(torch.tensor(self.past_rms).to(self.device) ** 2.0))
      input_win = input_win / norm_factor
    else:
      norm_factor = 1.0
    
    self.sf_file_in.write(input_win.squeeze().cpu().detach().numpy())
    
    #padding
    print(f"{input_win.shape=}")
    input_win = torch.cat((input_win, self.padded_zeros), dim=1)
    print(f"{input_win.shape=}")
    
    noisy_amp, noisy_pha, noisy_com = self.mag_pha_stft(input_win, self.n_fft, self.hop_size, self.win_size, self.compress_factor)
    amp_g, pha_g, com_g = self.muse(noisy_amp.to(self.device, non_blocking=True), noisy_pha.to(self.device, non_blocking=True))
    audio_g = self.mag_pha_istft(amp_g, pha_g, self.n_fft, self.hop_size, self.win_size, self.compress_factor)
    audio_g = audio_g * norm_factor
    audio_g = audio_g.squeeze()
    print(f"{audio_g.shape=}")
    audio_g = audio_g[ :-(self.muse_segment_size-self.segment_size)]
    print(f"{audio_g.shape=}")
    
    self.sf_file_out.write(audio_g.cpu().detach().numpy())
    
    if not self.muse_allocate:
      exec_time = time.time() - start_time
      print(f"execution time : {exec_time}")
      
      #self.get_logger().info('capture time: %f, response time: %f' % (capt_time, exec_time))
      
      #while not self.READY_TO_CLONE_OUT:
      #  time.sleep(0.001)
      #self.muse_out = audio_g.tolist()
      self.muse_ring_out.extend(audio_g.tolist())
      #self.READY_TO_CLONE_OUT = False
  
  def jackaudio_callback(self, msg):
    #self.muse_in[self.muse_in_win_i*self.jack_win_size:(self.muse_in_win_i+1)*self.jack_win_size] = msg.data
    self.muse_ring_in.extend(msg.data)
    
    if len(self.muse_ring_in) >= self.segment_size:
      print(f"{len(self.muse_ring_in)=}")
      if self.muse_thread.is_alive():
        print("waiting for muse to finish last segment")
        self.muse_thread.join()
      print("doing muse")
      self.muse_thread = Thread(target=self.muse_callback)
      self.muse_thread.start()
    
    #filt_win = self.muse_out[self.muse_out_win_i*self.jack_win_size:(self.muse_out_win_i+1)*self.jack_win_size]
    print(f"{len(self.muse_ring_out)=}")
    if len(self.muse_ring_out) >= self.jack_win_size:
      filt_win = [self.muse_ring_out.popleft() for _i in range(self.jack_win_size)]
    else:
      print("publishing zero'ed jack window")
      filt_win = [0.0]*(self.jack_win_size)
    
    msg_filt = JackAudio()
    msg_filt.size = len(filt_win)
    msg_filt.header.stamp = self.get_clock().now().to_msg()
    msg_filt.data = filt_win
    self.publisher.publish(msg_filt)
    
    #self.muse_out_win_i += self.jack_win_size
    #print(f"{self.muse_out_win_i=}")
    #if self.muse_out_win_i >= self.segment_size:
    #  self.muse_out_win_i = 0
    #  self.READY_TO_CLONE_OUT = True
  
  def mag_pha_stft(self, y, n_fft, hop_size, win_size, compress_factor=1.0, center=True):
    stft_spec = torch.stft(y, n_fft, hop_length=hop_size, win_length=win_size, window=self.hann_window, center=center, pad_mode='reflect', normalized=False, return_complex=True)
    mag = torch.abs(stft_spec)
    pha = torch.angle(stft_spec)
    
    # Magnitude Compression
    mag = torch.pow(mag, compress_factor)
    com = torch.stack((mag*torch.cos(pha), mag*torch.sin(pha)), dim=-1)
    
    return mag, pha, com
  
  def mag_pha_istft(self, mag, pha, n_fft, hop_size, win_size, compress_factor=1.0, center=True):
    # Magnitude Decompression
    mag = torch.pow(mag, (1.0/compress_factor))
    com = torch.complex(mag*torch.cos(pha), mag*torch.sin(pha))
    wav = torch.istft(com, n_fft, hop_length=hop_size, win_length=win_size, window=self.hann_window, center=center)
    
    return wav

def main(args=None):
  rclpy.init(args=args)
  musemvdrrosaudio = MuseMVDRROSAudio()
  rclpy.spin(musemvdrrosaudio)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  musemvdrrosaudio.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
