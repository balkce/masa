import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from jack_msgs.msg import JackAudio

import math
import time
import torch
from torchaudio.pipelines import SQUIM_OBJECTIVE
import numpy as np

from threading import Thread

class OptScale(torch.nn.Module):
  def __init__(self, device):
    super(OptScale, self).__init__()
    init_scaling_factor = torch.zeros(1, device=device)
    self.scaling_factor = torch.nn.Parameter(init_scaling_factor, requires_grad=True)

  def forward(self, x):
    output = self.scaling_factor * x
    return output

class OnlineSQA(Node):
  def __init__(self):
    super().__init__('onlinesqa')
    
    self.device = "cuda:0"
    
    self.subscription = self.create_subscription(JackAudio,'/jackaudio_filtered',self.jackaudio_callback,1000)
    #self.subscription = self.create_subscription(JackAudio,'/jackaudio',self.jackaudio_callback,1000)
    self.subscription  # prevent unused variable warning
    
    self.sdr_publisher = self.create_publisher(Float32, '/SDR', 1000)
    self.stoi_publisher = self.create_publisher(Float32, '/STOI', 1000)
    self.pesq_publisher = self.create_publisher(Float32, '/PESQ', 1000)
    
    self.declare_parameter('hop_secs', 1.5)
    self.hop_secs = self.get_parameter('hop_secs').get_parameter_value().double_value
    self.declare_parameter('win_len_secs', 3.0)
    self.win_len_secs = self.get_parameter('win_len_secs').get_parameter_value().double_value
    self.declare_parameter('smooth_weight', 0.9)
    self.smooth_weight = self.get_parameter('smooth_weight').get_parameter_value().double_value
    self.declare_parameter('vad_threshold', 0.85)
    self.vad_threshold = self.get_parameter('vad_threshold').get_parameter_value().double_value
    
    self.objective_model = SQUIM_OBJECTIVE.get_model().to(self.device)
    
    self.samplerate = 16000
    self.vad_hop_len = 512 #Silvero-VAD only permits chunks of 512 at samplerate = 16000
    self.vad_hop_secs = self.vad_hop_len/self.samplerate
    
    self.hop_len = int(round(self.hop_secs*self.samplerate))
    self.hop_len = math.ceil(self.hop_len/self.vad_hop_len)*self.vad_hop_len
    self.hop_secs = self.hop_len/self.samplerate
    self.hop_i = 0
    
    self.win_len = int(round(self.win_len_secs*self.samplerate))
    self.win_len = math.ceil(self.win_len/self.vad_hop_len)*self.vad_hop_len
    self.win_len_secs = self.win_len/self.samplerate
    self.win = torch.zeros(1,self.win_len)
    
    self.num_hops = int(self.win_len/self.hop_len) 
    self.vad_num_hops = math.ceil(self.win_len/self.vad_hop_len) 
    self.vad_num_hops_in_num_hops = int(self.hop_len/self.vad_hop_len) 
    self.num_hops_towait = int(self.vad_num_hops/3)
    self.num_hops_vad = int(self.num_hops/2)
    self.win_qual_start = 0
    self.win_qual_end = 0
    
    self.get_logger().info('sample rate    : %d' % self.samplerate)
    self.get_logger().info('VAD threshold  : %f' % self.vad_threshold)
    self.get_logger().info('window length  : %f (%d samples)' % (self.win_len_secs, self.win_len))
    self.get_logger().info('hop length     : %f (%d samples, %d hops in window)' % (self.hop_secs, self.hop_len, self.num_hops))
    self.get_logger().info('VAD hop length : %f (%d samples, %d hops in window, %d in hop length)' % (self.vad_hop_secs, self.vad_hop_len, self.vad_num_hops, self.vad_num_hops_in_num_hops))
    
    self.sqa_ready = False
    self.sqa_ready_ref = False
    self.sqa_thread = Thread(target=self.do_sqa)
    self.sqa_thread.start()
    
    self.smooth_val_sdr = None
    self.smooth_val_stoi = None
    self.smooth_val_pesq = None
  
    self.vad, utils = torch.hub.load(repo_or_dir='snakers4/silero-vad', model='silero_vad', trust_repo=True)
    self.vad_confs = torch.zeros(self.vad_num_hops)
    
  def jackaudio_callback(self, msg):
    self.win = torch.roll(self.win,-len(msg.data))
    self.win[0,-len(msg.data):] = torch.Tensor(msg.data)
    self.hop_i += len(msg.data)
    
    for i in range(0, len(msg.data), self.vad_hop_len):
      #start_time = time.time()
      i_start = self.win_len - (len(msg.data)-i)
      i_end = self.win_len - (len(msg.data) - (i+self.vad_hop_len))
      new_confidence = self.vad(self.win[0,i_start:i_end], self.samplerate).item()
      #exec_time = time.time() - start_time
      #self.get_logger().info('VAD confidence: %0.2f, in %f secs' % (new_confidence, exec_time))
      
      self.vad_confs = torch.roll(self.vad_confs,1)
      self.vad_confs[0] = 1 if new_confidence > self.vad_threshold else 0
    
    if self.hop_i >= self.hop_len:
      #print(self.vad_confs)
      if torch.sum(self.vad_confs) >= (self.vad_num_hops*3/4):
        #the user has talked through the whole win_len window
        self.win_qual_start = 0
        self.win_qual_end = self.win_len
        while self.sqa_ready:
          time.sleep(0.01)
        self.sqa_ready = True
      
      self.hop_i = 0
  
  def smooth(self,data_point,data_type):
    
    smooth_val = None
    if data_type == 'sdr':
      smooth_val = self.smooth_val_sdr
    elif data_type == 'stoi':
      smooth_val = self.smooth_val_stoi
    elif data_type == 'pesq':
      smooth_val = self.smooth_val_pesq
    else:
      print("smooth: Invalid data_type value ("+str(data_type)+"). Can only be 'sdr', 'stoi', or 'pesq'.")
      return None
    
    if smooth_val == None:
      this_smooth_val = data_point
    else:
      this_smooth_val = smooth_val * self.smooth_weight + (1 - self.smooth_weight) * data_point
    
    if data_type == 'sdr':
      self.smooth_val_sdr = this_smooth_val
    elif data_type == 'stoi':
      self.smooth_val_stoi = this_smooth_val
    elif data_type == 'pesq':
      self.smooth_val_pesq = this_smooth_val
    
    return this_smooth_val

  def do_sqa(self):
    while True:
      while not self.sqa_ready:
        time.sleep(0.001)
      
      win_clone = self.win.to(self.device)
      stoi_hyp, pesq_hyp, si_sdr_hyp = self.objective_model(win_clone[0,self.win_qual_start:self.win_qual_end].unsqueeze(0))
      
      # SDR publishing
      unfiltered_observation = si_sdr_hyp.item()
      if math.isnan(unfiltered_observation):
        unfiltered_observation = 0.0
      filtered_observation_smooth = self.smooth(unfiltered_observation,'sdr')
      self.get_logger().info('SDR:  %0.4f, smooth: %0.4f' % (unfiltered_observation,filtered_observation_smooth))
      msg = Float32()
      msg.data = filtered_observation_smooth
      self.sdr_publisher.publish(msg)
      
      # STOI publishing
      unfiltered_observation = stoi_hyp.item()
      if math.isnan(unfiltered_observation):
        unfiltered_observation = 0.0
      filtered_observation_smooth = self.smooth(unfiltered_observation,'stoi')
      self.get_logger().info('STOI: %0.4f, smooth: %0.4f' % (unfiltered_observation,filtered_observation_smooth))
      msg = Float32()
      msg.data = filtered_observation_smooth
      self.stoi_publisher.publish(msg)
      
      # PESQ publishing
      unfiltered_observation = pesq_hyp.item()
      if math.isnan(unfiltered_observation):
        unfiltered_observation = 0.0
      filtered_observation_smooth = self.smooth(unfiltered_observation,'pesq')
      self.get_logger().info('PESQ: %0.4f, smooth: %0.4f' % (unfiltered_observation,filtered_observation_smooth))
      msg = Float32()
      msg.data = filtered_observation_smooth
      self.pesq_publisher.publish(msg)
      
      self.sqa_ready = False


def main(args=None):
  rclpy.init(args=args)
  onlinesqa = OnlineSQA()
  rclpy.spin(onlinesqa)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  onlinesqa.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
