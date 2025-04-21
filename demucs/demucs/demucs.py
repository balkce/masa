import rclpy
from rclpy.node import Node

from jack_msgs.msg import JackAudio

import os
import math
import time
import torch
import numpy as np

import inspect
import denoiser

from threading import Thread

from ament_index_python.packages import get_package_share_directory

class DemucsPhaseROSAudio(Node):
  def __init__(self):
    super().__init__('demucs')
    
    self.device = "cuda"
    self.subscription = self.create_subscription(JackAudio, '/jackaudio', self.jackaudio_callback,1000)
    self.subscription  # prevent unused variable warning
    self.publisher = self.create_publisher(JackAudio, '/jackaudio_filtered', 1000)
    
    self.declare_parameter('input_length', 0.512)
    self.input_length = self.get_parameter('input_length').get_parameter_value().double_value
    
    this_share_directory = get_package_share_directory('demucs')
    this_base_directory = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(this_share_directory))))
    this_src_directory = os.path.join(this_base_directory,"src","demucs")
    self.demucs_modelpath = os.path.join(this_src_directory,"pretrained_model","best.th")
    print("Using the following pretrained model: "+self.demucs_modelpath)
    
    self.demucs = self.load_demucsmodel(self.demucs_modelpath, device=self.device)
    self.past_start = time.time()
    self.samplerate = 16000
    self.jack_win_size = 1024 #BIG assumption
    
    print(f"input_length     : {self.input_length} seconds")
    print(f"jack_win_size: {self.jack_win_size} samples")
    print(f"sample rate  : {self.samplerate} samples/second")
    
    self.demucs_win_num = int((self.input_length*self.samplerate)/self.jack_win_size)
    print(f"demucs_win_num: {self.demucs_win_num} windows")
    
    self.demucs_in = [0.0]*(self.demucs_win_num*self.jack_win_size)
    self.demucs_out = [0.0]*(self.demucs_win_num*self.jack_win_size)
    self.demucs_in_win_i = 0
    self.demucs_out_win_i = 0
    self.READY_TO_CLONE_OUT = False
    self.demucs_thread = Thread(target=self.demucs_callback)

  
  def demucs_callback(self):
    #capt_time = time.time() - self.past_start
    #print(f"capture time : {capt_time}")
    #self.past_start = time.time()
    
    #start_time = time.time()
    input_win = torch.tensor(self.demucs_in,device=self.device).unsqueeze(0).unsqueeze(0)
    interf_signal = input_win.clone() #this is ignored by the current version of demucs, but is required
    noisy_win = self.combine_interf (input_win,interf_signal) #this is ignored by the current version of demucs, but is required
    output_win = self.demucs(noisy_win)[0][0].tolist()
    #exec_time = time.time() - start_time
    #print(f"execution time : {exec_time}")
    
    #self.get_logger().info('capture time: %f, response time: %f' % (capt_time, exec_time))
    
    while not self.READY_TO_CLONE_OUT:
      time.sleep(0.001)
    self.demucs_out = output_win
    self.READY_TO_CLONE_OUT = False
  
  def jackaudio_callback(self, msg):
    self.demucs_in[self.demucs_in_win_i*self.jack_win_size:(self.demucs_in_win_i+1)*self.jack_win_size] = msg.data
    
    self.demucs_in_win_i += 1
    if self.demucs_in_win_i >= self.demucs_win_num:
      if self.demucs_thread.is_alive():
        self.demucs_thread.join()
      self.demucs_thread = Thread(target=self.demucs_callback)
      self.demucs_thread.start()
      self.demucs_in_win_i = 0
    
    filt_win = self.demucs_out[self.demucs_out_win_i*self.jack_win_size:(self.demucs_out_win_i+1)*self.jack_win_size]
    msg_filt = JackAudio()
    msg_filt.size = len(filt_win)
    msg_filt.header.stamp = self.get_clock().now().to_msg()
    msg_filt.data = filt_win
    self.publisher.publish(msg_filt)
    
    self.demucs_out_win_i += 1
    if self.demucs_out_win_i >= self.demucs_win_num:
      self.demucs_out_win_i = 0
      self.READY_TO_CLONE_OUT = True

  def combine_interf (self, signal,interf):
    return torch.cat((signal,interf),2)

  def deserialize_model(self, package, strict=False):
    return model
  
  def load_demucsmodel(self, model_path, device="cuda"):
    package = torch.load(model_path, map_location=device)
    
    klass = package['class']
    kwargs = package['kwargs']
    
    sig = inspect.signature(klass)
    kw = package['kwargs']
    for key in list(kw):
      if key not in sig.parameters:
        del kw[key]
    model = klass(*package['args'], **kw)
    model.load_state_dict(package['state'])
    
    model.to(device)
    return model


def main(args=None):
  rclpy.init(args=args)
  demucsphaserosaudio = DemucsPhaseROSAudio()
  rclpy.spin(demucsphaserosaudio)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  demucsphaserosaudio.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
