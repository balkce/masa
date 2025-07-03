import rclpy
from rclpy.node import Node

import os
import time
import jack
import joblib
import librosa
import numpy as np
from time import sleep

import threading

from soundloc.msg import Frequencies

from ament_index_python.packages import get_package_share_directory

class FrequencySelector(Node):
  def __init__(self):
    super().__init__('freqselect')
    
    self.declare_parameter('time_between_predictions', 0.5)
    self.time_between_predictions = self.get_parameter('time_between_predictions').get_parameter_value().double_value
    self.declare_parameter('vad_threshold', 0.005)
    self.vad_threshold = self.get_parameter('vad_threshold').get_parameter_value().double_value
    
    this_share_directory = get_package_share_directory('freqselect')
    this_base_directory = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(this_share_directory))))
    this_src_directory = os.path.join(this_base_directory,"src","freqselect")
    self.modelpath = os.path.join(this_src_directory,"pretrained_model","svc_filtro_flatten.pkl")
    self.get_logger().info("Using the following pretrained model: "+self.modelpath)
    self.model = joblib.load(self.modelpath)
    
    self.classes = {}
    self.classes["gun_shot"] = [2.5]
    self.classes["car_horn"] = [55.0, 381.5, 474.25]
    self.classes["dog_bark"] = [539.5, 625.0, 634.8, 685.0, 736.8, 778.9]
    self.classes["siren"]    = [110.0, 1095.0]
    self.classes["other"]    = []
    
    self.last_prediction = None
    
    self.publisher = self.create_publisher(Frequencies, '/doafrequencies', 1000)
    
    self.buffer_size = 0
    self.buffer_pasado = np.array([])
    self.hann_win = np.array([])
    
    self.jack_thread = threading.Thread(target=self.jack_main)
    self.jack_thread.start()
    
    self.model_thread = threading.Thread(target=self.get_prediction)
    self.model_thread.start()
  
  def calculate_mfcc(self, y, sr, buffer_size, n_mfcc=12):
    mfcc = librosa.feature.mfcc(y=y, sr=sr, n_fft=buffer_size, n_mfcc=n_mfcc, window='hann', hop_length=int(sr*.041)).flatten()
    
    return mfcc
  
  def get_prediction(self):
    input_i = 0
    while (True):
      sleep(self.time_between_predictions)
      #input_i += 1
      
      rms = np.sqrt(np.sum(np.power(self.buffer_pasado,2))/self.buffer_pasado.shape[0])
      #self.get_logger().info(f"FreqSelect: {rms = }")
      
      if rms >= self.vad_threshold:
        mfcc = self.calculate_mfcc(self.buffer_pasado*self.filt_win, self.jackclient.samplerate, self.buffer_size)
        #self.get_logger().info(f"{mfcc.shape = }, {type(mfcc) = }")
        
        prediction = self.model.predict(mfcc.reshape(1,-1))[0]
        
        if prediction in self.classes:
          #t = input_i * self.time_between_predictions
          #self.get_logger().info(f"{t = } -> {prediction = }")
          
          if self.last_prediction != prediction:
            msg = Frequencies()
            msg.w = self.classes[prediction]
            msg.size = len(msg.w)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            
            #self.get_logger().info(f"FreqSelect: {prediction = } !published")
            self.get_logger().info(f"FreqSelect: {prediction = }")
            self.last_prediction = prediction
          #else:
          #  self.get_logger().info(f"FreqSelect: {prediction = }")
        else:
          self.get_logger().info(f"FreqSelect: unknown class {prediction = }")
  
  def jack_main(self):
    self.jackclient = jack.Client('freqselect', use_exact_name=True)
    
    self.event = threading.Event()
    
    self.buffer_size = self.jackclient.blocksize
    self.buffer_pasado = np.zeros((93*self.buffer_size))
    
    #self.filt_win = np.hanning(self.buffer_pasado.shape[0])
    self.filt_win = np.ones(self.buffer_pasado.shape[0])
    filt_end_len = int(self.jackclient.samplerate/4)
    self.filt_win[:filt_end_len] = np.linspace(0.0,1.0,num=filt_end_len)
    self.filt_win[-filt_end_len:] = np.linspace(1.0,0.0,num=filt_end_len)
    
    self.get_logger().info(f"FreqSelect: using VAD threshold of {self.vad_threshold}")
    self.get_logger().info(f"FreqSelect: using buffer of {self.buffer_pasado.shape[0]/self.jackclient.samplerate} s.")
    
    @self.jackclient.set_process_callback
    def process(frames):
      # self.get_logger().info('Callback')
      # assert len(client.inports) == len(client.outports)
      assert frames == self.buffer_size
      
      audio = self.input.get_array()
      
      self.buffer_pasado[:-self.buffer_size] = self.buffer_pasado[self.buffer_size:]
      self.buffer_pasado[-self.buffer_size:] = audio
      #self.get_logger().info(f"FreqSelect: got audio window of {audio.shape[0]} samples")
    
    @self.jackclient.set_shutdown_callback
    def shutdown(status, reason):
      self.get_logger().info('FreqSelect: JACK shutdown!')
      self.get_logger().info('FreqSelect: status -> ', status)
      self.get_logger().info('FreqSelect: reason -> ', reason)
      self.event.set()
    
    self.input = self.jackclient.inports.register('input_1')
    
    with self.jackclient:
      try:
        self.event.wait()
      except KeyboardInterrupt:
        self.get_logger().info('\nFreqSelect: stopped by user.')

def main(args=None):
  rclpy.init(args=args)
  freqselector = FrequencySelector()
  rclpy.spin(freqselector)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  freqselector.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
