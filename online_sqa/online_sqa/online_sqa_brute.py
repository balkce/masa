import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import yaml
import xml.etree.ElementTree as ET

from std_msgs.msg import Float32
from jack_msgs.msg import JackAudio
from jack_msgs.srv import BruteTheta

import os
import sys
import math
import time
import torch
import torchaudio
import numpy as np

import jack
import soundfile as sf

import torch.multiprocessing as mp
import ctypes

import inspect

from ament_index_python.packages import get_package_share_directory

# assigned in main
jackqueue = None
jacktorchstarted = None
jackthread_stop = None
jacktorch_stop = None

# assigned in ros2
win_len_secs = None
ros2_defined = None
ports_num = None

# assigned in jack
win_len = None
fft_len = None
win_static_start = None
jack_samplerate = None
jack_defined = None
jack_started = None

samplerate = 16000
device = "cuda:0"

def jack_to_torch(jackqueue,jacktorchstarted,ports_num,win_len,winqueue,win_request,jacktorch_stop):
  global device
  
  jacktorchstarted.value = True
  
  win_shared = torch.zeros(ports_num.value,win_len.value).cpu().detach()
  win_shared.share_memory_()
  
  while jacktorch_stop.value == False:
    queued_audio = jackqueue.get()
    audio = queued_audio.clone()
    del queued_audio
    frames = audio.shape[0]
    win_shared = torch.roll(win_shared,-frames,dims=1)
    win_shared[:,-frames:] = audio.T
    
    if win_request.value:
      with win_request.get_lock():
        winqueue.put(win_shared)
        win_request.value = False

def jack_main(jack_defined,jack_started,ros2_defined,jack_samplerate,win_len_secs_shared,ports_num,win_len_shared,fft_len_shared,win_static_start,jackqueue,jacktorchstarted,jackthread_stop):
  global samplerate
  global device
  
  jackclient = jack.Client('online_sqa', use_exact_name=True)
  ratio = samplerate/jackclient.samplerate
  
  event = mp.Event()
  
  fft_len = int(jackclient.blocksize*2)
  fft_len_shared.value = fft_len
  jack_samplerate.value = jackclient.samplerate
  
  while ros2_defined.value == False:
    time.sleep(0.001)
  win_len = int(round(win_len_secs_shared.value*jackclient.samplerate))
  win_len = int(math.ceil(win_len/(fft_len))*(fft_len))
  win_len_secs = win_len/jackclient.samplerate
  win_len_shared.value = win_len
  win_len_secs_shared.value = win_len_secs
  
  win_static_start.value = False
  
  ### signal that jack info has been defined
  jack_defined.value = True
  
  print('JACKClient: jack winlen        : %d' % jackclient.blocksize)
  print('JACKClient: sample rate jack   : %d' % jackclient.samplerate)
  print('JACKClient: sample rate demucs : %d' % samplerate)
  print('JACKClient: window length      : %f (%d samples)' % (win_len_secs, win_len))
  
  global audio
  global buffer_captured
  global lastframetime
  
  buffer_captured = 0
  lastframetime = 0
  audio = torch.zeros((jackclient.blocksize,ports_num.value)).cpu().detach()
  audio.share_memory_()
  
  @jackclient.set_process_callback
  def process(frames):
    global audio
    global buffer_captured
    global lastframetime
    
    # assert len(client.inports) == len(client.outports)
    assert frames == jackclient.blocksize
    
    #print(jackclient.frame_time - lastframetime)
    #lastframetime = jackclient.frame_time
    
    if jacktorchstarted.value:
      #start_time = time.time()
      for i in range(ports_num.value):
        audio[:,i] = torch.Tensor(inputs[i].get_array())
      #exec_time = time.time() - start_time
      #print('copy input to audio : %f secs' % (exec_time))
      
      #start_time = time.time()
      jackqueue.put(audio)
      #exec_time = time.time() - start_time
      #print('jackqueue put       : %f secs' % (exec_time))
      #print('')
      
      if win_static_start.value == False:
        buffer_captured += frames
        
        #if self.buffer_captured >= win_len + (jackclient.samplerate*8): # waiting 8 seconds after full window is captured
        if buffer_captured >= win_len_shared.value:
          win_static_start.value = True
    
    if jackthread_stop.value:
      jackclient.deactivate()
      time.sleep(0.005)
      jackclient.close()
  
  @jackclient.set_xrun_callback
  def xrun_callback(delay):
    print('JACKClient: XRUN occurred : -> '+str(delay))
  
  @jackclient.set_shutdown_callback
  def shutdown(status, reason):
    print('JACKClient: JACK shutdown!')
    print('JACKClient: status -> '+str(status))
    print('JACKClient: reason -> '+str(reason))
    event.set()
  
  inputs = []
  for i in range(ports_num.value):
    inputs.append(jackclient.inports.register('input_'+str(i+1)))
  
  with jackclient:
    print('JACKClient: JACK started!')
    jack_started.value = True
    try:
      event.wait()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
      print('JACKClient: stopped by user.')



class OnlineSQA(Node):
  def __init__(self):
    super().__init__('onlinesqa')
    
    global win_len_secs
    global ports_num
    
    global samplerate
    global device
    
    global ros2_defined
    
    self.device = device
    
    self.wrotesomething = 0
    
    self.group = ReentrantCallbackGroup()
    
    self.service_qual = self.create_service(BruteTheta, 'brutetheta', self.brutetheta_callback, callback_group=self.group)
    
    self.declare_parameter('win_len_secs', 3.0)
    self.win_len_secs = self.get_parameter('win_len_secs').get_parameter_value().double_value
    win_len_secs = mp.Value('d',self.win_len_secs)
    
    self.deg2rad = math.pi/180.0
    self.rad2deg = 180.0/math.pi
    self.vsound = 343.0
    
    #get input ports from beamformphase config file
    beamformphase_share_directory = get_package_share_directory('beamformphase')
    beamformphase_config_filepath = os.path.join(beamformphase_share_directory,"config","beamform_config.yaml")
    beamformphase_launch_filepath = os.path.join(beamformphase_share_directory,"launch","phase.launch")
    
    self.get_logger().info('Getting microphone configuration from: '+beamformphase_config_filepath)
    with open(beamformphase_config_filepath, 'r') as file:
      data = yaml.safe_load(file)  # Deserializes YAML into a Python dict
      self.ports_num = data['/beamformphase']['ros__parameters']['number_of_microphones']
      ports_num = mp.Value('i',self.ports_num)
      self.get_logger().info('number of microphones : '+str(self.ports_num))
      self.mics = []
      ref_x = 0.0
      ref_y = 0.0
      ref_z = 0.0
      for i in range(self.ports_num):
        mic_name = "mic"+str(i)
        mic_conf = {}
        if i == 0:
          ref_x = data['/beamformphase']['ros__parameters'][mic_name][0]
          ref_y = data['/beamformphase']['ros__parameters'][mic_name][1]
          if len(data['/beamformphase']['ros__parameters'][mic_name]) > 2:
            ref_z = data['/beamformphase']['ros__parameters'][mic_name][2]
        mic_conf['x'] = torch.Tensor([data['/beamformphase']['ros__parameters'][mic_name][0] - ref_x]).to(self.device)
        mic_conf['y'] = torch.Tensor([data['/beamformphase']['ros__parameters'][mic_name][1] - ref_y]).to(self.device)
        if len(data['/beamformphase']['ros__parameters'][mic_name]) > 2:
          mic_conf['z'] = torch.Tensor([data['/beamformphase']['ros__parameters'][mic_name][2] - ref_z]).to(self.device)
        mic_conf['dist'] = torch.Tensor([math.sqrt(mic_conf['x']**2 + mic_conf['y']**2)]).to(self.device)
        mic_conf['angle'] = torch.Tensor([math.atan2(mic_conf['y'],mic_conf['x']) * self.rad2deg]).to(self.device)
        self.mics.append(mic_conf)
      self.get_logger().info('microphone locations : '+str(self.mics))
    
    ros2_defined.value = True
    
    self.declare_parameter('init_doa', 15.0)
    self.init_doa = self.get_parameter('init_doa').get_parameter_value().double_value
    self.declare_parameter('audio_to_eval', 'pre')
    self.audio_to_eval = self.get_parameter('audio_to_eval').get_parameter_value().string_value
    if self.audio_to_eval != 'pre' and self.audio_to_eval != 'post':
      self.get_logger().info("invalid audio_to_eval value ("+str(self.audio_to_eval)+"). Can only be 'pre' or 'post'. Defaulting to 'pre'.")
      self.audio_to_eval = 'pre'
    self.declare_parameter('quality_type', 'stoi')
    self.quality_type = self.get_parameter('quality_type').get_parameter_value().string_value
    if self.quality_type != 'sdr' and self.quality_type != 'stoi' and self.quality_type != 'pesq' and self.quality_type != 'scoreq' and self.quality_type != 'audbox':
      self.get_logger().info("invalid quality_type value ("+str(self.quality_type)+"). Can only be 'sdr', 'stoi', 'pesq', 'scoreq' or 'audbox'. Defaulting to 'stoi'.")
      self.quality_type = 'stoi'
    self.declare_parameter('beamformtype', 'mvdr')
    self.beamformtype = self.get_parameter('beamformtype').get_parameter_value().string_value
    if self.beamformtype != 'phase' and self.beamformtype != 'mvdr':
      self.get_logger().info("invalid beamformtype value ("+str(self.beamformtype)+"). Can only be 'phase' or 'mvdr'. Defaulting to 'mvdr'.")
      self.beamformtype = 'mvdr'
    
    if self.quality_type == 'sdr':
      from torchaudio.pipelines import SQUIM_OBJECTIVE
      self.objective_model = SQUIM_OBJECTIVE.get_model().to(self.device)
      self.sdr_publisher = self.create_publisher(Float32, '/SDR', 10)
    elif self.quality_type == 'stoi':
      from torchaudio.pipelines import SQUIM_OBJECTIVE
      self.objective_model = SQUIM_OBJECTIVE.get_model().to(self.device)
      self.stoi_publisher = self.create_publisher(Float32, '/STOI', 10)
    elif self.quality_type == 'pesq':
      from torchaudio.pipelines import SQUIM_OBJECTIVE
      self.objective_model = SQUIM_OBJECTIVE.get_model().to(self.device)
      self.pesq_publisher = self.create_publisher(Float32, '/PESQ', 10)
    elif self.quality_type == 'scoreq':
      from .submodules import scoreq
      self.scoreq_model = scoreq.Scoreq(data_domain='natural', mode='nr', use_onnx=False)
      self.scoreq_publisher = self.create_publisher(Float32, '/SCOREQ', 10)
    elif self.quality_type == 'audbox':
      from audiobox_aesthetics.infer import initialize_predictor
      self.audbox_model = initialize_predictor()
      self.audbox_publisher = self.create_publisher(Float32, '/AUDBOX', 10)
    
    self.samplerate = samplerate
    
    if self.audio_to_eval == 'post':
      self.get_logger().info('audio to eval. : demucs')
      this_share_directory = get_package_share_directory('demucs')
      this_base_directory = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(this_share_directory))))
      this_src_directory = os.path.join(this_base_directory,"src","demucs")
      self.demucs_modelpath = os.path.join(this_src_directory,"pretrained_model","best.th")
      self.get_logger().info("Using the following pretrained model: "+self.demucs_modelpath)
      
      sys.path.append(this_src_directory)
      import denoiser
      self.demucs = self.load_demucsmodel(self.demucs_modelpath, device=self.device)
    else:
      self.get_logger().info('audio to eval. : beamform ('+self.beamformtype+')')
    
    self.delays = torch.zeros(self.ports_num).to(self.device)
    
    self.get_logger().info('beamform type  : '+self.beamformtype)
    if self.beamformtype == 'phase':
      pair_num = int((self.ports_num*(self.ports_num-1))/2)
      self.indices = torch.triu_indices(pair_num, pair_num, offset=1)
      
      self.batch_avg_diff = torch.vmap(self.avg_diff_mult)
      self.batch_avg_diff_once = torch.vmap(self.avg_diff_once)
      
      #get beamform configuration from beamformphase launch file
      self.min_phase = 20.0
      self.mag_mult = 0.1
      self.mag_threshold = 0.0001
      
      self.get_logger().info('Getting beamformphase configuration from: '+beamformphase_launch_filepath)
      tree = ET.parse(beamformphase_launch_filepath)
      root = tree.getroot()
      for child in root.findall(".//param"):
        if "name" in child.attrib:
          if child.attrib["name"] == "min_phase":
            self.min_phase = float(child.attrib["value"])
          elif child.attrib["name"] == "mag_mult":
            self.mag_mult = float(child.attrib["value"])
          elif child.attrib["name"] == "mag_threshold":
            self.mag_threshold = float(child.attrib["value"])
      self.min_phase_diff_mean = self.min_phase * self.deg2rad
      self.get_logger().info('beamconf [min_phase]     : '+str(self.min_phase))
      self.get_logger().info('beamconf [min_phase_rad] : '+str(self.min_phase_diff_mean))
      self.get_logger().info('beamconf [mag_mult]      : '+str(self.mag_mult))
      self.get_logger().info('beamconf [mag_threshold] : '+str(self.mag_threshold))
    else:
      self.mag_mult = 0.1
      self.mag_threshold = 0.0001
      
      self.batch_get_invcovariance = torch.vmap(self.get_invcovariance)
      self.batch_get_optweights = torch.vmap(self.get_optweights)
      self.batch_apply_optweights = torch.vmap(self.apply_optweights)
      self.batch_apply_optweights_per_f = torch.vmap(self.apply_optweights_per_f)
      
      self.whitemult = torch.ones((self.ports_num,self.ports_num)).to(self.device)
      for i in range(self.ports_num):
        self.whitemult[i,i] = 1.001
      self.whiteadd = torch.zeros((self.ports_num,self.ports_num)).to(self.device)
      for i in range(self.ports_num):
        self.whiteadd[i,i] = 0.0000000001
      
      self.J = torch.fliplr(torch.eye(self.ports_num)).to(torch.complex64).to(self.device)
      
      self.get_logger().info('beamconf                 : -----')
  
  def initialize(self, win_len_shared, fft_len, jack_samplerate):
    global jackqueue
    global jacktorchstarted
    global jacktorch_stop
    global ports_num
    global win_len
    
    self.win_len = win_len_shared
    self.fft_len = fft_len
    self.hann_wola = torch.sqrt(torch.hann_window(self.fft_len).to(self.device))
    self.freqs = torch.fft.fftfreq(self.fft_len,d=1/jack_samplerate).to(self.device)
    
    self.win_static = torch.zeros(self.ports_num,self.win_len).to(self.device)
    self.win_static_fft = torch.stft(self.win_static,self.fft_len,hop_length=int(self.fft_len/2),window=self.hann_wola,onesided=False,return_complex=True)
    self.num_win = self.win_static_fft.shape[2]
    
    if self.beamformtype == 'phase':
      self.weights = self.win_static_fft.clone()
    else:
      self.weights = torch.zeros((self.fft_len,self.ports_num),dtype=torch.complex64).to(self.device)
    
    self.transform_resample = torchaudio.transforms.Resample(jack_samplerate, self.samplerate).to(self.device)
    
    self.update_weights(self.init_doa,ini=True)
    
    self.winqueue = mp.Queue()
    self.win_request = mp.Value('b',False)
    
    jacktorch_thread = mp.Process(target=jack_to_torch,args=(jackqueue,jacktorchstarted,ports_num,win_len,self.winqueue,self.win_request,jacktorch_stop))
    jacktorch_thread.start()
  
  def calculate_delays(self,theta):
    #self.get_logger().info('theta : %f' % (theta))
    for i in range(self.ports_num):
      if i == 0:
        self.delays[i] = 0.0
      else:
        this_dist = self.mics[i]["dist"];
        this_angle = self.mics[i]["angle"]-theta;
        if this_angle > 180.0:
          this_angle -= 360.0;
        elif this_angle < -180.0:
          this_angle += 360.0;
        
        self.delays[i] = this_dist*torch.cos(this_angle*self.deg2rad)/(-self.vsound);
    #self.get_logger().info('delays : '+str(self.delays))
  
  def update_weights(self,theta,ini=False):
    self.calculate_delays(theta)
    
    if self.beamformtype == 'phase':
      for i in range(self.ports_num):
        if i == 0:
          if ini:
            self.weights[i,:,:] = (torch.ones(self.fft_len,dtype=torch.complex64).to(self.device).unsqueeze(0).T).repeat(1,self.num_win)
        else:
          self.weights[i,:,:] = (torch.exp(-2.0j*torch.pi*self.freqs*self.delays[i]).unsqueeze(0).T).repeat(1,self.num_win)
    else:
      for i in range(self.ports_num):
        if i == 0:
          if ini:
            self.weights[:,i] = torch.ones(self.fft_len,dtype=torch.complex64).to(self.device)
        else:
          self.weights[:,i] = torch.exp(-2.0j*torch.pi*self.freqs*self.delays[i]).to(self.device)
        self.weightsconj = self.weights.conj_physical()
  
  def avg_diff_once(self,phases):
    dis = torch.abs(phases.view(-1, 1) - phases.view(1, -1))
    return dis[self.indices[0],self.indices[1]]
  
  def avg_diff_mult(self,phases):
    dis = self.batch_avg_diff_once(phases)
    return dis
  
  def get_invcovariance(self,data,dataconj):
    R = (data @ dataconj.T)/data.shape[1]
    return torch.nan_to_num(torch.linalg.pinv(R))
  
  def get_optweights(self,invR,weights,weightsconj):
    numerator = invR @ weights
    denominator = weightsconj.T @ invR @ weights
    return torch.nan_to_num(numerator / denominator)
  
  def apply_optweights_per_f(self,optweightsconj,data):
    return (optweightsconj.T @ data)
  
  def apply_optweights(self,optweightsconj,data):
    return self.batch_apply_optweights_per_f(optweightsconj.unsqueeze(2),data.unsqueeze(2)).squeeze(1).squeeze(1)
  
  def get_jackaudio(self):
    global win_static_start
    
    if win_static_start.value == True:
      with self.win_request.get_lock():
        self.win_request.value = True
      while self.win_request.value:
        time.sleep(0.001)
      with self.win_request.get_lock():
        this_win = self.winqueue.get()
        win_clone = this_win.clone().to(self.device)
        del this_win
      return torch.stft(win_clone,self.fft_len,hop_length=int(self.fft_len/2),window=self.hann_wola,onesided=False,return_complex=True)
    else:
      return None
  
  def apply_weights(self,win_fft):
    
    mag_mean = torch.mean(torch.abs(win_fft),0)
    pha_mean = win_fft[0,:,:].angle()
    
    if self.beamformtype == 'phase':
      phases_aligned = (self.weights.conj_physical() * win_fft).angle()
      
      phase_diffs = self.batch_avg_diff(phases_aligned.permute(2,1,0))
      ind_gt1 = phase_diffs > torch.pi
      phase_diffs[ind_gt1] = 2*torch.pi - phase_diffs[ind_gt1]
      mean_phase_diffs = torch.mean(phase_diffs,dim=2).T
      
      out = win_fft[0,:,:].clone()
      ind_gt2 = (mag_mean/self.fft_len) <= self.mag_threshold
      out[ind_gt2] = torch.complex(self.mag_mult*mag_mean[ind_gt2]*torch.cos(pha_mean[ind_gt2]),self.mag_mult*mag_mean[ind_gt2]*torch.sin(pha_mean[ind_gt2]))
      ind_gt3 = mean_phase_diffs >= self.min_phase_diff_mean
      out[ind_gt3] = torch.complex(self.mag_mult*mag_mean[ind_gt3]*torch.cos(pha_mean[ind_gt3]),self.mag_mult*mag_mean[ind_gt3]*torch.sin(pha_mean[ind_gt3]))
      
      return self.transform_resample(torch.istft(out,self.fft_len,hop_length=int(self.fft_len/2),window=self.hann_wola,onesided=False,return_complex=True).real)
    else:
      invRs = self.batch_get_invcovariance(win_fft.permute(1,0,2),win_fft.permute(1,0,2).conj_physical())
      otpweights = self.batch_get_optweights(invRs,self.weights.unsqueeze(2),self.weightsconj.unsqueeze(2))
      
      out = self.batch_apply_optweights(otpweights.repeat(1,1,self.num_win).permute(2,0,1).conj_physical(),win_fft.permute(2,1,0)).T
      
      ind_gt2 = (mag_mean/self.fft_len) <= self.mag_threshold
      out[ind_gt2] = torch.complex(self.mag_mult*mag_mean[ind_gt2]*torch.cos(pha_mean[ind_gt2]),self.mag_mult*mag_mean[ind_gt2]*torch.sin(pha_mean[ind_gt2]))
      
      return self.transform_resample(torch.istft(out,self.fft_len,hop_length=int(self.fft_len/2),window=self.hann_wola,onesided=False,return_complex=True).real)
  
  def get_audio_from_theta(self, theta, win_fft):
    # Updating microphone delays and weights
    self.update_weights(theta)
    
    # Apply weights to microphones
    if self.audio_to_eval == 'pre':
      beamformphase_result_raw = self.apply_weights(win_fft)
      
      beamformphase_result = beamformphase_result_raw.clone().unsqueeze(0)
      
      #sf.write('test_'+str(self.wrotesomething)+'_th_'+str(theta)+'_b.wav', beamformphase_result.T.cpu().detach().numpy(), samplerate=self.samplerate)
    else:
      beamformphase_result_raw = self.apply_weights(win_fft)
      
      beamformphase_result_pre = beamformphase_result_raw.clone()
      input_win = beamformphase_result_pre.unsqueeze(0).unsqueeze(0)
      interf_signal = input_win.clone() #this is ignored by the current version of demucs, but is required
      noisy_win = self.combine_interf (input_win,interf_signal) #this is ignored by the current version of demucs, but is required
      beamformphase_result = self.demucs(noisy_win)[0][0].unsqueeze(0).clone()
      
      #sf.write('test_'+str(self.wrotesomething)+'_th_'+str(theta)+'_b.wav', beamformphase_result_pre.unsqueeze(0).T.cpu().detach().numpy(), samplerate=self.samplerate)
      #sf.write('test_'+str(self.wrotesomething)+'_th_'+str(theta)+'_p.wav', beamformphase_result.T.cpu().detach().numpy(), samplerate=self.samplerate)
    
    return beamformphase_result
  
  def get_qual_from_audio(self, audio):
    ### feed to SQA model
    if self.quality_type == 'sdr':
      #start_time = time.time()
      stoi_hyp, pesq_hyp, si_sdr_hyp = self.objective_model(audio)
      #exec_time = time.time() - start_time
      #self.get_logger().info('SQUIM rt   : %f secs' % (exec_time))
      
      # SDR publishing
      return torch.nan_to_num(si_sdr_hyp).tolist()
      
    elif self.quality_type == 'stoi':
      #start_time = time.time()
      stoi_hyp, pesq_hyp, si_sdr_hyp = self.objective_model(audio)
      #exec_time = time.time() - start_time
      #self.get_logger().info('SQUIM rt   : %f secs' % (exec_time))
      
      # STOI publishing
      return torch.nan_to_num(stoi_hyp).tolist()
      
    elif self.quality_type == 'pesq':
      #start_time = time.time()
      stoi_hyp, pesq_hyp, si_sdr_hyp = self.objective_model(audio)
      #exec_time = time.time() - start_time
      #self.get_logger().info('SQUIM rt   : %f secs' % (exec_time))
      
      # PESQ publishing
      return torch.nan_to_num(pesq_hyp).tolist()
    
    else:
      return None
    
  
  def get_qual_from_theta(self, theta, win_fft):
    # Updating microphone delays and weights
    self.update_weights(theta)
    
    # Apply weights to microphones
    if self.audio_to_eval == 'pre':
      beamformphase_result_raw = self.apply_weights(win_fft)
      
      beamformphase_result = beamformphase_result_raw.clone().unsqueeze(0)
      
      #sf.write('test_b.wav', beamformphase_result.T.cpu().detach().numpy(), samplerate=self.samplerate)
      #sf.write('test_'+str(self.wrotesomething)+'_th_'+str(thetareq.theta)+'_b.wav', beamformphase_result.T.cpu().detach().numpy(), samplerate=self.samplerate)
    else:
      beamformphase_result_raw = self.apply_weights(win_fft)
      
      beamformphase_result_pre = beamformphase_result_raw.clone()
      input_win = beamformphase_result_pre.unsqueeze(0).unsqueeze(0)
      interf_signal = input_win.clone() #this is ignored by the current version of demucs, but is required
      noisy_win = self.combine_interf (input_win,interf_signal) #this is ignored by the current version of demucs, but is required
      beamformphase_result = self.demucs(noisy_win)[0][0].unsqueeze(0).clone()
      
      #sf.write('test_'+str(self.wrotesomething)+'_th_'+str(thetareq.theta)+'_b.wav', beamformphase_result_pre.unsqueeze(0).T.cpu().detach().numpy(), samplerate=self.samplerate)
      #sf.write('test_'+str(self.wrotesomething)+'_th_'+str(thetareq.theta)+'_p.wav', beamformphase_result.T.cpu().detach().numpy(), samplerate=self.samplerate)
    
    ### feed to SQA model
    if self.quality_type == 'scoreq':
      #start_time = time.time()
      scoreq_hyp = self.scoreq_model.predict_data(beamformphase_result, ref_wave_raw=None)
      #exec_time = time.time() - start_time
      #self.get_logger().info('SCOREQ rt  : %f secs' % (exec_time))
      
      # SCOREQ publishing
      observation = scoreq_hyp
      if math.isnan(observation):
        observation = 0.0
      return observation
      
    elif self.quality_type == 'audbox':
      #start_time = time.time()
      audbox_hyp = self.audbox_model.forward([{"path":beamformphase_result, "sample_rate":self.samplerate}])
      #exec_time = time.time() - start_time
      #self.get_logger().info('AUDIOBOX rt: %f secs' % (exec_time))
      
      # AUDBOX publishing
      observation = audbox_hyp.item()
      if math.isnan(observation):
        observation = 0.0
      return observation
    else:
      return -1000.0
  
  def brutetheta_callback(self, thetareq, optres):
    global win_static_start
    global jack_started
    
    while jack_started.value == False:
      time.sleep(0.001)
    
    self.get_logger().info("received theta :  "+str(thetareq.theta))
    
    optres.thetas = np.linspace(thetareq.theta-thetareq.range,thetareq.theta+thetareq.range,num=thetareq.steps).tolist()
    optres.quals = [0.0] * len(optres.thetas)
    
    if win_static_start.value == False:
      self.get_logger().info('OptTheta_generic:  -1000.0')
      optres.thetas[0] = -1000.0
      optres.quals[0] = -1000.0
      return optres
    
    win_fft = self.get_jackaudio()
    if win_fft == None:
      self.get_logger().info('OptTheta_generic:  -1000.0')
      optres.thetas[0] = -1000.0
      optres.quals[0] = -1000.0
      return optres
    
    #self.get_logger().info("got jack audio :  "+str(win_fft.shape))
    if self.quality_type == 'sdr' or self.quality_type == 'stoi' or self.quality_type == 'pesq':
      audio = None
      for i in range(len(optres.thetas)):
        thisaudio = self.get_audio_from_theta(optres.thetas[i],win_fft)
        if thisaudio == None:
          self.get_logger().info('OptTheta_generic:  -1000.0')
          optres.thetas[0] = -1000.0
          optres.quals[0] = -1000.0
          return optres
        
        if audio == None:
          audio = torch.zeros(len(optres.thetas),thisaudio.shape[1]).to(self.device)
        
        audio[i,:] = thisaudio
      
      #self.get_logger().info("got beam audio :  "+str(audio.shape))
      quals_tmp = self.get_qual_from_audio(audio)
      if quals_tmp == None:
        self.get_logger().info('OptTheta_generic:  -1000.0')
        optres.thetas[0] = -1000.0
        optres.quals[0] = -1000.0
        return optres
      optres.quals = quals_tmp
      #self.get_logger().info("got quality    :  "+str(optres.quals))
      
    else:
      for i in range(len(optres.thetas)):
        qual = self.get_qual_from_theta(optres.thetas[i],win_fft)
        if qual == -1000.0:
          self.get_logger().info('OptTheta_generic:  -1000.0')
          optres.thetas[0] = -1000.0
          optres.quals[0] = -1000.0
          return optres
        
        optres.quals[i] = float(qual)
    
    self.wrotesomething+=1
    
    theta_req_i = int(len(optres.thetas)/2)
    self.get_logger().info(str(optres.thetas[theta_req_i])+' ('+str(optres.quals[theta_req_i])+')')
    return optres
  
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
  global jackqueue
  global jacktorchstarted
  
  global win_len_secs
  global ros2_defined
  global ports_num
  
  global win_len
  global fft_len
  global win_static_start
  global jack_samplerate
  global jack_defined
  global jack_started
  
  global jackthread_stop
  global jacktorch_stop
  
  mp.set_start_method('spawn', force=True)
  jackqueue = mp.Queue()

  jacktorchstarted = mp.Value('b',False)
  jackthread_stop = mp.Value('b',False)
  jacktorch_stop = mp.Value('b',False)
  ros2_defined = mp.Value('b',False)
  jack_defined = mp.Value('b',False)
  jack_started = mp.Value('b',False)
  
  rclpy.init(args=args)
  onlinesqa = OnlineSQA()
  
  win_len = mp.Value('i',0)
  fft_len = mp.Value('i',0)
  win_static_start = mp.Value('b',False)
  jack_samplerate = mp.Value('i',0)
  
  try:
    jack_thread = mp.Process(target=jack_main, args=(jack_defined,jack_started,ros2_defined,jack_samplerate,win_len_secs,ports_num,win_len,fft_len,win_static_start,jackqueue,jacktorchstarted,jackthread_stop))
    jack_thread.start()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    jackthread_stop.value = True
    jack_thread.join()
  
  while jack_defined.value == False:
    time.sleep(0.001)
  
  onlinesqa.initialize(win_len.value,fft_len.value,jack_samplerate.value)
  
  executor = MultiThreadedExecutor()
  executor.add_node(onlinesqa)
  try:
    executor.spin()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    # graceful shutdown of threads
    jackthread_stop.value = True
    jacktorch_stop.value = True
    jack_thread.join()
  finally:
    # destroying ros2 node
    onlinesqa.destroy_node()
    executor.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
  main()
