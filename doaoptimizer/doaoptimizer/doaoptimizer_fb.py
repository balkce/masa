import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from soundloc.msg import DOA

import time
import sys
import numpy as np
from scipy.signal import detrend

from threading import Thread

class DOAOptimizer(Node):
  def __init__(self):
    super().__init__('doaoptimizer')
    
    self.declare_parameter('eta', 0.30)
    self.eta = self.get_parameter('eta').get_parameter_value().double_value
    #print(f"DOAOpt: eta is {self.eta}")
    self.declare_parameter('wait_for_sdr', 1.5)
    self.wait_for_sdr = self.get_parameter('wait_for_sdr').get_parameter_value().double_value
    self.declare_parameter('opt_correction', True)
    self.opt_correction = self.get_parameter('opt_correction').get_parameter_value().bool_value
    
    self.opt_correction = False
    
    self.subscription_SDR = self.create_subscription(Float32,'/SDR',self.sdr_callback,1000)
    self.subscription_SDR  # prevent unused variable warning
    self.subscription_theta_est = self.create_subscription(DOA,'/theta_est',self.theta_est_callback,1000)
    self.subscription_theta_est  # prevent unused variable warning
    self.publisher = self.create_publisher(Float32, '/theta', 1000)
    
    self.latest_curr_doa_hist_len = 5
    self.latest_curr_doa_hist = np.zeros(0)
    self.latest_theta_est_hist_len = 5
    self.latest_theta_est_hist = np.zeros(0)
    self.latest_theta_est = None
    self.latest_theta_est_time = None
    self.min_confidence = 4
    
    self.curr_doa = np.zeros(2)
    self.curr_doa[0] = self.latest_theta_est
    self.curr_doa[1] = 0.0 #this is really important so that Adam doesn't get stuck at the beginning
    self.request_sdr = False
    self.curr_sdr = np.zeros(2)
    
    self.bored_windows_max = 5
    self.bored_windows = 0
    self.bored_theta_est_var_max = 6.0
    self.bored_curr_doa_var_max = 0.7
    
    self.std_weights_curr_doa = 1/(1 + np.exp(-(np.arange(self.latest_curr_doa_hist_len)-(self.latest_curr_doa_hist_len/2))))
    self.std_weights_theta_est = 1/(1 + np.exp(-(np.arange(self.latest_theta_est_hist_len)-(self.latest_theta_est_hist_len/2))))
    self.doa_to_publish = self.latest_theta_est
    
    self.max_eta = 0.5
    self.min_eta = 0.01
    self.max_doavar = 5
    
    self.past_doa_num = 6
    self.past_doa = np.zeros(self.past_doa_num)
    self.past_doa_calc = 0
    self.past_win_wo_corr = 0
    self.past_win_wo_corr_max = 8
    
    self.best_doa = self.latest_theta_est
    self.best_sdr = None
    
    self.m_dw, self.v_dw = 0, 0
    self.beta1 = 0.9
    self.beta2 = 0.999
    self.epsilon = 1e-8
    
    self.past_doas_reset = np.zeros(self.past_doa_num)
    
    self.opt_thread = Thread(target=self.do_doaopt)
    self.opt_thread.start()
  
  def objective(self, x):
    return (100-x)
  
  def gradient(self,x,y):
    #coef = np.polyfit(x,y,1)
    #return coef[0]
    return (y[0] - y[1])/(x[0] - x[1] + self.epsilon)
  
  def add_curr_doa(self):
    if len(self.latest_curr_doa_hist) < self.latest_curr_doa_hist_len:
      self.latest_curr_doa_hist = np.append(self.latest_curr_doa_hist,[self.curr_doa[0]])
    else:
      self.latest_curr_doa_hist[:-1] = self.latest_curr_doa_hist[1:]
      self.latest_curr_doa_hist[-1] = self.curr_doa[0]
  
  def add_theta_est(self):
    if len(self.latest_theta_est_hist) < self.latest_theta_est_hist_len:
      self.latest_theta_est_hist = np.append(self.latest_theta_est_hist,[self.latest_theta_est])
    else:
      self.latest_theta_est_hist[:-1] = self.latest_theta_est_hist[1:]
      self.latest_theta_est_hist[-1] = self.latest_theta_est
  
  def theta_est_callback(self, msg):
    #print(msg.doas)
    #print(msg.confs)
    
    if self.latest_theta_est == None:
      #use the one with the highest confidence
      highest_conf = 0.0
      highest_conf_i = 0
      for i in range(len(msg.confs)):
        if msg.confs[i] > highest_conf:
          highest_conf = msg.confs[i]
          highest_conf_i = i
      
      highest_conf_doa = msg.doas[highest_conf_i]
      
      if abs(highest_conf_doa) < 60:
        #prefer a DOA that is in front of the array
        self.latest_theta_est = highest_conf_doa
        self.latest_theta_est_time = time.time()
        self.add_theta_est()
        
        self.curr_doa[0] = self.latest_theta_est
        self.add_curr_doa()
        
        self.doa_to_publish = self.latest_theta_est
    
    else:
      #use the one nearest to self.curr_doa[0]
      nearest_doa = 0.0
      nearest_doa_i = 0
      nearest_doa_dist = sys.float_info.max
      for i in range(len(msg.doas)):
        this_dist = abs(msg.doas[i] - self.curr_doa[0])
        if this_dist < nearest_doa_dist:
          nearest_doa = msg.doas[i]
          nearest_doa_i = i
          nearest_doa_dist = this_dist
      
      #use it only if it is actually close and has good confidence
      if nearest_doa_dist < 15 and msg.confs[nearest_doa_i] >= self.min_confidence:
        self.latest_theta_est = nearest_doa
        self.add_theta_est()
        #print("--- "+str(self.latest_theta_est))
  
  def sdr_callback(self, msg):
    if self.request_sdr:
      self.curr_sdr[1:] = self.curr_sdr[:-1]
      self.curr_sdr[0] = self.objective(msg.data)
      self.request_sdr = False
  
  def get_merged_doa(self):
    if (len(self.latest_theta_est_hist) < self.latest_theta_est_hist_len or len(self.latest_curr_doa_hist) < self.latest_curr_doa_hist_len):
      print("not enough history, publishing curr_doa")
      return self.latest_curr_doa_hist[-1]
    else:
      
      # get curr_doa stability (weighted std from detrended history)
      #latest_curr_doa_hist_detrend = detrend(self.latest_curr_doa_hist, type='linear')
      #curr_doa_var = np.sqrt(np.cov(latest_curr_doa_hist_detrend, aweights=self.std_weights_curr_doa))
      #print("curr_doa_var    : "+str(curr_doa_var))
      curr_doa_var_simple = np.std(self.latest_curr_doa_hist,ddof=1)
      print("curr_doa_var_sim: "+str(curr_doa_var_simple)+" <> "+str(self.bored_curr_doa_var_max))
      
      # get theta_est stability (weighted std from detrended history)
      latest_theta_est_hist_detrend = detrend(self.latest_theta_est_hist, type='linear')
      theta_est_var = np.sqrt(np.cov(latest_theta_est_hist_detrend, aweights=self.std_weights_theta_est))
      print("theta_est_var   : "+str(theta_est_var)+" <> "+str(self.bored_theta_est_var_max))
      
      if self.bored_windows >= self.bored_windows_max:
        if theta_est_var > self.bored_theta_est_var_max:
          self.bored_windows = 0
          print("EXCITED, starting to use theta_est")
          if self.opt_correction:
            self.opt_correction = False
        else:
          print("bored, keep using just curr_doa")
          if not self.opt_correction:
            self.reset_opt_correction(self.latest_curr_doa_hist[-1])
            self.opt_correction = True
        return self.latest_curr_doa_hist[-1]
      else:
        theta_est_weight = np.arctan(curr_doa_var_simple/theta_est_var)/(np.pi/2)
        
        print("curr_doa     :"+str(self.latest_curr_doa_hist[-1]))
        print("theta_est    :"+str(np.mean(self.latest_theta_est_hist)))
        doa_publish = (theta_est_weight*np.mean(self.latest_theta_est_hist)) + ((1-theta_est_weight)*self.latest_curr_doa_hist[-1])
        
        self.curr_doa[0] = doa_publish
        self.latest_curr_doa_hist[-1] = self.curr_doa[0]
        
        if curr_doa_var_simple > self.bored_curr_doa_var_max:
          self.bored_windows = 0
        else:
          self.bored_windows += 1
          print("   bored_win : "+str(self.bored_windows)+"/"+str(self.bored_windows_max))
        
        return doa_publish
  
  def reset_opt_correction(self, doa):
    self.curr_doa[0] = doa
    self.curr_doa[1] = 0.0
    self.add_curr_doa()
    
    self.past_doa = np.zeros(self.past_doa_num)
    self.past_doa_calc = 0
    self.best_sdr = None
    
    self.curr_sdr[0] = 0.0
    self.curr_sdr[1] = 0.0
    #self.curr_sdr[1] = self.curr_sdr[0]
    
    self.past_win_wo_corr = 0
  
  def do_doaopt(self):
    t = 0
    
    self.get_logger().info("Waiting for first DOA estimation from soundloc...")
    while self.latest_theta_est == None:
      time.sleep(0.001)
    
    self.get_logger().info("Starting DOA correction...")
    while True:
      #this_str = "sdr: "+str(self.curr_sdr[0])+" ->"
      #for i in range(len(self.past_doa)):
      #  this_str += " "+str(self.past_doa[i])
      #self.get_logger().info(this_str)
      
      self.past_doa[1:] = self.past_doa[:-1]
      self.past_doa[0] = self.doa_to_publish
      if self.opt_correction:
        self.past_doa_calc += 1
        
        if self.past_doa_calc >= self.past_doa_num:
          if self.best_sdr == None:
            self.best_sdr = self.curr_sdr[0]
            self.best_doa = self.past_doa[-1]
            print("sdr %f -> %f (first)" % (self.curr_sdr[0], self.past_doa[-1]))
          elif self.curr_sdr[0] < self.best_sdr:
            self.best_sdr = self.curr_sdr[0]
            self.best_doa = self.past_doa[-1]
            self.past_win_wo_corr = 0
            print("sdr %f -> %f (updated best)" % (self.curr_sdr[0], self.past_doa[-1]))
          else:
            self.past_win_wo_corr += 1
            if self.past_win_wo_corr >= self.past_win_wo_corr_max:
              print("sdr %f -> %f (corrected)" % (self.best_sdr, self.best_doa))
              self.reset_opt_correction(self.best_doa)
            else:
              print("sdr %f -> %f" % (self.curr_sdr[0], self.past_doa[-1]))
      else:
        print("sdr %f -> %f" % (self.curr_sdr[0], self.past_doa[-1]))
      
      self.doa_to_publish = self.get_merged_doa()
      print("doa_to_publis: "+str(self.doa_to_publish))
      print("")
      
      msg = Float32()
      msg.data = self.doa_to_publish
      self.publisher.publish(msg)
      
      #print(f"DOAOpt: giving time for the system to react to new theta...")
      time.sleep(self.wait_for_sdr)
      
      #print(f"DOAOpt: reading new SDR value...")
      self.request_sdr = True
      while self.request_sdr:
        time.sleep(0.001)
      
      #print(f"DOAOpt: doing optimization...")
      dw = self.gradient(self.curr_doa,self.curr_sdr)
      #print(f"DOAOpt: current gradient is {dw}")
      
      ## momentum beta 1
      self.m_dw = self.beta1*self.m_dw + (1-self.beta1)*dw
      
      ## rms beta 2
      self.v_dw = self.beta2*self.v_dw + (1-self.beta2)*(dw**2)
      
      ## update value
      self.curr_doa[1:] = self.curr_doa[:-1]
      self.curr_doa[0] = self.curr_doa[0] - self.eta*(self.m_dw/(np.sqrt(self.v_dw)+self.epsilon))
      self.add_curr_doa()

def main(args=None):
  rclpy.init(args=args)
  doaoptimizer = DOAOptimizer()
  rclpy.spin(doaoptimizer)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  doaoptimizer.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
