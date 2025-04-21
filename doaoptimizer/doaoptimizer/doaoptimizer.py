import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import time
import numpy as np
from scipy.signal import detrend

from threading import Thread

class DOAOptimizer(Node):
  def __init__(self):
    super().__init__('doaoptimizer')
    
    self.declare_parameter('init_doa', 0.0)
    self.init_doa = self.get_parameter('init_doa').get_parameter_value().double_value
    #print(f"DOAOpt: init_doa is {self.init_doa}")
    self.declare_parameter('eta', 0.30)
    self.eta = self.get_parameter('eta').get_parameter_value().double_value
    #print(f"DOAOpt: eta is {self.eta}")
    self.declare_parameter('wait_for_sdr', 1.5)
    self.wait_for_sdr = self.get_parameter('wait_for_sdr').get_parameter_value().double_value
    self.declare_parameter('opt_correction', True)
    self.opt_correction = self.get_parameter('opt_correction').get_parameter_value().bool_value
    
    self.subscription = self.create_subscription(Float32,'/SDR',self.sdr_callback,1000)
    self.subscription  # prevent unused variable warning
    self.publisher = self.create_publisher(Float32, '/theta', 1000)
    
    self.curr_doa = np.zeros(2)
    self.curr_doa[0] = self.init_doa
    self.curr_doa[1] = 0.0 #this is really important so that Adam doesn't get stuck at the beginning
    self.request_sdr = False
    self.curr_sdr = np.zeros(2)
    
    self.max_eta = 0.5
    self.min_eta = 0.01
    self.max_doavar = 5
    
    self.past_doa_num = 6
    self.past_doa = np.zeros(self.past_doa_num)
    self.past_doa_calc = 0
    self.past_win_wo_corr = 0
    self.past_win_wo_corr_max = 10
    
    self.best_doa = self.init_doa
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
  
  def sdr_callback(self, msg):
    if self.request_sdr:
      self.curr_sdr[1:] = self.curr_sdr[:-1]
      self.curr_sdr[0] = self.objective(msg.data)
      self.request_sdr = False
  
  def do_doaopt(self):
    t = 0
    while True:
      #this_str = "sdr: "+str(self.curr_sdr[0])+" ->"
      #for i in range(len(self.past_doa)):
      #  this_str += " "+str(self.past_doa[i])
      #self.get_logger().info(this_str)
      
      if self.opt_correction:
        self.past_doa[1:] = self.past_doa[:-1]
        self.past_doa[0] = self.curr_doa[0]
        self.past_doa_calc += 1
        
        if self.past_doa_calc >= self.past_doa_num:
          if self.best_sdr == None:
            self.best_sdr = self.curr_sdr[0]
            self.best_doa = self.past_doa[-1]
            print("%f -> %f (first)" % (self.curr_sdr[0], self.past_doa[-1]))
          elif self.curr_sdr[0] < self.best_sdr:
            self.best_sdr = self.curr_sdr[0]
            self.best_doa = self.past_doa[-1]
            self.past_win_wo_corr = 0
            print("%f -> %f (updated best)" % (self.curr_sdr[0], self.past_doa[-1]))
          else:
            self.past_win_wo_corr += 1
            if self.past_win_wo_corr >= self.past_win_wo_corr_max:
              self.curr_doa[0] = self.best_doa
              self.curr_doa[1] = 0.0
              
              self.past_doa = np.zeros(self.past_doa_num)
              self.past_doa_calc = 0
              self.best_sdr = None
              
              self.curr_sdr[0] = 0.0
              self.curr_sdr[1] = 0.0
              #self.curr_sdr[1] = self.curr_sdr[0]
              
              self.past_win_wo_corr = 0
              
              print("sdr (corrected)")
            else:
              print("sdr %f -> %f" % (self.curr_sdr[0], self.past_doa[-1]))
      
      
      #this_doa_var = np.std(detrend(self.past_doa, type='linear'),ddof=1)
      
      self.get_logger().info("publishing current doa -> %f" % self.curr_doa[0])
      msg = Float32()
      msg.data = self.curr_doa[0]
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
