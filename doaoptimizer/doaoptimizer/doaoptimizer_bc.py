import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import time
import numpy as np
from sklearn.linear_model import SGDRegressor

from threading import Thread

class DOAOptimizer(Node):
  def __init__(self):
    super().__init__('doaoptimizer')
    
    self.declare_parameter('init_doa', 0.0)
    self.init_doa = self.get_parameter('init_doa').get_parameter_value().double_value
    #print(f"DOAOpt: init_doa is {self.init_doa}")
    self.declare_parameter('eta', 0.1)
    self.eta = self.get_parameter('eta').get_parameter_value().double_value
    #print(f"DOAOpt: eta is {self.eta}")
    
    self.subscription = self.create_subscription(Float32,'/SDR',self.sdr_callback,1000)
    self.subscription  # prevent unused variable warning
    self.publisher = self.create_publisher(Float32, '/theta', 1000)
    
    self.past_samples = 2
    
    self.curr_doa = np.zeros(self.past_samples)
    self.curr_doa[0] = self.init_doa
    self.curr_doa[1] = 0.0 #this is really important so that Adam doesn't get stuck at the beginning
    self.request_sdr = False
    self.curr_sdr = np.zeros(self.past_samples)
    self.past_sdr = 0.0
    
    self.m_dw, self.v_dw = 0, 0
    self.beta1 = 0.9
    self.beta2 = 0.999
    self.epsilon = 1e-8
    
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
      print(f"DOAOpt: publishing current doa -> {self.curr_doa[0]}")
      msg = Float32()
      msg.data = self.curr_doa[0]
      self.publisher.publish(msg)
      
      #print(f"DOAOpt: giving time for the system to react to new theta...")
      time.sleep(0.1)
      
      #print(f"DOAOpt: reading new SDR value...")
      self.request_sdr = True
      while self.request_sdr:
        time.sleep(0.001)
      #print(f"DOAOpt: SDR -> {self.curr_sdr} - {t}")
      t += 1
      
      #print(f"DOAOpt: doing optimization...")
      dw = self.gradient(self.curr_doa,self.curr_sdr) # 
      #print(f"DOAOpt: current gradient is {dw}")
      
      ## momentum beta 1
      self.m_dw = self.beta1*self.m_dw + (1-self.beta1)*dw
      
      ## rms beta 2
      self.v_dw = self.beta2*self.v_dw + (1-self.beta2)*(dw**2)
      
      ## bias correction
      m_dw_corr = self.m_dw/(1-self.beta1**t)
      v_dw_corr = self.v_dw/(1-self.beta2**t)
      #m_dw_corr = self.m_dw/(1-self.beta1)
      #v_dw_corr = self.v_dw/(1-self.beta2)
      #m_dw_corr = self.m_dw
      #v_dw_corr = self.v_dw
      #print(f"DOAOpt: m_dw {self.m_dw}, v_dw {self.v_dw}, m_dw_corr {m_dw_corr}, v_dw_corr {v_dw_corr}")
      
      ## update value
      self.curr_doa[1:] = self.curr_doa[:-1]
      self.curr_doa[0] = self.curr_doa[0] - self.eta*(m_dw_corr/(np.sqrt(v_dw_corr)+self.epsilon))

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
