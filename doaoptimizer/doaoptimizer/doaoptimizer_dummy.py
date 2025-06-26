import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from soundloc.msg import DOA

import time

from threading import Thread

class DOAOptimizer(Node):
  def __init__(self):
    super().__init__('doaoptimizer')
    
    self.declare_parameter('wait_for_qual', 1.5)
    self.wait_for_qual = self.get_parameter('wait_for_qual').get_parameter_value().double_value
    
    self.latest_theta_est = None
    
    self.subscription_theta_est = self.create_subscription(DOA,'/theta_est',self.theta_est_callback,1000)
    self.subscription_theta_est  # prevent unused variable warning
    
    self.publisher = self.create_publisher(Float32, '/theta', 1000)
    
    self.opt_thread = Thread(target=self.do_doaopt)
    self.opt_thread.start()
  
  def theta_est_callback(self, msg):
    #print(msg.doas)
    #print(msg.confs)
    
    #use the one with the highest confidence
    highest_conf = 0.0
    highest_conf_i = 0
    for i in range(len(msg.confs)):
      if msg.confs[i] > highest_conf:
        highest_conf = msg.confs[i]
        highest_conf_i = i
    
    #if abs(msg.doas[highest_conf_i]) < 60:
    #  #prefer a DOA that is in front of the array
    
    self.latest_theta_est = msg.doas[highest_conf_i]
    
  
  def do_doaopt(self):
    t = 0
    
    self.get_logger().info("Waiting for first DOA estimation from soundloc...")
    while self.latest_theta_est == None:
      time.sleep(0.001)
    
    self.get_logger().info("Starting DOA dummy...")
    while True:
      
      msg = Float32()
      msg.data = self.latest_theta_est
      self.publisher.publish(msg)
      
      time.sleep(self.wait_for_qual)

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
