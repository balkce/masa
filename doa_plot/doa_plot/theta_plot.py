import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

def move_figure(f, x, y):
    """Move figure's upper left corner to pixel (x, y)"""
    backend = matplotlib.get_backend()
    if backend == 'TkAgg':
        f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
    elif backend == 'WXAgg':
        f.canvas.manager.window.SetPosition((x, y))
    else:
        # This works for QT and GTK
        # You can also use window.setGeometry
        f.canvas.manager.window.move(x, y)

class ThetaPlot(Node):
  def __init__(self):
    super().__init__('thetaplot')
    
    self.declare_parameter('max_time', 0.0)
    self.max_time = self.get_parameter('max_time').get_parameter_value().double_value
    self.declare_parameter('wait_for_sdr', 1.5)
    self.wait_for_sdr = self.get_parameter('wait_for_sdr').get_parameter_value().double_value
    
    self.subscription = self.create_subscription(Float32,'theta',self.theta_callback,10)
    self.subscription  # prevent unused variable warning
    
    if self.max_time > 0.0:
      #self.fig_theta, self.ax_theta = plt.subplots(figsize=(6.4*2, 4.8*2), subplot_kw={'projection': 'polar'})
      self.fig_theta, self.ax_theta = plt.subplots(subplot_kw={'projection': 'polar'},num='DOAOptimizer Output (theta)')
    else:
      self.fig_theta, self.ax_theta = plt.subplots(subplot_kw={'projection': 'polar'})
    #self.fig_theta.canvas.manager.window.move(640,0)
    move_figure(self.fig_theta,640,0)
    (self.ln_theta,) = self.ax_theta.plot([0.0], [-0.1], linestyle='-', marker='o', markersize=5, animated=True)
    plt.show(block=False)
    plt.pause(0.1)
    
    self.ax_theta.set_rmax(1.1)
    self.ax_theta.set_rticks([0.25, 0.5, 0.75, 1.0])
    self.ax_theta.set_rlabel_position(0.0)
    self.ax_theta.set_yticklabels(["", "", "", ""])
    self.ax_theta.set_theta_zero_location("N")
    self.ax_theta.set_theta_direction('clockwise')
    self.ax_theta.set_xticks([0, 45*3.14159/180, 90*3.14159/180, 135*3.14159/180, 180*3.14159/180, 225*3.14159/180, 270*3.14159/180, 315*3.14159/180])
    self.ax_theta.set_xticklabels(["0", "45", "90", "135", "180", "-135", "-90", "-45"])
    
    plt.pause(0.1)
    
    self.bg_theta = self.fig_theta.canvas.copy_from_bbox(self.fig_theta.bbox)
    self.ax_theta.draw_artist(self.ln_theta)
    self.fig_theta.canvas.blit(self.fig_theta.bbox)
    
    if self.max_time > 0.0:
      self.get_logger().info("Plotting theta history...")
      self.get_logger().info("max. time: %0.2f" % self.max_time)
      self.get_logger().info("SDR hop  : %0.2f" % self.wait_for_sdr)
      
      self.fig_time, self.ax_time = plt.subplots(num='DOAOptimizer History')
      #self.fig_time.canvas.manager.window.move(1280,0)
      move_figure(self.fig_time,1280,0)
      
      (self.ln_time,) = self.ax_time.plot([0.0], [0.0], linestyle='-', marker='.', markersize=5, animated=True)
      self.ax_time.set_xlim(-0.1,self.max_time+0.1)
      self.ax_time.set_ylim(-15.0,25.0)
      self.ax_time.grid(True)
      plt.show(block=False)
      plt.pause(1)
      
      self.bg_time = self.fig_time.canvas.copy_from_bbox(self.fig_time.bbox)
      self.ax_time.draw_artist(self.ln_time)
      self.fig_time.canvas.blit(self.fig_time.bbox)
      
      self.theta_hist = []
      self.t = []
  
  def theta_callback(self,msg):
    #print(msg.data)
    
    self.fig_theta.canvas.restore_region(self.bg_theta)
    self.ln_theta.set_xdata([0.0, msg.data*3.14159/180])
    self.ln_theta.set_ydata([-0.1, 1.0])
    self.ax_theta.draw_artist(self.ln_theta)
    self.fig_theta.canvas.blit(self.fig_theta.bbox)
    self.fig_theta.canvas.flush_events()
    
    if self.max_time > 0.0:
      if len(self.t) == 0:
        self.theta_hist.append(msg.data)
        self.t.append(0.0)
      else:
        self.theta_hist.append(msg.data)
        self.t.append(self.t[-1]+self.wait_for_sdr)
      
      #print(self.theta_hist)
      #print(self.t)
      
      self.fig_time.canvas.restore_region(self.bg_time)
      
      self.ln_time.set_xdata(self.t)
      self.ln_time.set_ydata(self.theta_hist)
      
      self.ax_time.draw_artist(self.ln_time)
      self.fig_time.canvas.blit(self.fig_time.bbox)
      self.fig_time.canvas.flush_events()

def main(args=None):
  rclpy.init(args=args)
  thetaplot = ThetaPlot()
  rclpy.spin(thetaplot)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  thetaplot.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
