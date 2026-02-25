import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

import time
from threading import Thread

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

class QualPlot(Node):
  def __init__(self):
    super().__init__('qualplot')
    
    self.declare_parameter('max', 120.0)
    self.max = self.get_parameter('max').get_parameter_value().double_value
    if self.max <= 0.0:
      print("max needs to be greater than 0.0. Defaulting to: 120.0")
      self.max = 120.0
    self.declare_parameter('wait_for_qual', 1.5)
    self.wait_for_qual = self.get_parameter('wait_for_qual').get_parameter_value().double_value
    
    self.sdr_subscription = self.create_subscription(Float32,'/SDR',self.sdr_callback,10)
    self.sdr_subscription  # prevent unused variable warning
    self.stoi_subscription = self.create_subscription(Float32,'/STOI',self.stoi_callback,10)
    self.stoi_subscription  # prevent unused variable warning
    self.pesq_subscription = self.create_subscription(Float32,'/PESQ',self.pesq_callback,10)
    self.pesq_subscription  # prevent unused variable warning
    self.scoreq_subscription = self.create_subscription(Float32,'/SCOREQ',self.scoreq_callback,10)
    self.scoreq_subscription  # prevent unused variable warning
    self.audbox_subscription = self.create_subscription(Float32,'/AUDBOX',self.audbox_callback,10)
    self.audbox_subscription  # prevent unused variable warning
    
    self.get_logger().info("Plotting quality history...")
    self.get_logger().info("max. time: %0.2f" % self.max)
    self.get_logger().info("time hop : %0.2f" % self.wait_for_qual)
    
    self.fig, self.ax = plt.subplots(num='Quality History')
    self.fig.subplots_adjust(right=0.75)
    move_figure(self.fig,1280,600)
    
    (self.ln_sdr,) = self.ax.plot([0.0], [0.0], "C0", linestyle='-', marker='.', markersize=5, animated=True, label="SDR")
    self.ax.set_xlim(-0.1,self.max+0.1)
    self.ax.set_ylim(0.0,25.0)
    self.ax.yaxis.label.set_color(self.ln_sdr.get_color())
    self.ax.tick_params(axis='y', colors=self.ln_sdr.get_color())
    
    self.twin_stoi = self.ax.twinx()
    (self.ln_stoi,) = self.twin_stoi.plot([0.0], [0.0], "C1", linestyle='-', marker='.', markersize=5, animated=True, label="STOI")
    self.twin_stoi.set_ylim(0.0,1.0)
    self.twin_stoi.yaxis.label.set_color(self.ln_stoi.get_color())
    self.twin_stoi.tick_params(axis='y', colors=self.ln_stoi.get_color())
    
    self.twin_pesq = self.ax.twinx()
    self.twin_pesq.spines.right.set_position(("axes", 1.1))
    (self.ln_pesq,) = self.twin_pesq.plot([0.0], [0.0], "C2", linestyle='-', marker='.', markersize=5, animated=True, label="PESQ")
    self.twin_pesq.set_ylim(0.0,5.0)
    self.twin_pesq.yaxis.label.set_color(self.ln_pesq.get_color())
    self.twin_pesq.tick_params(axis='y', colors=self.ln_pesq.get_color())
    
    self.twin_scoreq = self.ax.twinx()
    self.twin_scoreq.spines.right.set_position(("axes", 1.2))
    (self.ln_scoreq,) = self.twin_scoreq.plot([0.0], [0.0], "C3", linestyle='-', marker='.', markersize=5, animated=True, label="SCOREQ")
    self.twin_scoreq.set_ylim(0.0,5.0)
    self.twin_scoreq.yaxis.label.set_color(self.ln_scoreq.get_color())
    self.twin_scoreq.tick_params(axis='y', colors=self.ln_scoreq.get_color())
    
    self.twin_audbox = self.ax.twinx()
    self.twin_audbox.spines.right.set_position(("axes", 1.3))
    (self.ln_audbox,) = self.twin_audbox.plot([0.0], [0.0], "C4", linestyle='-', marker='.', markersize=5, animated=True, label="AUDBOX")
    self.twin_audbox.set_ylim(0.0,10.0)
    self.twin_audbox.yaxis.label.set_color(self.ln_audbox.get_color())
    self.twin_audbox.tick_params(axis='y', colors=self.ln_audbox.get_color())
    
    self.fig.legend()
    self.ax.grid(True)
    plt.show(block=False)
    plt.pause(1)
    
    self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
    self.ax.draw_artist(self.ln_sdr)
    self.ax.draw_artist(self.ln_stoi)
    self.ax.draw_artist(self.ln_pesq)
    self.ax.draw_artist(self.ln_scoreq)
    self.ax.draw_artist(self.ln_audbox)
    self.fig.canvas.blit(self.fig.bbox)
    
    self.sdr_hist = []
    self.sdr_t = []
    self.stoi_hist = []
    self.stoi_t = []
    self.pesq_hist = []
    self.pesq_t = []
    self.scoreq_hist = []
    self.scoreq_t = []
    self.audbox_hist = []
    self.audbox_t = []
    
    self.sdr_updated = False
    self.stoi_updated = False
    self.pesq_updated = False
    self.scoreq_updated = False
    self.audbox_updated = False
  
  def refresh_plot(self):
    self.fig.canvas.restore_region(self.bg)
    
    self.ln_sdr.set_xdata(self.sdr_t)
    self.ln_sdr.set_ydata(self.sdr_hist)
    self.ax.draw_artist(self.ln_sdr)
    
    self.ln_stoi.set_xdata(self.stoi_t)
    self.ln_stoi.set_ydata(self.stoi_hist)
    self.ax.draw_artist(self.ln_stoi)
    
    self.ln_pesq.set_xdata(self.pesq_t)
    self.ln_pesq.set_ydata(self.pesq_hist)
    self.ax.draw_artist(self.ln_pesq)
    
    self.ln_scoreq.set_xdata(self.scoreq_t)
    self.ln_scoreq.set_ydata(self.scoreq_hist)
    self.ax.draw_artist(self.ln_scoreq)
    
    self.ln_audbox.set_xdata(self.audbox_t)
    self.ln_audbox.set_ydata(self.audbox_hist)
    self.ax.draw_artist(self.ln_audbox)
    
    self.fig.canvas.blit(self.fig.bbox)
    self.fig.canvas.flush_events()
    
    self.sdr_updated = False
    self.stoi_updated = False
    self.pesq_updated = False
    self.scoreq_updated = False
    self.audbox_updated = False
  
  def sdr_callback(self,msg):
    if len(self.sdr_t) == 0:
      self.sdr_hist.append(msg.data)
      self.sdr_t.append(0.0)
    else:
      self.sdr_hist.append(msg.data)
      self.sdr_t.append(self.sdr_t[-1]+self.wait_for_qual)
    
    self.sdr_updated = True
    
    if self.sdr_updated and self.stoi_updated and self.pesq_updated and self.scoreq_updated and self.audbox_updated:
      self.refresh_plot()
  
  def stoi_callback(self,msg):
    if len(self.stoi_t) == 0:
      self.stoi_hist.append(msg.data)
      self.stoi_t.append(0.0)
    else:
      self.stoi_hist.append(msg.data)
      self.stoi_t.append(self.stoi_t[-1]+self.wait_for_qual)
    
    self.stoi_updated = True
    
    if self.sdr_updated and self.stoi_updated and self.pesq_updated and self.scoreq_updated and self.audbox_updated:
      self.refresh_plot()
  
  def pesq_callback(self,msg):
    if len(self.pesq_t) == 0:
      self.pesq_hist.append(msg.data)
      self.pesq_t.append(0.0)
    else:
      self.pesq_hist.append(msg.data)
      self.pesq_t.append(self.pesq_t[-1]+self.wait_for_qual)
    
    self.pesq_updated = True
    
    if self.sdr_updated and self.stoi_updated and self.pesq_updated and self.scoreq_updated and self.audbox_updated:
      self.refresh_plot()
  
  def scoreq_callback(self,msg):
    if len(self.scoreq_t) == 0:
      self.scoreq_hist.append(msg.data)
      self.scoreq_t.append(0.0)
    else:
      self.scoreq_hist.append(msg.data)
      self.scoreq_t.append(self.scoreq_t[-1]+self.wait_for_qual)
    
    self.scoreq_updated = True
    
    if self.sdr_updated and self.stoi_updated and self.pesq_updated and self.scoreq_updated and self.audbox_updated:
      self.refresh_plot()
  
  def audbox_callback(self,msg):
    if len(self.audbox_t) == 0:
      self.audbox_hist.append(msg.data)
      self.audbox_t.append(0.0)
    else:
      self.audbox_hist.append(msg.data)
      self.audbox_t.append(self.audbox_t[-1]+self.wait_for_qual)
    
    self.audbox_updated = True
    
    if self.sdr_updated and self.stoi_updated and self.pesq_updated and self.scoreq_updated and self.audbox_updated:
      self.refresh_plot()

def main(args=None):
  rclpy.init(args=args)
  qualplot = QualPlot()
  rclpy.spin(qualplot)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  qualplot.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
