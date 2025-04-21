import rclpy
from rclpy.node import Node

from soundloc.msg import DOA

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


class DOAPlot(Node):
  def __init__(self):
    super().__init__('doaplot')
    
    self.max_confidence = 14
    
    self.subscription = self.create_subscription(DOA,'theta_est',self.doa_callback,10)
    self.subscription  # prevent unused variable warning
    
    self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'},num='SoundLoc Output (theta_est)')
    #self.fig.canvas.manager.window.move(0,0)
    move_figure(self.fig,0,0)
    (self.ln,) = self.ax.plot([0.0], [0.0], linestyle='', marker='.', markersize=20, animated=True)
    plt.show(block=False)
    plt.pause(0.1)
    
    self.ax.set_rmax(1.1)
    self.ax.set_rticks([0.25, 0.5, 0.75, 1.0])
    self.ax.set_rlabel_position(0.0)
    self.ax.set_yticklabels(["", "", "", ""])
    self.ax.set_theta_zero_location("N")
    self.ax.set_theta_direction('clockwise')
    #self.ax.set_xticks([0, 45, 90, 135, 180, 225, 270, 315])
    self.ax.set_xticklabels(["0", "45", "90", "135", "180", "-135", "-90", "-45"])
    
    plt.pause(0.1)
    
    self.bg = self.fig.canvas.copy_from_bbox(self.fig.bbox)
    self.ax.draw_artist(self.ln)
    self.fig.canvas.blit(self.fig.bbox)
  
  def doa_callback(self,msg):
    #print(msg.doas)
    #print(msg.confs)
    
    #self.ax.plot(msg.doas, msg.confs)
    self.fig.canvas.restore_region(self.bg)
    self.ln.set_xdata(np.array(msg.doas)*3.14159/180)
    #self.ln.set_ydata(msg.confs)
    self.ln.set_ydata([1.0 if mag > self.max_confidence else mag/self.max_confidence for mag in msg.confs])
    self.ax.draw_artist(self.ln)
    self.fig.canvas.blit(self.fig.bbox)
    self.fig.canvas.flush_events()
    #plt.pause(1)
    

def main(args=None):
  rclpy.init(args=args)
  doaplot = DOAPlot()
  rclpy.spin(doaplot)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  doaplot.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
