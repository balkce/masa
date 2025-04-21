import signal
import shutil

import rclpy
from rclpy.node import Node

from jack_cmd.srv import JACKcmd

from threading import Thread
import subprocess

# run auto_start:
#   ros2 run jack_control jack_control --ros-args -p auto_start:=True


class JACKWrapper(Node):
  def __init__(self):
    super().__init__('jack_control')
    
    self.declare_parameter('auto_start', False)
    self.auto_start = self.get_parameter('auto_start').get_parameter_value().bool_value
    self.declare_parameter('refresh_rate', 48000)
    self.refresh_rate = self.get_parameter('refresh_rate').get_parameter_value().integer_value
    self.declare_parameter('window_length', 1024)
    self.window_length = self.get_parameter('window_length').get_parameter_value().integer_value
    self.declare_parameter('period_num', 2)
    self.period_num = self.get_parameter('period_num').get_parameter_value().integer_value
    self.declare_parameter('duplex', True)
    self.duplex = self.get_parameter('duplex').get_parameter_value().bool_value
    self.declare_parameter('capture_device', "hw:CODEC")
    self.capture_device = self.get_parameter('capture_device').get_parameter_value().string_value
    self.declare_parameter('playback_device', "hw:CODEC")
    self.playback_device = self.get_parameter('playback_device').get_parameter_value().string_value
    
    self.srv = self.create_service(JACKcmd, 'JACKcmd', self.JACKcmd_callback)
    
    self.jack_thread = Thread(target=self.jack_start)
    
    self.jack_server_path = shutil.which("jackd")
    
    if self.jack_server_path == None:
      print("JACKd: jackd appears to not be installed. Exiting.")
      self.destroy_node()
      rclpy.shutdown()
      exit()
    
    print("JACK Server path: "+self.jack_server_path)
    
    if self.auto_start:
      print("JACKd: starting jackd server...")
      self.jack_thread.start()
      subprocess.run(['/usr/bin/jack_wait','-w'])
      print("JACKd: jackd server started.")
  
  def jack_stop(self, sig, frame):
    if self.jack_thread.is_alive():
      print("JACKd: stopping jackd server...")
      subprocess.run(['/usr/bin/killall','jackd'])
      self.jack_thread.join()
      #subprocess.run(['/usr/bin/jack_wait','-q'])
      print("JACKd: jackd server stopped.")
    
    self.destroy_node()
    rclpy.shutdown()
  
  
  def jack_start(self):
    jackarg_refresh_rate  = "-r"+str(self.refresh_rate)
    jackarg_window_length = "-p"+str(self.window_length)
    jackarg_period_num    = "-n"+str(self.period_num)
    jackarg_duplex = ""
    if self.duplex:
      jackarg_duplex = "-D"
    jackarg_capture_device  = "-C"+self.capture_device
    jackarg_playback_device  = "-P"+self.playback_device
    
    subprocess.run([self.jack_server_path,'-dalsa',jackarg_refresh_rate,jackarg_window_length,jackarg_period_num,jackarg_duplex,jackarg_capture_device,jackarg_playback_device])
  
  def JACKcmd_callback(self, request, response):
    #print(request)
    if request.command == "start":
      if self.jack_thread.is_alive():
        print("JACKd: jackd already started.")
        response.error = 1
      else:
        print("JACKd: starting jackd server...")
        self.jack_thread.start()
        subprocess.run(['/usr/bin/jack_wait','-w'])
        print("JACKd: jackd server started.")
        response.error = 0
    elif request.command == "stop":
      if self.jack_thread.is_alive():
        print("JACKd: stopping jackd server...")
        subprocess.run(['/usr/bin/killall','jackd'])
        self.jack_thread.join()
        self.jack_thread = Thread(target=self.jack_start)
        #subprocess.run(['/usr/bin/jack_wait','-q'])
        print("JACKd: jackd server stopped.")
        response.error = 0
      else:
        print("JACKd: jackd already stopped.")
        response.error = 1
    elif request.command == "status":
      if self.jack_thread.is_alive():
        response.error = 0
      else:
        response.error = 1
    
    return response

def main(args=None):
  rclpy.init(args=args)
  jackwrapper = JACKWrapper()
  signal.signal(signal.SIGINT, jackwrapper.jack_stop)
  rclpy.spin(jackwrapper)


if __name__ == '__main__':
  main()
