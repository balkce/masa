import shutil
import re

import rclpy
from rclpy.node import Node

import subprocess


class JACKListDevices(Node):
  def __init__(self):
    super().__init__('jack_listdevices')
    
    self.declare_parameter('devicetype', "all")
    self.devicetype = self.get_parameter('devicetype').get_parameter_value().string_value
    
    if self.devicetype != "all" and self.devicetype != "playback" and self.devicetype != "capture":
      print('JACKListDevices: Invalid device type ("all", "playback" or "capture"). Exiting.')
      self.destroy_node()
      rclpy.shutdown()
      exit()
    
    
    self.aplay_path = shutil.which("aplay")
    
    if self.aplay_path == None:
      print("JACKListDevices: aplay appears to not be installed. Exiting.")
      self.destroy_node()
      rclpy.shutdown()
      exit()
    
    self.arecord_path = shutil.which("arecord")
    
    if self.arecord_path == None:
      print("JACKListDevices: arecord appears to not be installed. Exiting.")
      self.destroy_node()
      rclpy.shutdown()
      exit()
    
    self.re_card = re.compile("card \d+:")
    self.re_device = re.compile("device \d+:")
    #self.re_label = re.compile(": \w+ \[[ \w]+\],")
    self.re_label = re.compile(": (.*?) \[(.*?)\],")
    
    if self.devicetype == "all" or self.devicetype == "playback":
      aplay_output = str(subprocess.check_output([self.aplay_path, "-l"]))
      aplay_output_lines = aplay_output.split("\\n")
      
      print("JACKListDevices: playback devices...")
      playback_devices = []
      for line in aplay_output_lines:
        if "card" in line:
          card_re = self.re_card.findall(line)[0]
          device_re = self.re_device.findall(line)[0]
          label_re = self.re_label.findall(line)[0]
          
          card = card_re.split("card ")[1].split(":")[0]
          device = device_re.split("device ")[1].split(":")[0]
          label = label_re[0]
          
          description = line.split(device_re)[1].split(" [")[0]
          
          device_label = "hw:"+label
          if device != "0":
            device_label += ","+device
            device_str = "\t"+device_label+"\t-> "+label_re[1]+","+description
            playback_devices.append(device_str)
          else:
            device_str = "\t"+device_label+"\t-> "+label_re[1]+","+description
            playback_devices.append(device_str)
            
            device_label += ","+device
            device_str = "\t"+device_label+"\t-> "+label_re[1]+","+description
            playback_devices.append(device_str)
      
      for playback_device in playback_devices:
        print(playback_device)
    
    if self.devicetype == "all" or self.devicetype == "capture":
      arecord_output = str(subprocess.check_output([self.arecord_path, "-l"]))
      arecord_output_lines = arecord_output.split("\\n")
      
      print("JACKListDevices: capture devices...")
      capture_devices = []
      for line in arecord_output_lines:
        if "card" in line:
          card_re = self.re_card.findall(line)[0]
          device_re = self.re_device.findall(line)[0]
          label_re = self.re_label.findall(line)[0]
          
          card = card_re.split("card ")[1].split(":")[0]
          device = device_re.split("device ")[1].split(":")[0]
          label = label_re[0]
          
          description = line.split(device_re)[1].split(" [")[0]
          
          device_label = "hw:"+label
          if device != "0":
            device_label += ","+device
            device_str = "\t"+device_label+"\t-> "+label_re[1]+","+description
            capture_devices.append(device_str)
          else:
            device_str = "\t"+device_label+"\t-> "+label_re[1]+","+description
            capture_devices.append(device_str)
            
            device_label += ","+device
            device_str = "\t"+device_label+"\t-> "+label_re[1]+","+description
            capture_devices.append(device_str)
      
      for capture_devices in capture_devices:
        print(capture_devices)
    
    self.destroy_node()
    rclpy.shutdown()
    exit()

def main(args=None):
  rclpy.init(args=args)
  jacklistdevices = JACKListDevices()
  rclpy.spin(jacklistdevices)


if __name__ == '__main__':
  main()
