import signal
import shutil
import sys
import time
import rclpy
from rclpy.node import Node

import io
from contextlib import redirect_stdout

from terminatorlib.util import dbg, err
from terminatorlib.version import APP_VERSION
try:
  from terminatorlib import ipc
except ImportError:
  err('Unable to initialise Terminator remote library. This probably means dbus is not available')
  sys.exit(1)

from threading import Thread
import subprocess

def cmd_str (cmd):
  return 'bash --init-file <(echo "source ~/.bashrc; '+cmd+'; history -r; history -s '+cmd+' ")'

def term_command(command_label, options, uuid=None,):
  func = getattr(ipc, command_label)
  
  if uuid==None:
    f = io.StringIO()
    with redirect_stdout(f):
        func(options)
    return f.getvalue()
  else:
    options['uuid'] = uuid
    f = io.StringIO()
    with redirect_stdout(f):
        func(uuid,options)
    return f.getvalue()

class MASACoord(Node):
  def __init__(self):
    super().__init__('masacoord')
    
    self.terminator_path = shutil.which("terminator")
    if self.terminator_path == None:
      self.get_logger().info("terminator appears to not be installed. Exiting.")
      self.destroy_node()
      rclpy.shutdown()
      exit()
    self.get_logger().info("Terminator path -> "+self.terminator_path)
    
    self.declare_parameter('input_length', 0.512)
    self.input_length = str(self.get_parameter('input_length').get_parameter_value().double_value)
    self.declare_parameter('init_doa', 15.0)
    self.init_doa = str(self.get_parameter('init_doa').get_parameter_value().double_value)
    self.declare_parameter('eta', 0.3)
    self.eta = str(self.get_parameter('eta').get_parameter_value().double_value)
    self.declare_parameter('wait_for_qual', 1.5)
    self.wait_for_qual = str(self.get_parameter('wait_for_qual').get_parameter_value().double_value)
    self.declare_parameter('max_time', 120.0)
    self.max_time = str(self.get_parameter('max_time').get_parameter_value().double_value)
    self.declare_parameter('smooth_weight', 0.9)
    self.smooth_weight = str(self.get_parameter('smooth_weight').get_parameter_value().double_value)
    self.declare_parameter('merge_doas', True)
    self.merge_doas = str(self.get_parameter('merge_doas').get_parameter_value().bool_value)
    self.declare_parameter('quality_type', 'sdr')
    self.quality_type = self.get_parameter('quality_type').get_parameter_value().string_value
    if self.quality_type != 'sdr' and self.quality_type != 'stoi' and self.quality_type != 'pesq':
      print("invalid quality_type value ("+str(self.quality_type)+"). Can only be 'sdr', 'stoi', or 'pesq'. Defaulting to 'stoi'.")
      self.quality_type = 'stoi'
    
    self.masa_nodes ={
      'jackd':
        {
          'cmd': 'ros2 run jack_control jack_control --ros-args -p auto_start:=True',
          'cmd_fi': None,
          'cmd_fu':
            [
              ['sleep','2']
            ]
        },
      
      'beamformer':
        {
          'cmd': 'ros2 launch beamformphase phase.launch',
          'cmd_fi': None,
          'cmd_fu':
            [
              ['sleep','2'],
              ['jack_disconnect','beamformphase:output','system:playback_1']
            ]
        },
      
      'beamformermix':
        {
          'cmd': 'ros2 launch beamformphase phasemix.launch',
          'cmd_fi': None,
          'cmd_fu':
            [
              ['sleep','2'],
              ['jack_disconnect','beamformphase:output','system:playback_1']
            ]
        },
      
      'demucs':
        {
          'cmd': 'ros2 run demucs demucs  --ros-args -p input_length:='+self.input_length+'',
          'cmd_fi': None,
          'cmd_fu': None
        },
      
      'demucsmix':
        {
          'cmd': 'ros2 run demucsmix demucsmix  --ros-args -p input_length:='+self.input_length+'',
          'cmd_fi': None,
          'cmd_fu': None
        },
      
      'jack_write':
        {
          'cmd': 'ros2 launch beamformphase rosjack_write.launch',
          #'cmd': 'ros2 launch beamformphase rosjack_writewav.launch',
          'cmd_fi': None,
          'cmd_fu': None
        },
      
      'jack_writemix':
        {
          'cmd': 'ros2 launch beamformphase rosjack_write_stereo.launch',
          #'cmd': 'ros2 launch beamformphase rosjack_writewav_stereo.launch',
          'cmd_fi': None,
          'cmd_fu': None
        },
      
      'online_sqa':
        {
          'cmd': 'ros2 run online_sqa online_sqa --ros-args -p hop_secs:='+self.wait_for_qual+' -p smooth_weight:='+self.smooth_weight+'',
          'cmd_fi': None,
          'cmd_fu':
            [
              ['sleep','1']
            ]
        },
      
      'soundloc':
        {
          'cmd': 'ros2 launch soundloc soundloc.launch',
          'cmd_fi': None,
          'cmd_fu':
            [
              ['sleep','2'],
              ['jack_connect','system:capture_1','soundloc:input_1'],
              ['jack_connect','system:capture_2','soundloc:input_2'],
              ['jack_connect','system:capture_3','soundloc:input_3']
            ]
        },
      
      'doaoptimizer':
        {
          'cmd': 'ros2 run doaoptimizer doaoptimizer_fb  --ros-args -p wait_for_qual:='+self.wait_for_qual+' -p eta:='+self.eta+' -p quality_type:='+self.quality_type+' -p merge_doas:='+self.merge_doas+'',
          'cmd_fi':
            [
              ['sleep','1']
            ],
          'cmd_fu': None
        },
      
      'doaoptimizer_dummy':
        {
          'cmd': 'ros2 run doaoptimizer doaoptimizer_dummy  --ros-args -p wait_for_qual:='+self.wait_for_qual+'',
          'cmd_fi':
            [
              ['sleep','1']
            ],
          'cmd_fu': None
        },
      
      'freqselect':
        {
          'cmd': 'ros2 run freqselect freqselect',
          'cmd_fi': None,
          'cmd_fu':
            [
              ['sleep','2'],
              ['jack_connect','system:capture_1','freqselect:input_1']
            ]
        },
      
      'theta_plot':
        {
          #'cmd': 'ros2 run doa_plot theta_plot --ros-args -p max_time:='+self.max_time+'',
          'cmd': 'ros2 run doa_plot theta_plot --ros-args -p max_time:='+self.max_time+' -p terminal_output:=True 2>&1 | tee theta_hist.txt',
          #'cmd': 'ros2 run doa_plot theta_plot',
          'cmd_fi': None,
          'cmd_fu': None
        },
      
      'doa_plot':
        {
          'cmd': 'ros2 run doa_plot doa_plot',
          'cmd_fi': None,
          'cmd_fu': None
        },
      
      'qual_plot':
        {
          'cmd': 'ros2 run doa_plot qual_plot',
          'cmd_fi':None,
          'cmd_fu':None
        },
      
      #'template1':
      #  {
      #    'cmd': '',
      #    'cmd_fi': None,
      #    'cmd_fu': None
      #  },
      
      #'template2':
      #  {
      #    'cmd': '',
      #    'cmd_fi':
      #      [
      #        ['command1','arg1'],
      #        ['command2','arg2']
      #      ],
      #    'cmd_fu':
      #      [
      #        ['command3','arg3'],
      #        ['command4','arg4']
      #      ]
      #  },
      
    }
    
    self.declare_parameter('masa_order', [""])
    self.masa_order_orig = self.get_parameter('masa_order').get_parameter_value().string_array_value
    self.masa_order = []
    for node in self.masa_order_orig:
      if node in self.masa_nodes:
        self.masa_order.append(node)
      else:
        self.get_logger().warning("Invalid node: "+node+". Ignoring.")
    
    self.get_logger().info("The following nodes are to be started, in this order:")
    for node in self.masa_order:
      self.get_logger().info("\t "+node)
    
    self.row_len = 4
    
    self.get_logger().info("starting terminator...")
    first_label = self.masa_order[0]
    first_node = self.masa_nodes[first_label]
    self.get_logger().info("\t --- "+first_label)
    self.run_fi(first_node)
    self.get_logger().info("\t starting node")
    self.get_logger().info("\t \t "+first_node['cmd'])
    first_cmd = cmd_str(first_node['cmd'])
    self.terminator_thread = Thread(target=self.term_start,args=(first_cmd,))
    self.terminator_thread.start()
    self.run_fu(first_node)
    time.sleep(0.25)
    self.get_logger().info("terminator started.")
    
    self.get_logger().info("STARTING ALL NODES...")
    curr_row_len = 1
    row_num = 0
    terms = term_command("get_terminals",{}).split("\n")[:-1]
    next_term = terms[0]
    next_term_split = "vsplit"
    for node_i in range(1,len(self.masa_order)):
      this_label = self.masa_order[node_i]
      this_node = self.masa_nodes[this_label]
      self.get_logger().info("--- "+this_label)
      self.run_fi(this_node)
      self.get_logger().info("\t starting node")
      self.get_logger().info("\t \t "+this_node['cmd'])
      term_command(next_term_split,{'execute':cmd_str(this_node['cmd']),'title':this_label},uuid=next_term)
      self.run_fu(this_node)
      
      terms = term_command("get_terminals",{}).split("\n")[:-1]
      
      curr_row_len += 1
      if curr_row_len >= self.row_len and row_num == 0:
        curr_row_len = 0
        row_num += 1
        next_term_split = "hsplit"
      
      if row_num == 0:
        if curr_row_len == 1:
          next_term = terms[0]
        elif curr_row_len == 2:
          next_term = terms[0]
        elif curr_row_len == 3:
          next_term = terms[1]
        else:
          next_term = terms[0]
      else:
          next_term = terms[-self.row_len]
    
    self.get_logger().info("ALL NODES STARTED.")
  
  def term_start(self,first_cmd):
    dbg ("remotinator starting up, version %s" % (APP_VERSION))
    subprocess.run([self.terminator_path,'--maximise','-e',first_cmd])
  
  def stop(self, sig, frame):
    #self.get_logger().info("stopping masa nodes...")
    
    if 'jackd' in self.masa_order:
      #self.get_logger().info("jackd was on of the nodes, killing at the end")
      subprocess.run(['/usr/bin/killall','jackd'])
    
    self.destroy_node()
    rclpy.shutdown()
  
  def run_fi (self, node):
    if node['cmd_fi'] != None:
      self.get_logger().info("\t before start: ")
      for cmd_arg in node['cmd_fi']:
        self.get_logger().info("\t \t "+str(cmd_arg))
        subprocess.run(cmd_arg)
  
  def run_fu (self, node):
    if node['cmd_fu'] != None:
      self.get_logger().info("\t after start : ")
      for cmd_arg in node['cmd_fu']:
        self.get_logger().info("\t \t "+str(cmd_arg))
        subprocess.run(cmd_arg)



def main(args=None):
  rclpy.init(args=args)
  masacoord = MASACoord()
  signal.signal(signal.SIGINT, masacoord.stop)
  rclpy.spin(masacoord)


if __name__ == '__main__':
  main()
