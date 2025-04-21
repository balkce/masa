import rclpy
from rclpy.node import Node

from jack_msgs.msg import JackAudio

import wavfile
import torch

class WriteWAV(Node):
  def __init__(self, hop_secs=0.5):
    super().__init__('writewav')
    
    self.subscription = self.create_subscription(JackAudio, '/jackaudio_filtered', self.jackaudio_callback,10)
    self.subscription  # prevent unused variable warning
    self.subscription_ref = self.create_subscription(JackAudio, '/jackaudio_ref', self.jackaudio_callback_ref,10)
    #self.subscription_ref = self.create_subscription(JackAudio, '/jackaudio', self.jackaudio_callback_ref,10)
    self.subscription_ref  # prevent unused variable warning
    
    self.wavfile = wavfile.open('/home/balkce/demucs.wav', 'w',
                 sample_rate=16000,
                 num_channels=None,
                 bits_per_sample=16,
                 fmt=wavfile.chunk.WavFormat.PCM)
    self.wavfile_ref = wavfile.open('/home/balkce/demucs_ref.wav', 'w',
                 sample_rate=16000,
                 num_channels=None,
                 bits_per_sample=16,
                 fmt=wavfile.chunk.WavFormat.PCM)
    
  def jackaudio_callback(self, msg):
    #data_to_write = torch.tensor(msg.data).unsqueeze(0).tolist()
    data_to_write = [[m] for m in msg.data]
    #print(len(data_to_write))
    #print(len(data_to_write[0]))
    self.wavfile.write(data_to_write)
    print("filt: wrote " +str(len(msg.data))+" samples")

  def jackaudio_callback_ref(self, msg):
    #data_to_write = torch.tensor(msg.data).unsqueeze(0).tolist()
    data_to_write = [[m] for m in msg.data]
    #print(len(data_to_write))
    #print(len(data_to_write[0]))
    self.wavfile_ref.write(data_to_write)
    print("ref : wrote " +str(len(msg.data))+" samples")

def main(args=None):
  rclpy.init(args=args)
  writewav = WriteWAV()
  rclpy.spin(writewav)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  writewav.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
