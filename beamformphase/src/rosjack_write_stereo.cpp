/**
 * ROS agent that reads from ROS topic and outputs to speaker
 */

#include "rosjack/rosjack.hpp"

int jack_callback (jack_nframes_t nframes, void *arg){
  output_to_rosjack (input_from_ros2jack_buffer (nframes), nframes, ROSJACK_OUT_JACK);
  return 0;
}


int main (int argc, char *argv[]) {
  /* ROS initialization*/
  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::Node> rosjack_node = rclcpp::Node::make_shared("rosjack_write");
  
  bool second_channel = false;
  rosjack_node->declare_parameter("second_channel",false);
  
  if (rosjack_node->get_parameter("second_channel",second_channel)){
      RCLCPP_INFO(rosjack_node->get_logger(),"Output Second Channel: %d",second_channel);
  }else{
      second_channel = false;
      RCLCPP_WARN(rosjack_node->get_logger(),"Output Second Channel argument not found in ROS param server, using default value (%d).",second_channel);
  }
  
  /* create JACK agent */
  if (second_channel){
    if(rosjack_create (ROSJACK_WRITE_STEREO2, rosjack_node, "jackaudiostereo", rosjack_node->get_name(), 0, jack_callback)){
      RCLCPP_ERROR(rosjack_node->get_logger(),"JACK agent could not be created.\n");
      rclcpp::shutdown();
      exit(1);
    }
  }else{
    if(rosjack_create (ROSJACK_WRITE_STEREO, rosjack_node, "jackaudiostereo", rosjack_node->get_name(), 0, jack_callback)){
      RCLCPP_ERROR(rosjack_node->get_logger(),"JACK agent could not be created.\n");
      rclcpp::shutdown();
      exit(1);
    }
  }
  
  output_type = 2;
  auto_connect = 1;
  
  RCLCPP_INFO(rosjack_node->get_logger(),"Beamform node started.");
  
  /* keep running until stopped by the user */
  rclcpp::spin(rosjack_node);
  
  exit(0);
}
