/**
 * Functions to ease connection to JACK
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <signal.h>
#include <sys/types.h>
#include <pwd.h>
#include <mutex>

#include <jack/jack.h>

//for ringbuffer handling
#include <jack/ringbuffer.h>

#include <sndfile.h>

#include <samplerate.h>

/*** ROS libraries ***/
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "jack_msgs/msg/jack_audio.hpp"
/*** End ROS libraries ***/

#define ROSJACK_OUT_BOTH 0
#define ROSJACK_OUT_JACK 1
#define ROSJACK_OUT_ROS 2
#define ROSJACK_OUT_ENUM 3

#define ROSJACK_READ 0
#define ROSJACK_WRITE 1
#define ROSJACK_READ_STEREO 2
#define ROSJACK_WRITE_STEREO 3

typedef jack_default_audio_sample_t rosjack_data;

extern int rosjack_type;
extern char *rosjack_home_path;

//sndfile stuff
extern char audio_file_path[];
extern SNDFILE * audio_file;
extern SF_INFO audio_info;
extern float *write_file_buffer;
extern int write_file_count;

//samplerate stuff
extern unsigned int ros_output_sample_rate;
extern double stamp_factor;
extern bool ros_output_sample_rate_defined;
#define DEFAULT_CONVERTER SRC_SINC_FASTEST
extern unsigned int samplerate_circbuff_size;
extern unsigned int samplerate_circbuff_w;
extern unsigned int samplerate_circbuff_r;
extern SRC_STATE * samplerate_conv;
extern SRC_STATE * samplerate_conv_int;
extern SRC_DATA samplerate_data;
extern SRC_DATA samplerate_data_int;
extern float * samplerate_buff_in;
extern float * samplerate_buff_in_int;
extern rosjack_data * samplerate_circbuff;
extern rosjack_data * samplerate_circbuff_int;


extern const char *ROSJACK_OUT_OUTPUT_TYPES[]; 

extern jack_port_t    **jack_input_port;
extern jack_port_t    *jack_output_port;
extern jack_client_t  *jack_client;
extern int jack_num_inputs;
extern int output_type;
extern rclcpp::Publisher<jack_msgs::msg::JackAudio>::SharedPtr rosjack_out;
extern rclcpp::Subscription<jack_msgs::msg::JackAudio>::SharedPtr rosjack_in;

extern std::mutex jack_mtx;

extern bool auto_connect;
extern bool write_file;
extern bool write_xrun;

extern unsigned int xruns_count;

extern rosjack_data *ros2jack_buffer;
extern unsigned int  ros2jack_buffer_size;
extern unsigned int ros2jack_buffer_size_r;
extern unsigned int ros2jack_buffer_size_w;
extern unsigned int rosjack_window_size;
extern unsigned int rosjack_sample_rate;

void rosjack_handle_params(std::shared_ptr<rclcpp::Node> rosjack_node);
void jack_shutdown (void *arg);
int jack_xrun (void *arg);
void rosjack_roscallback(const jack_msgs::msg::JackAudio::SharedPtr msg);
int rosjack_create (int rosjack_type, std::shared_ptr<rclcpp::Node> rosjack_node, const char *topic_name, const char *client_name, int input_number, int (*callback_function)(jack_nframes_t, void*));
void close_rosjack();
void siginthandler(int sig);
void convert_to_sample_rate(rosjack_data *data_in, unsigned int data_length);
void convert_to_sample_rate_stereo(rosjack_data *data_in, unsigned int data_length);
bool convert_to_sample_rate_ready(unsigned int data_length);
rclcpp::Time get_stamp(jack_nframes_t last_frame_time);
void output_to_rosjack (rosjack_data *data, unsigned int data_length, int output_type);
void output_to_rosjack (rosjack_data *data, unsigned int data_length);
rosjack_data ** input_from_rosjack (unsigned int data_length);
rosjack_data * input_from_ros2jack_buffer (unsigned int data_length);
