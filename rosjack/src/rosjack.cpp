/**
 * Functions to ease connection to JACK
 */

#include "rosjack.hpp"

int rosjack_type;
char *rosjack_home_path;

//sndfile stuff
char audio_file_path[FILENAME_MAX];
SNDFILE * audio_file;
SF_INFO audio_info;
float *write_file_buffer;
int write_file_count;

//samplerate stuff
unsigned int ros_output_sample_rate;
double stamp_factor;
bool ros_output_sample_rate_defined;
#define DEFAULT_CONVERTER SRC_SINC_FASTEST
unsigned int samplerate_circbuff_size;
unsigned int samplerate_circbuff_w=0;
unsigned int samplerate_circbuff_r=0;
SRC_STATE * samplerate_conv;
SRC_STATE * samplerate_conv_int;
SRC_DATA samplerate_data;
SRC_DATA samplerate_data_int;
float * samplerate_buff_in;
float * samplerate_buff_in_int;
rosjack_data * samplerate_circbuff;
rosjack_data * samplerate_circbuff_int;

const char *ROSJACK_OUT_OUTPUT_TYPES[] = {
  "ROSJACK_OUT_BOTH",
  "ROSJACK_OUT_JACK",
  "ROSJACK_OUT_ROS"
}; 

jack_port_t    **jack_input_port;
jack_port_t    *jack_output_port;
jack_client_t  *jack_client;
int jack_num_inputs = 1;
int output_type;
rclcpp::Publisher<jack_msgs::msg::JackAudio>::SharedPtr rosjack_out;
rclcpp::Subscription<jack_msgs::msg::JackAudio>::SharedPtr rosjack_in;

std::mutex jack_mtx;

bool auto_connect = true;
bool write_file = false;
bool write_xrun = false;

unsigned int xruns_count = 0;

rosjack_data *ros2jack_buffer;
unsigned int  ros2jack_buffer_size;
unsigned int ros2jack_buffer_size_r = 0;
unsigned int ros2jack_buffer_size_w = 0;
unsigned int rosjack_window_size = 0;
unsigned int rosjack_sample_rate = 0;

void rosjack_handle_params(std::shared_ptr<rclcpp::Node> rosjack_node){
  if ((rosjack_home_path = getenv("HOME")) == NULL) {
    rosjack_home_path = getpwuid(getuid())->pw_dir;
  }
  
  std::string node_name = rosjack_node->get_name();
  std::cout << "JACK ROS parameters: " << node_name << std::endl;
  
  rosjack_node->declare_parameter("output_type",ROSJACK_OUT_BOTH);
  rosjack_node->declare_parameter("auto_connect",true);
  rosjack_node->declare_parameter("write_file",false);
  rosjack_node->declare_parameter("write_file_path","");
  rosjack_node->declare_parameter("write_xrun",false);
  rosjack_node->declare_parameter("ros_output_sample_rate",0);

  if (rosjack_node->get_parameter("output_type",output_type)){
    if (output_type >=0 && output_type < ROSJACK_OUT_ENUM){
      RCLCPP_INFO(rosjack_node->get_logger(),"Output type: %s",ROSJACK_OUT_OUTPUT_TYPES[output_type]);
    }else{
      output_type = ROSJACK_OUT_BOTH;
      RCLCPP_WARN(rosjack_node->get_logger(),"Invalid output type argument, using default value (%s).",ROSJACK_OUT_OUTPUT_TYPES[output_type]);
    }
  }else{
    output_type = ROSJACK_OUT_BOTH;
    RCLCPP_WARN(rosjack_node->get_logger(),"Output type argument not found in ROS param server, using default value (%s).",ROSJACK_OUT_OUTPUT_TYPES[output_type]);
  }
  
  if (rosjack_node->get_parameter("auto_connect",auto_connect)){
    RCLCPP_INFO(rosjack_node->get_logger(),"Auto connect: %d",auto_connect);
  }else{
    auto_connect = true;
    RCLCPP_WARN(rosjack_node->get_logger(),"Auto connect argument not found in ROS param server, using default value (%d).",auto_connect);
  }
  
  if (rosjack_node->get_parameter("write_file",write_file)){
    RCLCPP_INFO(rosjack_node->get_logger(),"Write output to audio file: %d",write_file);
    if(write_file){
      std::string write_file_path;
      if (rosjack_node->get_parameter("write_file_path",write_file_path)){
        if(write_file_path.empty()){
          strcpy(audio_file_path,rosjack_home_path);
          strcat(audio_file_path,(char *)"/rosjack_write_file.wav");
          RCLCPP_WARN(rosjack_node->get_logger(),"Audio file path argument found in ROS param server is empty, using default value (%s).",audio_file_path);
        }else{
          strcpy(audio_file_path,write_file_path.c_str());
          RCLCPP_INFO(rosjack_node->get_logger(),"Audio file path: %s",audio_file_path);
        }
      }else{
        strcpy(audio_file_path,rosjack_home_path);
        strcat(audio_file_path,(char *)"/rosjack_write_file.wav");
        RCLCPP_WARN(rosjack_node->get_logger(),"Audio file path argument not found in ROS param server, using default value (%s).",audio_file_path);
      }
    }
  }else{
    write_file = false;
    RCLCPP_WARN(rosjack_node->get_logger(),"Write output to audio file argument not found in ROS param server, using default value (%d).",write_file);
  }
  
  if (rosjack_node->get_parameter("write_xrun",write_xrun)){
    RCLCPP_INFO(rosjack_node->get_logger(),"Write XRUN count to file: %d",write_xrun);
  }else{
    write_xrun = false;
    RCLCPP_WARN(rosjack_node->get_logger(),"Write XRUN count to file argument not found in ROS param server, using default value (%d).",write_xrun);
  }
  
  if (rosjack_node->get_parameter("ros_output_sample_rate",ros_output_sample_rate)){
    if (ros_output_sample_rate != 0){
      RCLCPP_INFO(rosjack_node->get_logger(),"ROS Output Sample Rate: %d",ros_output_sample_rate);
      ros_output_sample_rate_defined = true;
    }else{
      ros_output_sample_rate_defined = false;
      RCLCPP_WARN(rosjack_node->get_logger(),"ROS Output Sample Rate argument not defined in ROS param server, using JACK sample rate.");
    }
  }else{
    ros_output_sample_rate_defined = false;
    RCLCPP_WARN(rosjack_node->get_logger(),"ROS Output Sample Rate argument not found in ROS param server, using JACK sample rate.");
  }
}

void jack_shutdown (void *arg){
  exit (1);
}

int jack_xrun (void *arg){
  xruns_count++;
  //printf("xrun count: %d \n",xruns_count);
  return 0;
}


int rosjack_create (int rosjack_readwrite, std::shared_ptr<rclcpp::Node> rosjack_node, const char *topic_name, const char *client_name, int input_number, int (*callback_function)(jack_nframes_t, void*)) {
  /* ROS stuff */
  signal(SIGINT, siginthandler);
  rosjack_type = rosjack_readwrite;
  
  if(rosjack_type == ROSJACK_READ || rosjack_type == ROSJACK_READ_STEREO){
    rosjack_out = rosjack_node->create_publisher<jack_msgs::msg::JackAudio>(topic_name, 10);
  }
  
  rosjack_handle_params(rosjack_node);
  
  /* JACK initialization */
  int i;
  jack_num_inputs = input_number;
  RCLCPP_INFO(rosjack_node->get_logger(),"Connecting to Jack Server...");
  jack_options_t options = JackNoStartServer;
  jack_status_t status;
  
  /* open a client connection to the JACK server */
  jack_client = jack_client_open (client_name, options, &status);
  if (jack_client == NULL){
    /* if connection failed, say why */
    RCLCPP_ERROR(rosjack_node->get_logger(),"jack_client_open() failed, status = 0x%2.0x", status);
    if (status & JackServerFailed) {
      RCLCPP_ERROR(rosjack_node->get_logger(),"Unable to connect to JACK server.");
    }
    return 1;
  }
  
  /* if connection was successful, check if the name we proposed is not in use */
  if (status & JackNameNotUnique){
    client_name = jack_get_client_name(jack_client);
    RCLCPP_WARN(rosjack_node->get_logger(),"Warning: other agent with our name is running, `%s' has been assigned to us.", client_name);
  }
  
  /* tell the JACK server to call 'jack_callback()' whenever there is work to be done. */
  jack_set_process_callback (jack_client, callback_function, 0);
  
  
  /* tell the JACK server to call 'jack_shutdown()' if it ever shuts down,
     either entirely, or if it just decides to stop calling us. */
  jack_on_shutdown (jack_client, jack_shutdown, 0);
  
  /* tell the JACK server to call 'jack_shutdown()' if there is an xrun. */
  jack_set_xrun_callback (jack_client, jack_xrun, 0);
  
  
  /* display the current sample rate. */
  rosjack_window_size = jack_get_buffer_size (jack_client);
  RCLCPP_INFO(rosjack_node->get_logger(),"JACK window size: %d", rosjack_window_size);
  rosjack_sample_rate = jack_get_sample_rate (jack_client);
  stamp_factor = (double)1000000000/rosjack_sample_rate;
  RCLCPP_INFO(rosjack_node->get_logger(),"JACK sample rate: %d", rosjack_sample_rate);
  if(!ros_output_sample_rate_defined){
    ros_output_sample_rate = rosjack_sample_rate;
    RCLCPP_INFO(rosjack_node->get_logger(),"ROS Output Sample Rate: %d",ros_output_sample_rate);
  }
  
  
  /* create the agent input ports */
  jack_input_port = (jack_port_t **)malloc(sizeof(jack_port_t *)*jack_num_inputs);
  char input_port_name[100];
  for(i = 0; i < jack_num_inputs; i++){
    sprintf(input_port_name,"input_%d",i+1);
    jack_input_port[i] = jack_port_register (jack_client, input_port_name, JACK_DEFAULT_AUDIO_TYPE,JackPortIsInput, 0);
    
    /* check that the port were created succesfully */
    if ((jack_input_port[i] == NULL)) {
      RCLCPP_ERROR(rosjack_node->get_logger(),"Could not create input port %s. Have we reached the maximum amount of JACK input ports?",input_port_name);
      return 1;
    }
  }
  
  jack_output_port = jack_port_register (jack_client, "output", JACK_DEFAULT_AUDIO_TYPE,JackPortIsOutput, 0);
  if ((jack_output_port == NULL)) {
    RCLCPP_ERROR(rosjack_node->get_logger(),"Could not create output port. Have we reached the maximum amount of JACK output ports?");
    return 1;
  }
  
  int ros2jack_buffer_mult = 5;
  
  if(ros_output_sample_rate != rosjack_sample_rate){
    RCLCPP_INFO(rosjack_node->get_logger(),"Creating the sample rate converter for ROS output...");
    int samplerate_error;
    samplerate_conv = src_new (DEFAULT_CONVERTER,1,&samplerate_error);
    if(samplerate_conv == NULL){
      RCLCPP_ERROR(rosjack_node->get_logger(),"%s",src_strerror (samplerate_error));
      exit(1);
    }
    
    if (rosjack_type == ROSJACK_READ_STEREO){
      samplerate_conv_int = src_new (DEFAULT_CONVERTER,1,&samplerate_error);
      if(samplerate_conv_int == NULL){
        RCLCPP_ERROR(rosjack_node->get_logger(),"%s",src_strerror (samplerate_error));
        exit(1);
      }
    }
    
    if(rosjack_type == ROSJACK_WRITE || rosjack_type == ROSJACK_WRITE_STEREO){
      samplerate_data.src_ratio = (double)(((double)rosjack_sample_rate)/((double)ros_output_sample_rate));
    }else if (rosjack_type == ROSJACK_READ){
      samplerate_data.src_ratio = (double)(((double)ros_output_sample_rate)/((double)rosjack_sample_rate));
    }else{
      samplerate_data.src_ratio = (double)(((double)ros_output_sample_rate)/((double)rosjack_sample_rate));
      samplerate_data_int.src_ratio = (double)(((double)ros_output_sample_rate)/((double)rosjack_sample_rate));
    }
    
    RCLCPP_INFO(rosjack_node->get_logger(),"Using ROS Sample Rate ratio: %f", samplerate_data.src_ratio);
    if (src_is_valid_ratio (samplerate_data.src_ratio) == 0){
      RCLCPP_WARN(rosjack_node->get_logger(),"Warning: ROS Output Sample Rate change out of valid range. Using JACK sample rate as output.");
      ros_output_sample_rate = rosjack_sample_rate;
    }
    
    if(rosjack_type == ROSJACK_WRITE || rosjack_type == ROSJACK_WRITE_STEREO){
      int data_out_size = rosjack_window_size*samplerate_data.src_ratio;
      
      samplerate_circbuff_size = data_out_size*ros2jack_buffer_mult;
      samplerate_data.data_out = (float *)malloc(data_out_size*sizeof(float));
      samplerate_data.output_frames = data_out_size;
    }else if (rosjack_type == ROSJACK_READ){
      samplerate_circbuff_size = rosjack_window_size*ros2jack_buffer_mult;
      samplerate_data.data_out = (float *)malloc(rosjack_window_size*sizeof(float));
      samplerate_data.output_frames = rosjack_window_size;
    }else{
      samplerate_circbuff_size = rosjack_window_size*ros2jack_buffer_mult;
      samplerate_data.data_out = (float *)malloc(rosjack_window_size*sizeof(float));
      samplerate_data.output_frames = rosjack_window_size;
      samplerate_data_int.data_out = (float *)malloc(rosjack_window_size*sizeof(float));
      samplerate_data_int.output_frames = rosjack_window_size;
    }
    
    samplerate_circbuff = (rosjack_data *)malloc(samplerate_circbuff_size*sizeof(rosjack_data));
    samplerate_data.input_frames = 0;
    samplerate_data.end_of_input = 0;
    samplerate_buff_in = (float *)malloc(rosjack_window_size*sizeof(float));
    samplerate_data.data_in = samplerate_buff_in;
    if (rosjack_type == ROSJACK_READ_STEREO){
      samplerate_circbuff_int = (rosjack_data *)malloc(samplerate_circbuff_size*sizeof(rosjack_data));
      samplerate_data_int.input_frames = 0;
      samplerate_data_int.end_of_input = 0;
      samplerate_buff_in_int = (float *)malloc(rosjack_window_size*sizeof(float));
      samplerate_data_int.data_in = samplerate_buff_in_int;
    }
    
  }else{
    RCLCPP_INFO(rosjack_node->get_logger(),"ROS Output Sample Rate and JACK sample rate are the same. Not creating sample rate converter.");
  }
  
  if(write_file){
    RCLCPP_INFO(rosjack_node->get_logger(),"Writing ROS output in: %s",audio_file_path);
    
    //if(ros_output_sample_rate == rosjack_sample_rate || output_type != ROSJACK_OUT_JACK)
    //  audio_info.samplerate = rosjack_sample_rate;
    //else
    //  audio_info.samplerate = ros_output_sample_rate;
    
    audio_info.samplerate = rosjack_sample_rate;
    audio_info.channels = 1;
    audio_info.format = SF_FORMAT_WAV | SF_FORMAT_PCM_16;
    audio_file = sf_open (audio_file_path,SFM_WRITE,&audio_info);
    if(audio_file == NULL){
      RCLCPP_WARN(rosjack_node->get_logger(),"WARNING: Could not open file for writing output, with error: %s",sf_strerror(NULL));
      RCLCPP_WARN(rosjack_node->get_logger(),"WARNING: Continuing without file output.");
      write_file = false;
    }else{
      RCLCPP_INFO(rosjack_node->get_logger(),"Audio file info:");
      RCLCPP_INFO(rosjack_node->get_logger(),"\tSample Rate: %d",audio_info.samplerate);
      RCLCPP_INFO(rosjack_node->get_logger(),"\tChannels: %d",audio_info.channels);
      RCLCPP_INFO(rosjack_node->get_logger(),"\tFormat: WAV, PCM 16-bit");
      write_file_buffer = (float *)malloc(rosjack_window_size*sizeof(float));
    }
  }
  
  if(rosjack_type == ROSJACK_WRITE || rosjack_type == ROSJACK_WRITE_STEREO){
    ros2jack_buffer_size = jack_get_buffer_size (jack_client)*ros2jack_buffer_mult;
    ros2jack_buffer = (rosjack_data *)malloc(sizeof(rosjack_data)*ros2jack_buffer_size);
    RCLCPP_INFO(rosjack_node->get_logger(),"ROSJACK Buffer size: %d", ros2jack_buffer_size);
  }
  
  /* Tell the JACK server that we are ready to roll.
     Our jack_callback() callback will start running now. */
  if (jack_activate (jack_client)) {
    RCLCPP_ERROR(rosjack_node->get_logger(),"Cannot activate JACK agent.");
    return 1;
  }
  
  RCLCPP_INFO(rosjack_node->get_logger(),"Agent activated.");
  
  /* Connect the ports.  You can't do this before the client is
   * activated, because we can't make connections to clients
   * that aren't running.  Note the confusing (but necessary)
   * orientation of the driver backend ports: playback ports are
   * "input" to the backend, and capture ports are "output" from
   * it.
   */
  const char **serverports_names;
  if(auto_connect){
    RCLCPP_INFO(rosjack_node->get_logger(),"Connecting input ports... ");
     
    /* Assign our ports to a server ports*/
    // Find possible output server port names
    serverports_names = jack_get_ports (jack_client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput);
    if (serverports_names == NULL) {
      RCLCPP_ERROR(rosjack_node->get_logger(),"No available physical capture (server output) ports.");
      return 1;
    }
    // Connect the first available to our input port
    for(i = 0; i < jack_num_inputs; i++){
      if (jack_connect (jack_client, serverports_names[i], jack_port_name (jack_input_port[i]))) {
        RCLCPP_WARN(rosjack_node->get_logger(),"Cannot connect input port %s.",jack_port_name (jack_input_port[i]));
        RCLCPP_WARN(rosjack_node->get_logger(),"Not connecting any more input ports, sticking with the ones that were connected.\n");
        break;
      }
    }
    // free serverports_names variable for reuse in next part of the code
    free (serverports_names);
  }
  
  if(output_type != ROSJACK_OUT_ROS){
    // Find possible input server port names
    serverports_names = jack_get_ports (jack_client, NULL, NULL, JackPortIsPhysical|JackPortIsInput);
    if (serverports_names == NULL) {
      RCLCPP_ERROR(rosjack_node->get_logger(),"No available physical playback (server input) ports.");
      return 1;
    }
    // Connect the first available to our output port
    if (jack_connect (jack_client, jack_port_name (jack_output_port), serverports_names[0])) {
      RCLCPP_ERROR(rosjack_node->get_logger(),"Cannot connect output port.");
      return 1;
    }
    // free serverports_names variable for reuse in next part of the code
    free (serverports_names);
  }
  
  
  if(rosjack_type == ROSJACK_WRITE || rosjack_type == ROSJACK_WRITE_STEREO){
    rosjack_in = rosjack_node->create_subscription<jack_msgs::msg::JackAudio>(topic_name, 1000,
      [rosjack_node, topic_name](const jack_msgs::msg::JackAudio::SharedPtr msg) {
          rosjack_roscallback(msg);
        }
    );
    
    // if using any of the below alternatives
    //   makes sure to change the type of msg in rosjack_roscallback to just "jack_msgs::msg::JackAudio"
    //   instead of "jack_msgs::msg::JackAudio::::SharedPtr"
    //   in both rosjack.cpp and rosjack.hpp
    
    //rosjack_in = rosjack_node->create_subscription<jack_msgs::msg::JackAudio>(topic_name, 1000, rosjack_roscallback);
    //rosjack_in = rosjack_node->create_subscription<jack_msgs::msg::JackAudio>(topic_name, 1000, std::bind(&rosjack_roscallback, rosjack_node, std::placeholders::_1));
  }
  
  RCLCPP_INFO(rosjack_node->get_logger(),"done.");
  return 0;
}

void siginthandler(int sig){
  std::cout << "\nClosing JACK client." << std::endl;
  jack_client_close (jack_client);
  
  if(write_file){
    std::cout << "Closing file output." << std::endl;
    sf_close(audio_file);
  }
  
  if(write_xrun){
    char *xrun_file_path;
    if ((xrun_file_path = getenv("HOME")) == NULL) {
      xrun_file_path = getpwuid(getuid())->pw_dir;
    }
    strcat(xrun_file_path,(char *)"/rosjack_xrun_count.txt");
    std::cout << "Writing XRUN count file to: " << xrun_file_path << std::endl;
    FILE *xrun_file = fopen(xrun_file_path,"w");
    fprintf(xrun_file,"%d\n",xruns_count);
    fclose(xrun_file);
  }
  
  if(rosjack_type == ROSJACK_READ || rosjack_type == ROSJACK_READ_STEREO){
    std::cout << "Closing topic jackaudio publisher." << std::endl;
    rosjack_out.reset();
  }
  if(rosjack_type == ROSJACK_WRITE || rosjack_type == ROSJACK_WRITE_STEREO){
    std::cout << "Closing topic jackaudio subscriber." << std::endl;
    rosjack_in.reset();
  }
  
  std::cout << "Closing ROS node." << std::endl;
  rclcpp::shutdown();
}

void close_rosjack(){
  jack_client_close (jack_client);
  src_delete(samplerate_conv);
}

void convert_to_sample_rate(rosjack_data *data_in, unsigned int data_length){
  unsigned int i;
  int samplerate_error;
  
  if (samplerate_data.input_frames == 0){
    for (i=0;i<data_length;i++){
      samplerate_buff_in[i] = data_in[i];
    }
    samplerate_data.data_in = samplerate_buff_in;
    samplerate_data.input_frames = data_length;
  }
  
  
  if ((samplerate_error = src_process (samplerate_conv, &samplerate_data))){
    printf ("\nSample rate conversion error : %s\n", src_strerror (samplerate_error)) ;
    exit (1) ;
  }
  
  //Output to buffer
  for (i=0;i<samplerate_data.output_frames_gen;i++){
    samplerate_circbuff[samplerate_circbuff_w] = (rosjack_data)samplerate_data.data_out[i];
    samplerate_circbuff_w++;
    if(samplerate_circbuff_w >= samplerate_circbuff_size)
      samplerate_circbuff_w = 0;
  }
  
  samplerate_data.data_in += samplerate_data.input_frames_used;
  samplerate_data.input_frames -= samplerate_data.input_frames_used;
}

void convert_to_sample_rate_stereo(rosjack_data *data_in, unsigned int data_length){
  unsigned int i;
  int samplerate_error;
  
  if (samplerate_data.input_frames == 0){
    for (i=0;i<data_length/2;++i){
      samplerate_buff_in[i] = data_in[(i*2)];
      samplerate_buff_in_int[i] = data_in[(i*2)+1];
    }
    samplerate_data.data_in = samplerate_buff_in;
    samplerate_data.input_frames = data_length/2;
    samplerate_data_int.data_in = samplerate_buff_in_int;
    samplerate_data_int.input_frames = data_length/2;
  }
  
  if ((samplerate_error = src_process (samplerate_conv, &samplerate_data))){
    printf ("\nSample rate conversion error (est): %s\n", src_strerror (samplerate_error)) ;
    exit (1) ;
  }
  
  if ((samplerate_error = src_process (samplerate_conv_int, &samplerate_data_int))){
    printf ("\nSample rate conversion error (int): %s\n", src_strerror (samplerate_error)) ;
    exit (1) ;
  }
  
  //Output to buffer
  for (i=0;i<samplerate_data.output_frames_gen;++i){
    samplerate_circbuff[samplerate_circbuff_w] = (rosjack_data)samplerate_data.data_out[i];
    samplerate_circbuff_int[samplerate_circbuff_w] = (rosjack_data)samplerate_data_int.data_out[i];
    samplerate_circbuff_w++;
    if(samplerate_circbuff_w >= samplerate_circbuff_size)
      samplerate_circbuff_w = 0;
  }
  
  samplerate_data.data_in += samplerate_data.input_frames_used;
  samplerate_data.input_frames -= samplerate_data.input_frames_used;
  samplerate_data_int.data_in += samplerate_data_int.input_frames_used;
  samplerate_data_int.input_frames -= samplerate_data_int.input_frames_used;
}

bool convert_to_sample_rate_ready(unsigned int data_length){
  //printf("w: %d \t r: %d, length: %d \t",samplerate_circbuff_w,samplerate_circbuff_r, data_length);fflush(stdout);
  
  if(samplerate_circbuff_w - samplerate_circbuff_r >= data_length || (samplerate_circbuff_r > samplerate_circbuff_w && samplerate_circbuff_w+(samplerate_circbuff_size-samplerate_circbuff_r) >= data_length)){
    //printf("ready: 1 \n");fflush(stdout);
    return true;
  }else{
    //printf("ready: 0 \n");fflush(stdout);
    return false;
  }
}

rclcpp::Time get_stamp(jack_nframes_t last_frame_time){
    rclcpp::Time this_stamp((double)last_frame_time*stamp_factor);
    return this_stamp;
}

void output_to_rosjack (rosjack_data *data, unsigned int data_length, int out_type){
  output_type = out_type;
  output_to_rosjack (data, data_length);
}

void output_to_rosjack (rosjack_data *data_out, unsigned int data_length){
  /* This may seem as too much code repetition, but it's quicker this way online */
  /* The enclosing brackets in the switch cases are necessary to avoid re-definition errors */
  unsigned int j;
  
  if(ros_output_sample_rate == rosjack_sample_rate){
    
    switch(output_type){
      case ROSJACK_OUT_BOTH:{
        jack_msgs::msg::JackAudio out;
        out.size = data_length;
        out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
        rosjack_data *out_j = (rosjack_data *)jack_port_get_buffer (jack_output_port, data_length); 
        for (j = 0; j < data_length; ++j){
          out.data.push_back(data_out[j]);
          out_j[j] = data_out[j];
          if(fabs(data_out[j]) >= 1.0){
            std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[j]) << std::endl;
          }
        }
        rosjack_out->publish(out);
      }break;
      
      case ROSJACK_OUT_JACK:{
        rosjack_data *out_j = (rosjack_data *)jack_port_get_buffer (jack_output_port, data_length);
        for (j = 0; j < data_length; ++j){
          out_j[j] = data_out[j];
          if(fabs(data_out[j]) >= 1.0){
            std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[j]) << std::endl;
          }
        }
      }break;
      
      case ROSJACK_OUT_ROS:{
        jack_msgs::msg::JackAudio out;
        out.size = data_length;
        out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
        for (j = 0; j < data_length; ++j){
          out.data.push_back(data_out[j]);
          if(fabs(data_out[j]) >= 1.0){
            std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[j]) << std::endl;
          }
        }
        rosjack_out->publish(out);
      }break;
    }
    
    if(write_file){
      for (j = 0; j < data_length; ++j){
        write_file_buffer[j] = data_out[j];
      }
      write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
    }
  }else{
    
    if (rosjack_type == ROSJACK_READ_STEREO){
      switch(output_type){
        
        case ROSJACK_OUT_BOTH:{
          rosjack_data *out_j = (rosjack_data *)jack_port_get_buffer (jack_output_port, data_length/2);
          for (j = 0; j < data_length/2; ++j){
            out_j[j] = data_out[(2*j)];
            if(fabs(data_out[(2*j)]) >= 1.0){
              std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[(2*j)]) << std::endl;
            }
          }
          convert_to_sample_rate_stereo(data_out,data_length);
          if(write_file){
            if(convert_to_sample_rate_ready(data_length/2)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length/2; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                out.data.push_back(samplerate_circbuff_int[samplerate_circbuff_r]);
                write_file_buffer[j] = samplerate_circbuff[samplerate_circbuff_r];
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
              write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
            }
          }else{
            if(convert_to_sample_rate_ready(data_length/2)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length/2; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                out.data.push_back(samplerate_circbuff_int[samplerate_circbuff_r]);
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
            }
          }
        }break;
        
        case ROSJACK_OUT_JACK:{
          rosjack_data *out_j = (rosjack_data *)jack_port_get_buffer (jack_output_port, data_length);
          for (j = 0; j < data_length/2; ++j){
            out_j[j] = data_out[(2*j)];
            if(fabs(data_out[(2*j)]) >= 1.0){
              std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[(2*j)]) << std::endl;
            }
          }
          
          if(write_file){
            for (j = 0; j < data_length; j+=2){
              write_file_buffer[j] = data_out[j];
            }
            write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
          }
        }break;
        
        case ROSJACK_OUT_ROS:{
          convert_to_sample_rate(data_out,data_length);
          if(write_file){
            if(convert_to_sample_rate_ready(data_length)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                write_file_buffer[j] = samplerate_circbuff[samplerate_circbuff_r];
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
              write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
            }
          }else{
            if(convert_to_sample_rate_ready(data_length)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
            }
          }
        }break;
      }
    }else{
      switch(output_type){
        
        case ROSJACK_OUT_BOTH:{
          rosjack_data *out_j = (rosjack_data *)jack_port_get_buffer (jack_output_port, data_length);
          for (j = 0; j < data_length; ++j){
            out_j[j] = data_out[j];
            if(fabs(data_out[j]) >= 1.0){
              std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[j]) << std::endl;
            }
          }
          
          convert_to_sample_rate(data_out,data_length);
          if(write_file){
            if(convert_to_sample_rate_ready(data_length)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                write_file_buffer[j] = samplerate_circbuff[samplerate_circbuff_r];
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
              write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
            }
          }else{
            if(convert_to_sample_rate_ready(data_length)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
            }
          }
        }break;
        
        case ROSJACK_OUT_JACK:{
          rosjack_data *out_j = (rosjack_data *)jack_port_get_buffer (jack_output_port, data_length);
          for (j = 0; j < data_length; ++j){
            out_j[j] = data_out[j];
            if(fabs(data_out[j]) >= 1.0){
              std::cout << "Audio output out of [-1,1] range: " << fabs(data_out[j]) << std::endl;
            }
          }
          
          if(write_file){
            for (j = 0; j < data_length; ++j){
              write_file_buffer[j] = data_out[j];
            }
            write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
          }
        }break;
        
        case ROSJACK_OUT_ROS:{
          convert_to_sample_rate(data_out,data_length);
          if(write_file){
            if(convert_to_sample_rate_ready(data_length)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                write_file_buffer[j] = samplerate_circbuff[samplerate_circbuff_r];
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
              write_file_count = sf_write_float(audio_file,write_file_buffer,data_length);
            }
          }else{
            if(convert_to_sample_rate_ready(data_length)){
              jack_msgs::msg::JackAudio out;
              out.size = data_length;
              out.header.stamp = get_stamp(jack_last_frame_time(jack_client));
              for (j = 0; j < data_length; ++j){
                out.data.push_back(samplerate_circbuff[samplerate_circbuff_r]);
                samplerate_circbuff_r++;
                if(samplerate_circbuff_r >= samplerate_circbuff_size)
                  samplerate_circbuff_r = 0;
              }
              rosjack_out->publish(out);
            }
          }
        }break;
      }
    }
  }
  free(data_out);
}

rosjack_data ** input_from_rosjack (unsigned int data_length){
  int i;
  
  rosjack_data **data = (rosjack_data **)malloc(sizeof(rosjack_data *)*jack_num_inputs);
  for (i = 0; i < jack_num_inputs; i++){
    data[i] = (rosjack_data *)jack_port_get_buffer (jack_input_port[i], data_length);
  }
  
  return data;
}

void rosjack_roscallback(const jack_msgs::msg::JackAudio::SharedPtr msg){
  int msg_size = msg->size;
  
  if(ros_output_sample_rate == rosjack_sample_rate){
    jack_mtx.lock();
    for (int i = 0; i < msg_size; i++){
      ros2jack_buffer[ros2jack_buffer_size_w] = msg->data[i];
      
      ros2jack_buffer_size_w++;
      if(ros2jack_buffer_size_w >= ros2jack_buffer_size)
        ros2jack_buffer_size_w = 0;
    }
    jack_mtx.unlock();
  }else{
    int i;
    if(rosjack_type == ROSJACK_WRITE_STEREO){
      msg_size /= 2;
    }
    rosjack_data *data_out = (rosjack_data *)malloc(sizeof(rosjack_data)*msg_size);
    
    if(rosjack_type == ROSJACK_WRITE_STEREO){
      for (i = 0; i < msg_size; i++){
        data_out[i] = msg->data[(i*2)];
      }
    }else{
      for (i = 0; i < msg_size; i++){
        data_out[i] = msg->data[i];
      }
    }
    
    convert_to_sample_rate(data_out,msg_size);
    
    if (convert_to_sample_rate_ready(samplerate_data.output_frames_gen) && samplerate_data.output_frames_gen >= msg_size*samplerate_data.src_ratio){
      //printf("ROS -> msg_size: %d, samplerate_data.output_frames_gen: %ld\n", msg_size, samplerate_data.output_frames_gen); fflush(stdout);
      jack_mtx.lock();
      for (i = 0; i < samplerate_data.output_frames_gen; i++){
        ros2jack_buffer[ros2jack_buffer_size_w] = samplerate_circbuff[samplerate_circbuff_r];
        
        ros2jack_buffer_size_w++;
        if(ros2jack_buffer_size_w >= ros2jack_buffer_size)
          ros2jack_buffer_size_w = 0;
          
        samplerate_circbuff_r++;
        if(samplerate_circbuff_r >= samplerate_circbuff_size)
          samplerate_circbuff_r = 0;
      }
      jack_mtx.unlock();
    }
    free(data_out);
  }
  //printf("ROS -> samplerate_circbuff_w: %d, samplerate_circbuff_r: %d\n", samplerate_circbuff_w, samplerate_circbuff_r); fflush(stdout);
  //printf("ROS -> ros2jack_buffer_size_w: %d, ros2jack_buffer_size_r: %d\n", ros2jack_buffer_size_w, ros2jack_buffer_size_r); fflush(stdout);
}

rosjack_data * input_from_ros2jack_buffer (unsigned int data_length){
  rosjack_data *out = (rosjack_data *)malloc(sizeof(rosjack_data)*data_length);
  
  jack_mtx.lock();
  for (unsigned int i = 0; i < data_length; i++){
    out[i] = ros2jack_buffer[ros2jack_buffer_size_r];
    ros2jack_buffer[ros2jack_buffer_size_r] = 0.0;
    
    ros2jack_buffer_size_r++;
    if(ros2jack_buffer_size_r >= ros2jack_buffer_size)
      ros2jack_buffer_size_r = 0;
  }
  //printf("JAC -> ros2jack_buffer_size_w: %d, ros2jack_buffer_size_r: %d\n", ros2jack_buffer_size_w, ros2jack_buffer_size_r); fflush(stdout);
  jack_mtx.unlock();
  
  return out;
}
