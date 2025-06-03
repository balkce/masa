/*
	Source coded by Caleb Rascon, 2010
	IIMAS, UNAM
	México
	
	Example that uses libmultisoundloc.a
*/

#include "multisoundloc.h"

#include <iostream>
#include <signal.h>
#include <string.h>
#include <stdio.h>

//ROS libs
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/header.hpp"
#include "soundloc/msg/doa.hpp"
#include "soundloc/msg/frequencies.hpp"

//Glib, for threading (GThread)
#include <glib.h>

std::shared_ptr<rclcpp::Node> rosjack_node;
rclcpp::Publisher<soundloc::msg::DOA>::SharedPtr theta_topic;
int max_plot_confidence;

int freqselect_range;

static gpointer doa_stream(gpointer data){
	std::cout << "SoundLoc.DOAStream: Starting doa_stream thread.\n";fflush(stdout);
	
	while(true){
		if (sources.size() > 0){
			auto message = soundloc::msg::DOA();
			message.header.stamp = rosjack_node->get_clock()->now();
			message.size = sources.size();
			for (long unsigned int i = 0; i < sources.size(); ++i){
				message.doas.push_back(sources[i].doa);
				message.confs.push_back(sources[i].confidence);
				printf("SoundLoc: DOA[%ld] = %1.1f\n", i, sources[i].doa); fflush(stdout);
			}
			printf("SoundLoc: ---\n"); fflush(stdout);
			theta_topic->publish(message);
			soundloc_clear();
			//if (sources[0].confidence > max_plot_confidence){
			//}
		}
		
		millisleep(200);
	}
	
	return NULL;
}

bool ros_reset_soundloc(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"resetting soundloc");
	soundloc_clear();
	response->success = true;
	response->message = "";
	return true;
}

bool ros_get_sound_directions(const std::shared_ptr<std_srvs::srv::Trigger::Request> request, std::shared_ptr<std_srvs::srv::Trigger::Response> response){
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"providing current sound directions");
	
	response->message = "";
	
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"size: %d",(int)sources.size());
	
	if(sources.size() > 0){
		response->success = true;
		bool first_angle = true;
		for(long unsigned int i = 0; i <sources.size(); i++){
			if (first_angle){
				first_angle = false;
			}else{
				response->message += ",";
			}
			response->message += std::to_string(sources[i].doa);
		}
	}else{
		response->success = false;
	}
	
	return true;
}

void freqs_roscallback(const soundloc::msg::Frequencies::SharedPtr msg){
	std::cout << "SoundLoc: Updating frequencies for source localization... " << std::endl;
	
	doafrequencies_clear();
	for (int i = 0; i < msg->size; ++i){
		doafrequencies.push_back(msg->w[i]);
		std::cout << "\t "<< msg->w[i];
	}
	std::cout << std::endl;
	
	build_freqmask(freqselect_range);
	
	std::cout << "\t done. " << std::endl;
}

int main( int argc, char *argv[] )
{
	//ROS initialization
	std::cout << "SoundLoc: Starting up ROS connection...\n";
	
	rclcpp::init(argc, argv);
	rosjack_node = rclcpp::Node::make_shared("soundloc");
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_soundloc = rosjack_node->create_service<std_srvs::srv::Trigger>("reset_soundloc", &ros_reset_soundloc);
	rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_sound_directions = rosjack_node->create_service<std_srvs::srv::Trigger>("get_sound_directions", &ros_get_sound_directions);
	rclcpp::Subscription<soundloc::msg::Frequencies>::SharedPtr doafrequencies_subscriber = rosjack_node->create_subscription<soundloc::msg::Frequencies>("doafrequencies", 1000, freqs_roscallback);
	theta_topic = rosjack_node->create_publisher<soundloc::msg::DOA>("theta_est", 10);
	
	
	std::string node_name = rosjack_node->get_name();
	std::cout << "ROS parameters: " << node_name << std::endl;
	
	// Obtaining parameters from ROS parameter server
	float distance_between_mics;
	rosjack_node->declare_parameter("distance_between_mics",0.21);
	if (rosjack_node->get_parameter("distance_between_mics",distance_between_mics)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Distance between microphones: %f",distance_between_mics);
	}else{
		distance_between_mics = 0.21;
		RCLCPP_WARN(rosjack_node->get_logger(),"Distance between microphones argument not found in ROS param server, using default value (%f).",distance_between_mics);
	}
	
	int max_number_sources;
	rosjack_node->declare_parameter("max_number_sources",3);
	if (rosjack_node->get_parameter("max_number_sources",max_number_sources)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Maximum number of sources: %d",max_number_sources);
	}else{
		max_number_sources = 3;
		RCLCPP_WARN(rosjack_node->get_logger(),"Maximum number of sources argument not found in ROS param server, using default value (%d).",max_number_sources);
	}
	
	bool connect_ports;
	rosjack_node->declare_parameter("connect_ports",false);
	if (rosjack_node->get_parameter("connect_ports",connect_ports)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Automatic port connection: %d",connect_ports);
	}else{
		connect_ports = false;
		RCLCPP_WARN(rosjack_node->get_logger(),"Automatic port connection argument not found in ROS param server, using default value (%d).",connect_ports);
	}
	
	int gcc_style;
	rosjack_node->declare_parameter("gcc_style",4);
	if (rosjack_node->get_parameter("gcc_style",gcc_style)){
		RCLCPP_INFO(rosjack_node->get_logger(),"GCC style: %d",gcc_style);
	}else{
		gcc_style = 4;
		RCLCPP_WARN(rosjack_node->get_logger(),"GCC style argument not found in ROS param server, using default value (%d).",gcc_style);
	}
	
	double gcc_th;
	rosjack_node->declare_parameter("gcc_th",100.0);
	if (rosjack_node->get_parameter("gcc_th",gcc_th)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Correlation treshold: %f",gcc_th);
	}else{
		gcc_th = 100.0;
		RCLCPP_WARN(rosjack_node->get_logger(),"Correlation treshold argument not found in ROS param server, using default value (%f).",gcc_th);
	}
	
	double redundancy_th;
	rosjack_node->declare_parameter("redundancy_th",20.0);
	if (rosjack_node->get_parameter("redundancy_th",redundancy_th)){
		RCLCPP_INFO(rosjack_node->get_logger(),"DOA redundancy treshold: %f",redundancy_th);
	}else{
		redundancy_th = 20.0;
		RCLCPP_WARN(rosjack_node->get_logger(),"DOA redundancy treshold argument not found in ROS param server, using default value (%f).",redundancy_th);
	}
	
	int dynamic_gcc_th;
	rosjack_node->declare_parameter("dynamic_gcc_th",1);
	if (rosjack_node->get_parameter("dynamic_gcc_th",dynamic_gcc_th)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Dynamic GCC: %d",dynamic_gcc_th);
	}else{
		dynamic_gcc_th = 1;
		RCLCPP_WARN(rosjack_node->get_logger(),"Dynamic GCC argument not found in ROS param server, using default value (%d).",dynamic_gcc_th);
	}
	
	int moving_average;
	rosjack_node->declare_parameter("moving_average",1);
	if (rosjack_node->get_parameter("moving_average",moving_average)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Moving average: %d",moving_average);
	}else{
		moving_average = 1;
		RCLCPP_WARN(rosjack_node->get_logger(),"Moving average argument not found in ROS param server, using default value (%d).",moving_average);
	}
	
	int moving_factor;
	rosjack_node->declare_parameter("moving_factor",1);
	if (rosjack_node->get_parameter("moving_factor",moving_factor)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Moving factor: %d",moving_factor);
	}else{
		moving_factor = 1;
		RCLCPP_WARN(rosjack_node->get_logger(),"Moving factor argument not found in ROS param server, using default value (%d).",moving_factor);
	}
	
	int memory_factor;
	rosjack_node->declare_parameter("memory_factor",5);
	if (rosjack_node->get_parameter("memory_factor",memory_factor)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Memory factor: %d",memory_factor);
	}else{
		memory_factor = 5;
		RCLCPP_WARN(rosjack_node->get_logger(),"Memory factor argument not found in ROS param server, using default value (%d).",memory_factor);
	}
	
	int kmeans_min_dist;
	rosjack_node->declare_parameter("kmeans_min_dist",10);
	if (rosjack_node->get_parameter("kmeans_min_dist",kmeans_min_dist)){
		RCLCPP_INFO(rosjack_node->get_logger(),"kmean minimum distance: %d",kmeans_min_dist);
	}else{
		kmeans_min_dist = 10;
		RCLCPP_WARN(rosjack_node->get_logger(),"kmean minimum distance argument not found in ROS param server, using default value (%d).",kmeans_min_dist);
	}
	
	rosjack_node->declare_parameter("max_plot_confidence",4);
	if (rosjack_node->get_parameter("max_plot_confidence",max_plot_confidence)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Cluster size for maximum confidence: %d",max_plot_confidence);
	}else{
		max_plot_confidence = 4;
		RCLCPP_WARN(rosjack_node->get_logger(),"Cluster size for maximum confidence argument not found in ROS param server, using default value (%d).",max_plot_confidence);
	}
	
	double noise_threshold;
	rosjack_node->declare_parameter("noise_threshold",0.001);
	if (rosjack_node->get_parameter("noise_threshold",noise_threshold)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Noise treshold: %f",noise_threshold);
	}else{
		noise_threshold = 0.001;
		RCLCPP_WARN(rosjack_node->get_logger(),"Noise treshold argument not found in ROS param server, using default value (%f).",noise_threshold);
	}
	
	double noise_peak_change;
	rosjack_node->declare_parameter("noise_peak_change",0.0015);
	if (rosjack_node->get_parameter("noise_peak_change",noise_peak_change)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Noise peak change: %f",noise_peak_change);
	}else{
		noise_peak_change = 0.0015;
		RCLCPP_WARN(rosjack_node->get_logger(),"Noise peak change argument not found in ROS param server, using default value (%f).",noise_peak_change);
	}
	
	rosjack_node->declare_parameter("freqselect_range",10);
	if (rosjack_node->get_parameter("freqselect_range",freqselect_range)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Frequency select range: %d",freqselect_range);
	}else{
		freqselect_range = 10;
		RCLCPP_WARN(rosjack_node->get_logger(),"Frequency select range argument not found in ROS param server, using default value (%d).",freqselect_range);
	}
	
	bool verbose;
	rosjack_node->declare_parameter("verbose",false);
	if (rosjack_node->get_parameter("verbose",verbose)){
		RCLCPP_INFO(rosjack_node->get_logger(),"Verbose: %d",verbose);
	}else{
		verbose = false;
		RCLCPP_WARN(rosjack_node->get_logger(),"Verbose argument not found in ROS param server, using default value (%d).",verbose);
	}
	
	std::cout << "\nSoundLoc: Stand by while everything is initialized.\n\n";
	
	soundloc_init(distance_between_mics, max_number_sources, connect_ports, gcc_style, gcc_th, redundancy_th, dynamic_gcc_th, moving_average, moving_factor, memory_factor, kmeans_min_dist, noise_threshold, noise_peak_change, verbose);
	
	GThread *doastream = NULL;
	doastream = g_thread_new("doa_stream", (GThreadFunc) doa_stream, NULL);
	
	RCLCPP_INFO(rosjack_node->get_logger(),"SoundLoc node started.");
	
	/* keep running until stopped by the user */
	rclcpp::spin(rosjack_node);
	exit(0);
}
