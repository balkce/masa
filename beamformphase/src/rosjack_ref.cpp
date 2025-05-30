/**
 * Agent that outputs the first microphone of the array in the same delayed manner as the rest of the frequency-domain beamformers do when using the overlap-and-add logistics
 */

#include "rosjack/rosjack.hpp"
#include "util.h"

#include <map>
#include <vector>
#include <cmath>

// Include FFTW header
#include <complex>
#include <fftw3.h>

bool READY = false;
std::complex<double> * x;

void apply_weights (jack_ringbuffer_t *in, rosjack_data *out, int mic){
    unsigned int j;
    
    overlap_and_add_prepare_input(in, x);
    
    for (j = 0; j<fft_win; j++){
        out[j] = real(x[j]);
        
        //applying wola to avoid discontinuities in the time domain
        out[j] *= hann_win[j];
    }
}

int jack_callback (jack_nframes_t nframes, void *arg){
    //TimeVar t_bef = timeNow();
    jack_nframes_t i;
    
    //Inputing from ROS
    rosjack_data **out;  //needs to be dynamically allocated to be used by do_overlap_bymic
    out = (rosjack_data **) malloc (sizeof(rosjack_data*)*1);
    out[0] = (rosjack_data *) calloc (nframes,sizeof(rosjack_data));
    
    if(READY){
        rosjack_data **in = input_from_rosjack (nframes);
        do_overlap_bymic(in, out, nframes, apply_weights);
    }else{
        for (i = 0; i < nframes; i++){
            out[0][i] = 0.0;
        }
    }
    
    //Outputing to ROS
    output_to_rosjack (out[0], nframes, output_type);
    
    //std::cout << "Callback took: " << duration(timeNow()-t_bef)/1000000.0 << " ms.\n";
    return 0;
}

int main (int argc, char *argv[]) {
    /* ROS initialization*/
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> rosjack_node = rclcpp::Node::make_shared("rosjack_ref");
    handle_params(rosjack_node);
    
    number_of_microphones = 1;
    
    /* create JACK agent */
    if(rosjack_create (ROSJACK_READ, rosjack_node, "jackaudio_ref", rosjack_node->get_name(), number_of_microphones, jack_callback)){
        RCLCPP_ERROR(rosjack_node->get_logger(),"JACK agent could not be created.\n");
        rclcpp::shutdown();
        exit(1);
    }
    
    std::cout << "Pre-allocating space for internal buffers." << std::endl;
    prepare_overlap_and_add_bymic(); //fft_win is assigned here
    
    x = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * fft_win);
    
    READY = true;
    
    RCLCPP_INFO(rosjack_node->get_logger(),"Beamform node started.");

    /* keep running until stopped by the user */
    rclcpp::spin(rosjack_node);

    exit(0);
}
