/**
 * Beamform that carries out phase-based frequency masking by simple thresholding.
 */

#include "rosjack/rosjack.hpp"
#include "util.h"

// Include FFTW header
#include <complex>
#include <fftw3.h>

// Eigen include
#include <Eigen/Eigen>

bool READY = false;

std::complex<double> *x_fft, *x_time, *y_fft, *y_time;
std::complex<double> *y_fft_int;
fftw_plan x_forward, y_inverse;

double *freqs;
double *delays;
Eigen::MatrixXcd weights;

double mag_mult = 0.0001;
double mag_threshold = 0.0001;
double min_phase = 10;
double min_phase_diff_mean = 10*M_PI/180;

//reused buffers
double *phases_aligned;
Eigen::MatrixXcd in_fft;

void update_weights(bool ini=false){
    calculate_delays(delays);
    
    long unsigned int i;
    unsigned int j;
    
    for(i = 0; i < array_geometry.size(); i++){
        if (i == 0){
            if(ini){
                for(j = 0; j < fft_win; j++){
                    weights(i,j) = 1.0; 
                }
            }
        }else{
            for(j = 0; j < fft_win; j++){
                weights(i,j) = std::exp(-M_I*(double)2*PI*freqs[j]*delays[i]); 
            }
        }
    }
}

double get_overall_phase_diff(int min_i,int *num_i){
    if (min_i < number_of_microphones-1){
        double this_diff = 0;
        double this_diff_raw;
        for (int i = min_i+1; i < number_of_microphones; i++){
            this_diff_raw = abs(phases_aligned[min_i]-phases_aligned[i]);
            if (this_diff_raw > M_PI)
                this_diff_raw = 2*M_PI - this_diff_raw;
            this_diff += this_diff_raw;
            (*num_i)++;
        }
        return this_diff + get_overall_phase_diff(min_i+1,num_i);
    }else{
        return 0;
    }
}

void apply_weights (jack_ringbuffer_t **in, rosjack_data **out){
    int i;
    unsigned int j;
    double phase_diff_sum;
    int phase_diff_num;
    double phase_diff_mean;
    double mag_mean;
    double pha_mean;
    
    // fft
    for(i = 0; i < number_of_microphones; i++){
        overlap_and_add_prepare_input(in[i], x_time);
        fftw_execute(x_forward);
        for(j = 0; j < fft_win; j++){
            in_fft(i,j) = x_fft[j];
        }
    }
    
    y_fft[0] = in_fft(0,0);
    y_fft_int[0] = in_fft(0,0);
    
    for(j = 1; j < fft_win; j++){
        //creating new frequency data bin from mean magnitude
        mag_mean = 0;
        for(i = 0; i < number_of_microphones; i++){
            mag_mean += abs(in_fft(i,j));
        }
        mag_mean /= number_of_microphones;
        
        //and getting the phase of the reference microphone
        pha_mean = arg(in_fft(0,j));
        
        if (mag_mean/fft_win > mag_threshold){
          //applying weights to align phases
          for(i = 0; i < number_of_microphones; i++){
            phases_aligned[i] = arg(conj(weights(i,j))*in_fft(i,j));
          }
          
          //getting the mean phase difference between all microphones
          phase_diff_num = 0;
          phase_diff_sum = get_overall_phase_diff(0,&phase_diff_num);
          phase_diff_mean = phase_diff_sum/(double)phase_diff_num;
          
          pha_mean = arg(in_fft(0,j));
          
          if (phase_diff_mean < min_phase_diff_mean){
              y_fft[j] = std::complex<double>(mag_mean*cos(pha_mean),mag_mean*sin(pha_mean));
              y_fft_int[j] = std::complex<double>(0.0,0.0);
          }else{
              mag_mean *= mag_mult;
              y_fft[j] = std::complex<double>(mag_mean*cos(pha_mean),mag_mean*sin(pha_mean));
              y_fft_int[j] = in_fft(0,j);
          }
        }else{
          mag_mean *= mag_mult;
          y_fft[j] = std::complex<double>(mag_mean*cos(pha_mean),mag_mean*sin(pha_mean));
          y_fft_int[j] = in_fft(0,j);
        }
        
        //mag_mean *= 1 / (1+(15.0)*phase_diff_mean); // 15.0 here is a constant that is linked to the minimum phase difference
        //y_fft[j] = std::complex<double>(mag_mean*cos(pha_mean),mag_mean*sin(pha_mean));
    }
    
    // ifft
    fftw_execute(y_inverse);
    
    // preparing output for estimation
    overlap_and_add_prepare_output(y_time,out[0]);
    
    // preparing output for interference
    for(j = 0; j < fft_win; j++){
      y_fft[j] = y_fft_int[j];
    }
    fftw_execute(y_inverse);
    overlap_and_add_prepare_output(y_time,out[1]);
}

int jack_callback (jack_nframes_t nframes, void *arg){
    //TimeVar t_bef = timeNow();
    jack_nframes_t i;
    
    //Inputing from ROS
    rosjack_data **out = new rosjack_data*[2];
    out[0] = new rosjack_data[nframes];
    out[1] = new rosjack_data[nframes];
    if(READY){
        rosjack_data **in = input_from_rosjack (nframes);
        do_overlap_multi(in, out, nframes, apply_weights);
    }else{
        for (i = 0; i < nframes; i++){
            out[0][i] = 0.0;
            out[1][i] = 0.0;
        }
    }
    
    //Outputing to ROS in stereo
    rosjack_data *out_stereo = new rosjack_data[nframes*2];
    for (i = 0; i < nframes; i++){
      out_stereo[(i*2)] = out[0][i];
      out_stereo[(i*2)+1] = out[1][i];
    }
    output_to_rosjack (out_stereo, nframes*2);
    
    //std::cout << "Callback took: " << duration(timeNow()-t_bef)/1000000.0 << " ms.\n";
    return 0;
}

void theta_roscallback(const std_msgs::msg::Float32::SharedPtr msg){
    std::cout << "Updating weights for angle: " << msg->data << std::endl;
    
    angle = msg->data;
    update_weights();
}

void phase_handle_params(std::shared_ptr<rclcpp::Node> n){
    std::cout << "Phase ROS parameters: " << std::endl;
    
    n->declare_parameter("min_phase",10.0);
    n->declare_parameter("mag_mult",0.1);
    n->declare_parameter("mag_threshold",0.05);

    if (n->get_parameter("min_phase",min_phase)){
        RCLCPP_INFO(n->get_logger(),"Min Phase Threshold: %f",min_phase);
    }else{
        min_phase = 10.0;
        RCLCPP_WARN(n->get_logger(),"Min Phase Threshold argument not found in ROS param server, using default value (%f).",min_phase);
    }
    min_phase_diff_mean = min_phase*M_PI/180;
    
    if (n->get_parameter("mag_mult",mag_mult)){
        RCLCPP_INFO(n->get_logger(),"Mag Multiplier: %f",mag_mult);
    }else{
        mag_mult = 0.1;
        RCLCPP_WARN(n->get_logger(),"Mag Multiplier argument not found in ROS param server, using default value (%f).",mag_mult);
    }
    
    if (n->get_parameter("mag_threshold",mag_threshold)){
        RCLCPP_INFO(n->get_logger(),"Mag Threshold: %f",mag_threshold);
    }else{
        mag_threshold = 0.05;
        RCLCPP_WARN(n->get_logger(),"Mag Threshold argument not found in ROS param server, using default value (%f).",mag_threshold);
    }
    
}

int main (int argc, char *argv[]) {
    /* ROS initialization*/
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> rosjack_node = rclcpp::Node::make_shared("beamformphase");
    handle_params(rosjack_node);
    phase_handle_params(rosjack_node);
    
    const char *topic_name = "theta";
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr theta_subscriber = rosjack_node->create_subscription<std_msgs::msg::Float32>(topic_name, 1000,
      [rosjack_node, topic_name](const std_msgs::msg::Float32::SharedPtr msg) {
          theta_roscallback(msg);
        }
    );
    
    // if using the below alternative
    //   make sure to change the type of msg in theta_roscallback to just "std_msgs::msg::Float32"
    //   instead of "std_msgs::msg::Float32::SharedPtr"
    //rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr theta_subscriber = rosjack_node->create_subscription<std_msgs::msg::Float32>("theta", 1000, theta_roscallback);
    
    /* create JACK agent */
    if(rosjack_create (ROSJACK_READ_STEREO, rosjack_node, "jackaudiostereo", rosjack_node->get_name(), number_of_microphones, jack_callback)){
        RCLCPP_ERROR(rosjack_node->get_logger(),"JACK agent could not be created.\n");
        rclcpp::shutdown();
        exit(1);
    }
    
    std::cout << "Pre-allocating space for internal buffers." << std::endl;
    prepare_overlap_and_add_multi(2); //fft_win is assigned here
    
    x_fft = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * fft_win);
    x_time = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * fft_win);
    y_fft = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * fft_win);
    y_time = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * fft_win);
    y_fft_int = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * fft_win);
    
    x_forward = fftw_plan_dft_1d(fft_win, reinterpret_cast<fftw_complex*>(x_time), reinterpret_cast<fftw_complex*>(x_fft), FFTW_FORWARD, FFTW_MEASURE);
    y_inverse = fftw_plan_dft_1d(fft_win, reinterpret_cast<fftw_complex*>(y_fft), reinterpret_cast<fftw_complex*>(y_time), FFTW_BACKWARD, FFTW_MEASURE);
    
    freqs = (double *)malloc(sizeof(double)*fft_win);
    calculate_frequency_vector(freqs,fft_win);
    
    delays = (double *) malloc (sizeof(double)*number_of_microphones);
    phases_aligned = (double *) malloc (sizeof(double)*number_of_microphones);
    
    weights.resize(number_of_microphones,fft_win);
    in_fft.resize(number_of_microphones,fft_win);
    
    update_weights(true);
    
    READY = true;
    
    RCLCPP_INFO(rosjack_node->get_logger(),"Beamform ROS node started.");
    
    /* keep running until stopped by the user */
    rclcpp::spin(rosjack_node);
    
    std::cout << "Closing theta subscriber." << std::endl;
    theta_subscriber.reset();
    
    exit(0);
}
