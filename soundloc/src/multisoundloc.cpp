/*
 * Multiple Sound Source Direction of Arrival Estimation with Feedback
 *
 * Author:
 *      Caleb Rascon, caleb.rascon@gmail.com
 * Based on work from:
 *      Luis Miguel Gato Diaz, lmiguelgato@gmail.com
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <math.h>
#include <random>
#include <iostream>
#include <fstream>
#include <iomanip>

// JACK
#include <jack/jack.h>

// FFTW
#include <complex.h> 	// needs to be included before fftw3.h for compatibility
#include <cmath>
#include <fftw3.h>

// Include everything else we coded
#include "multisoundloc.h"

int GCC_STYLE = 4;
double GCC_TH = 100.0;
double REDUNDANCY_TH = 20.0;
int DYNAMIC_GCC_TH = 1;
int MOVING_AVERAGE = 1;
int MOVING_FACTOR = 1;
int MEMORY_FACTOR = 5;
bool VERBOSE = false;
int kmeans_min_dist = 10;

// Jack:
jack_port_t **input_ports;
jack_client_t *client;

// FFTW:
std::complex<double> *i_fft_2N, *i_time_2N, *o_fft_2N, *o_time_2N;
fftw_plan i_forward_2N, o_inverse_2N;
std::complex<double> ig(0.0, 1.0); 		// imaginary unit

// default parameters:
double sample_rate  = 48000.0;			// default sample rate [Hz]
int nframes 		= 1024;				// default number of frames per jack buffer
unsigned int window_size, window_size_2, nframes_2;	
double mic_separation = 0.18;			// default microphone separation [meters]
double c = 343.364;						// default sound speed [m/s]
double dt_max, N_max;					// maximum delay between microphones [s, samples]
double doa;								// direction of arrival
double mean_doa = 0.0;					// average direction of arrival
double std_doa = 90.0;					// standard deviation of the direction of arrival
double std_cum = 90.0;					// standard deviation of the direction of arrival (cummulative)
unsigned int n_sources = 1; 			// default number of sources to be detected
double gcc_th = GCC_TH;					// default GCC threshold

double f_min = 1000.0;					// minimum frequency of the desired signal [Hz]
double f_max = 4000.0;					// maximum frequency of the desired signal [Hz]
int kmin, kmax;							// discrete minimum and maximum frequencies of the desired signal
double f0;

unsigned int n_in_channels = 3;			// number of input channels

int state = 0;							// beamformer state

jack_default_audio_sample_t *hann;		// array to store the Hann window

// GCC registers:
std::complex<double> **X_gcc;			// store GCC result (length 2 times 'nframes')
std::complex<double> *Aux_gcc;			// store axuliary GCC result (length 2 times 'nframes')

// DOA registers:
double *DOA_hist;						// store DOA history
unsigned int *DOA_class;				// store classification of DOA history
double *DOA_kmean;						// store DOA mean for each source (kmeans algorithm)
double *DOA_mean;						// store DOA mean for each source
double *DOA_stdev;						// store DOA standard deviation for each source
double *DOA_valid;						// store valid DOAs
unsigned int *counter;					// store number of detections for each source
int    *dcounter;						// number of valid DOAs
int    *ecounter;						// number of smoothed valid DOAs
int    icounter = 0;					// number of detections
int    ccounter = 0;					// number of cycles
int    hist_length;						// number of cycles
bool   firstDetection = true;

double **kalmanState;
double **covMatrix;

// VAD:
double noise_threshold = 0.001;
double noise_peak_change = 0.0015;

// Feedback:



// Misc:
bool FINISHED_CAPTURING_NOISE = false;
unsigned int noise_j = 0;
unsigned int number_of_samples_of_noise = 10;
double mag_main[10];
double mag_left = 0, mag_right = 0, mag_front = 0;
double mag_avg = 0;
double noise_mag = 0;
double this_mag = 0;
int frames_not_passed = 0;
bool part_of_source = 0;
bool in_silence = 1;

std::vector < source > sources;
std::vector < double > doafrequencies;

// frequency mask based on source class
double* freqmask;
double* freqs;

//GLib Thread:
GMutex mutex_sources;
GMutex mutex_doafrequencies;

//Sleep function in milliseconds
void millisleep(int milli){
	struct timespec st = {0,0};
	st.tv_sec = milli/1000;
	st.tv_nsec = (milli%1000)*1000000L;
	nanosleep(&st, NULL);
}

void soundloc_clear(){
	g_mutex_lock (&mutex_sources);
	sources.clear();
	g_mutex_unlock (&mutex_sources);
}

void doafrequencies_clear(){
	g_mutex_lock (&mutex_doafrequencies);
	doafrequencies.clear();
	g_mutex_unlock (&mutex_doafrequencies);
}

double state2angle (double* state) {
	
	/*
		Convert a state (Cartesian coordinates) to the corresponding angle
		
		input: 	state (Cartesian coordinates)
		output: (the corresponding angle)
	*/
	
	if (abs(state[1]) <= 0.001) {
		if (state[0] >= 0.0)
			return 90.0;
		else
			return -90.0;
	} else {
		if (state[1] >= 0) {
			return atan(state[0]/state[1])*RAD2DEG;
		} else {	
			if (state[0] > 0) {
				return atan(state[0]/state[1])*RAD2DEG + 180.0;
			} else {
				if (state[0] < 0)
					return atan(state[0]/state[1])*RAD2DEG - 180.0;
				else
					return 180.0;
			}
		}
	}
	
}

void angle2state (double angle, double* state) {
	
	/*
		Convert an angle to its corresponding state (Cartesian coordinates)
		
		input: 	an angle (in radians)
		output: state (the corresponding Cartesian coordinates)
	*/
	
	state[0] = sin(angle*DEG2RAD);
	state[1] = cos(angle*DEG2RAD);
	
}

double angleRedundancy (double* theta, double* thetaRedundant, double Ethresh) {
	int i, j, k;
	double theta12, theta23, theta31;
	double Epqr, minAngleDiff;
	double minEpqr = 100000;
	double doa;
	double xy_1[2];
	double xy_2[2];
	double xy_diff[2];
	bool found = false;
	
	for (i = 0; i < 2; ++i)	{
		theta12 = theta[i*3];
		for (j = 0; j < 2; ++j)	{
			theta23 = theta[1+j*3];
			for (k = 0; k < 2; ++k)	{
				theta31 = theta[2+k*3];
				
				angle2state (theta12, xy_1);
				angle2state (theta23, xy_2);
				
				xy_diff[0] = xy_1[0]-xy_2[0];
				xy_diff[1] = xy_1[1]-xy_2[1];
				
				Epqr = xy_diff[0]*xy_diff[0] + xy_diff[1]*xy_diff[1];
				if (Epqr < minEpqr)
				{
					minEpqr = Epqr;
					minAngleDiff = abs(theta12-theta23);
					
					if (minAngleDiff <= Ethresh) {
						found = true;
						doa = (theta12+theta23)/2;
					}else if (minAngleDiff >= (360.0-Ethresh)) {
						found = true;
						doa = (abs(theta12)+abs(theta23))/2;
					}
					
				}
				
				angle2state (theta31, xy_1);
				
				xy_diff[0] = xy_1[0]-xy_2[0];
				xy_diff[1] = xy_1[1]-xy_2[1];
				
				Epqr = xy_diff[0]*xy_diff[0] + xy_diff[1]*xy_diff[1];
				if (Epqr < minEpqr)
				{
					minEpqr = Epqr;
					minAngleDiff = abs(theta31-theta23);
					if (minAngleDiff <= Ethresh) {
						found = true;
						doa = (theta31+theta23)/2;
					}else if (minAngleDiff >= (360.0-Ethresh)) {
						found = true;
						doa = (abs(theta31)+abs(theta23))/2;
					}
				}
				
				angle2state (theta12, xy_2);
				
				xy_diff[0] = xy_1[0]-xy_2[0];
				xy_diff[1] = xy_1[1]-xy_2[1];
				
				Epqr = xy_diff[0]*xy_diff[0] + xy_diff[1]*xy_diff[1];
				if (Epqr < minEpqr)
				{
					minEpqr = Epqr;
					minAngleDiff = abs(theta31-theta12);
					if (minAngleDiff <= Ethresh) {
						found = true;
						doa = (theta12+theta31)/2;
					}else if (minAngleDiff >= (360.0-Ethresh)) {
						found = true;
						doa = (abs(theta12)+abs(theta31))/2;
					}
				}
			}
		}
	}
	
	if (!found) 	doa = 181.0;
	
	return doa;

}

void angleTranslation (double *theta) {
	
	int i;
	
	for (i = 0; i < 3; ++i) {
		if (theta[i] >= 0.0) {
			theta[i+3] = 180.0 - theta[i];
		} else {
			theta[i+3] = -180.0 - theta[i];
		}
	}
	
	theta[1] -= 120.0;
	theta[2] += 120.0;
	theta[4] -= 120.0;
	theta[5] += 120.0;
	
	for (i = 0; i < 6; ++i) {
		if (theta[i] > 180.0) {
			theta[i] -= 360.0;
		}
		if (theta[i] < -180.0) {
			theta[i] += 360.0;
		}
	}
	
	return;
}

void kalman (double y_array[2], double *x_array, double (*P_array)[4]) {
	
	/*
		Kalman filter
		
		input:	x   (previous state)
				y   (current measurement)
				P   (previous-state error posterior covariance matrix)
	*/
	
	// initialization:
	Eigen::MatrixXd y(2,1);
	Eigen::MatrixXd x(4,1);
	Eigen::MatrixXd P(4,4);
	
	y(0,0) = y_array[0];	y(1,0) = y_array[1];
	
	for (int i = 0; i < 4; ++i)
	{
		x(i,0) = x_array[i];
		
		for (int j = 0; j < 4; ++j)
		{
			P(i,j) = P_array[i][j];
		}
	}
	
	Eigen::MatrixXd P_(4,4);		// previous-state error prior covariance matrix
	Eigen::MatrixXd K(4,2);		// Kalman gain
	Eigen::MatrixXd temp2x2(2,2);	// temporary 2x2 matrix
	Eigen::MatrixXd temp4x1(4,1);	// temporary 4x1 matrix
	
	// prediction:
	temp4x1	= F*x;
	P_ = F*P*F.transpose() + Q;
	
	// update:
	temp2x2 = H*P_*H.transpose() + R;
	K = (P_ * H.transpose()) * temp2x2.inverse();
	
	x = temp4x1 + K*(y - H*temp4x1);
	P = (I4x4 - K*H)*P_;
	
	for (int i = 0; i < 4; ++i)
	{
		x_array[i] = x(i,0);
		
		for (int j = 0; j < 4; ++j)
		{
			P_array[i][j] = P(i,j);
		}
	}
}

void kmeans (double* DOA_hist, unsigned int *DOA_class, double* DOA_mean, unsigned int* class_count, unsigned int n_sources, unsigned int hist_length) {
	unsigned int i, j;
	double min_dist, dist;
	
	for (i = 0; i < n_sources; ++i)
	{
		class_count[i] = 0;
	}
	
	for (j = 0; j < hist_length; ++j)
	{
		min_dist = kmeans_min_dist; 	// a number grater than 4
		for (i = 0; i < n_sources; ++i)
		{
			dist = pow(cos(DOA_hist[j]*DEG2RAD) - cos(DOA_mean[i]*DEG2RAD), 2) + pow(sin(DOA_hist[j]*DEG2RAD) - sin(DOA_mean[i]*DEG2RAD), 2);
			if (dist < min_dist) {
				min_dist = dist;
				DOA_class[j] = i;
			}
		}
		++class_count[ DOA_class[j] ];
	}
	
	for (i = 0; i < n_sources; ++i)
	{
		for (j = 0; j < hist_length; ++j)
		{
			if 	(DOA_class[j] == i) {
				DOA_mean[i] += DOA_hist[j];
			}
		}
		if (class_count[i] != 0)
			DOA_mean[i] /= (class_count[i]+1);
	}
}

int max (std::complex<double> *signal, int signal_length, int Nmax, double *max_value) {
	int max_index = -1;
	double tmp2 = -1.0;
	
	max_value[0] = -1.0;
	for (int i = 0; i < Nmax; ++i) {
		tmp2 = real(signal[i]);
		if(tmp2 > *max_value) {
			max_index = i;
			*max_value = tmp2;
		}
	}
	for (int i = signal_length-Nmax; i < signal_length; ++i) {
		tmp2 = real(signal[i]);
		if(tmp2 > *max_value) {
			max_index = i;
			*max_value = tmp2;
		}
	}
	
	return max_index;
}

double mean (double* array, int length) {
	double thisMean = 0;
	double dlength = (double) length;
	
	for (int i = 0; i < length; ++i) {
		thisMean += array[i];
	}
	
	return thisMean/dlength;
}

void phat (std::complex<double>* Z, std::complex<double>* X, std::complex<double>* Y, int X_length, int kmin, int kmax, int type) {
	
	int i;
	std::complex<double> temp;
	
	switch ( type ) {
		case 2 :	// GCC (frequency restrained)
			for (i = 0; i < kmin; ++i)							Z[i] = 0.0;
			for (i = kmax; i < X_length-kmax; ++i)				Z[i] = 0.0;
			for (i = X_length-kmin; i < X_length; ++i)			Z[i] = 0.0;
			for (i = kmin; i < kmax; ++i)						Z[i] = X[i] * conj(Y[i]);
			for (i = X_length-kmax; i < X_length-kmin; ++i)		Z[i] = X[i] * conj(Y[i]);
			break;
		case 3 :	// GCC-PHAT
			for (i = 0; i < X_length; ++i) {
				temp = X[i] * conj(Y[i]);
				(abs(temp) == 0.0) ? (Z[i] = 0.0) : (Z[i] = temp/abs(temp));
			}
			break;
		case 4 :	// GCC-PHAT (frequency restrained)
			for (i = 0; i < kmin; ++i)							Z[i] = 0.0;
			for (i = kmax; i < X_length-kmax; ++i)				Z[i] = 0.0;
			for (i = X_length-kmin; i < X_length; ++i) 			Z[i] = 0.0;
			for (i = kmin; i < kmax; ++i) {
				temp = X[i] * conj(Y[i]);
				(abs(temp) == 0.0) ? (Z[i] = 0.0) : (Z[i] = temp/abs(temp));
			}
			for (i = X_length-kmax; i < X_length-kmin; ++i)	{
				temp = X[i] * conj(Y[i]);
				(abs(temp) == 0.0) ? (Z[i] = 0.0) : (Z[i] = temp/abs(temp));
			}
			break;
		default : 	// GCC
			for (i = 0; i < X_length; ++i)	Z[i] = X[i] * conj(Y[i]);
			break;
	}
}

double unwrap (int index, int N, double max_index) {
	double temp;
	
	if (index < N)
		temp = -((double) index);
	else
		temp = ((double) 2*N - index);
	
	if (temp > max_index)
		return max_index;
	if (temp < -max_index)
		return -max_index;
	
	return temp;
}

void build_freqmask(int freqselect_range){
	unsigned int i,j;
	g_mutex_lock (&mutex_doafrequencies);
	
	double range_in_hertz = f0 * (double)freqselect_range;
	
	for(i = 0; i < window_size_2; ++i) {
		freqmask[i] = 1.0;
	}
	
	for(j = 0; j < doafrequencies.size(); ++j) {
		for(i = 0; i < window_size_2; ++i) {
			if(freqs[i] >= doafrequencies[j] - range_in_hertz && freqs[i] <= doafrequencies[j] + range_in_hertz){
				freqmask[i] = 0.0;
			}
		}
	}
	
	g_mutex_unlock (&mutex_doafrequencies);
}

int jack_callback (jack_nframes_t nframes, void *arg){
	
	unsigned int i, j;
	
	jack_default_audio_sample_t **in;
	
	in = (jack_default_audio_sample_t **)malloc(n_in_channels*sizeof(jack_default_audio_sample_t *));
	for(i = 0; i < n_in_channels; ++i)
		in[i] = (jack_default_audio_sample_t *)jack_port_get_buffer (input_ports[i], nframes);
	
	mag_left = 0;
	mag_right = 0;
	mag_front = 0;
	
	for (i = 0; i<nframes; i++){
		mag_left += fabs(in[0][i]);
		mag_right += fabs(in[1][i]);
		mag_front += fabs(in[2][i]);
	}
	
	if (noise_j >= number_of_samples_of_noise){
		//mag_main[noise_j]= ((mag_left+mag_right+mag_front)/number_of_samples)/n_in_channels;
		//noise_j++;
		
		mag_avg = 0;
		for(j=0; j<number_of_samples_of_noise; j++)
			mag_avg += mag_main[j];
		mag_avg /= number_of_samples_of_noise;
		noise_mag = mag_avg;
	}else{
		mag_main[noise_j]= ((mag_left+mag_right+mag_front)/nframes)/(n_in_channels);
		noise_j++;
		return 0;
	}
	
	this_mag = ((mag_left+mag_right+mag_front)/nframes)/(n_in_channels);
	if(VERBOSE)
		std::cout << "TestVAD.Capture: (" << in_silence << ", "<< noise_j << ") this_mag -> " << this_mag << " (noise_mag: " << noise_mag << ")" << " <> " << noise_mag+noise_threshold << "\n";
	
	if (!in_silence && this_mag > noise_mag+noise_threshold){
		
		for (j = 0; j < n_in_channels; ++j) {
			// cross-correlation in four steps:
			// 1- zero padding:
			for (i = 0; i < nframes; ++i)
				i_time_2N[i] = in[j][i];
			
			for (i = nframes; i < window_size_2; ++i)
				i_time_2N[i] = 0.0;
			
			// 2- apply FFT:
			fftw_execute(i_forward_2N);
			
			for (i = 0; i < window_size_2; ++i)
				X_gcc[j][i] = i_fft_2N[i];
		}
		
		//applying frequency mask
		g_mutex_lock (&mutex_doafrequencies);
		for (j = 0; j < n_in_channels; ++j) {
			for (i = 0; i < window_size_2; ++i){
				X_gcc[j][i] *= freqmask[i];
			}
		}
		g_mutex_unlock (&mutex_doafrequencies);
		
		// 3- multiply pairs of FFTs (time reversing one of them), and
		// 4- apply iFFT
		phat (o_fft_2N, X_gcc[0], X_gcc[1], window_size_2, kmin, kmax, GCC_STYLE);
		fftw_execute(o_inverse_2N);
		
		for (i = 0; i < window_size_2; ++i)
			Aux_gcc[i] = o_time_2N[i];
		
		phat (o_fft_2N, X_gcc[1], X_gcc[2], window_size_2, kmin, kmax, GCC_STYLE);
		fftw_execute(o_inverse_2N);
		
		for (i = 0; i < window_size_2; ++i) {
			X_gcc[1][i] = Aux_gcc[i];
			Aux_gcc[i] = o_time_2N[i];
		}
		
		phat (o_fft_2N, X_gcc[2], X_gcc[0], window_size_2, kmin, kmax, GCC_STYLE);
		fftw_execute(o_inverse_2N);
		
		for (i = 0; i < window_size_2; ++i) {
			X_gcc[2][i] = Aux_gcc[i];
			X_gcc[0][i] = o_time_2N[i];
		}
		
		double max_val12, max_val23, max_val31;
		
		// find maximum of the cross-correlations, and estimate DOA:
		double theta[6] = {asin(  unwrap(  max(X_gcc[1], window_size_2, N_max, &max_val12), nframes, N_max  )/sample_rate*c/mic_separation  )*RAD2DEG,
						   asin(  unwrap(  max(X_gcc[2], window_size_2, N_max, &max_val23), nframes, N_max  )/sample_rate*c/mic_separation  )*RAD2DEG,
						   asin(  unwrap(  max(X_gcc[0], window_size_2, N_max, &max_val31), nframes, N_max  )/sample_rate*c/mic_separation  )*RAD2DEG,
						   0.0,
						   0.0,
						   0.0};
						
		angleTranslation(theta);	// use a coherent reference to measure DOA
		
		double thetaRedundant[3] = {0.0, 0.0, 0.0};
		
		doa = angleRedundancy (theta, thetaRedundant, REDUNDANCY_TH);
		
		double max_mean, max_max;
		
		bool passed_gcc_threshold = false;
		
		switch (DYNAMIC_GCC_TH) { //enable a dynamic GCC threshold
			case 1:	// mean peak values
				max_mean = (max_val12 + max_val23 + max_val31)/3;
				
				if (max_mean > GCC_TH) {
					++ccounter;
					//gcc_th = (gcc_th*ccounter + max_mean)/(ccounter+1);
					gcc_th = max_mean;
				}
				
				if (doa != 181.0 && max_val12 > 0.9*gcc_th && max_val23 > 0.9*gcc_th && max_val31 > 0.9*gcc_th) {
					passed_gcc_threshold = true;
				}
			break;
			
			case 2:	// max peak values
				max_max = 0.0;
				
				if (max_val12 > max_max)
					max_max = max_val12;
				if (max_val23 > max_max)
					max_max = max_val23;
				if (max_val31 > max_max)
					max_max = max_val31;
				
				if (max_max > 1.0) {
					++ccounter;
					gcc_th = (gcc_th*(ccounter-1) + max_max)/ccounter;
				}
				
				if (doa != 181.0 && max_val12 > 0.9*gcc_th && max_val23 > 0.9*gcc_th && max_val31 > 0.9*gcc_th) {
					passed_gcc_threshold = true;
				}
			break;
			
			default:
				max_mean = (max_val12 + max_val23 + max_val31)/3;
				if (doa != 181.0 && max_mean > GCC_TH) {
					passed_gcc_threshold = true;
				}
				break;
		}
		
		if (VERBOSE)
		{
			printf("SoundLoc: theta1 = [%1.5f, %1.5f];\ttheta2 = [%1.5f, %1.5f];\ttheta3 = [%1.5f, %1.5f]\n", theta[0], theta[3], theta[1], theta[4], theta[2], theta[5]);
			printf("          val1 = %1.5f;\tval2 = %1.5f;\tval3 = %1.5f\n", max_val12, max_val23, max_val31);
			printf("          max_mean = %1.5f\n", max_mean);
			printf("          gcc_th = %1.5f\n", gcc_th);
			printf("          thetaR = [%1.5f, %1.5f, %1.5f]\n", thetaRedundant[0], thetaRedundant[1], thetaRedundant[2]);
			printf("          doa = %1.5f\n", doa);
			fflush(stdout);
		}
		
		if (passed_gcc_threshold) { 	// are these DOAs valid?
			DOA_hist[icounter%hist_length] = doa; 	// save into shift register
			++icounter;
			
			if (icounter >= hist_length) {	// is the shift register full?
				
				if (icounter == hist_length && firstDetection) {
					firstDetection = false;
					
					DOA_kmean[0] = mean(DOA_hist, hist_length);
					
					for (i = 1; i < n_sources; ++i)
					{
						DOA_kmean[i] = doa + 360.0/n_sources*i;
						
						if (DOA_kmean[i] > 180.0) {
							DOA_kmean[i] -= 360.0;
						}
					}
					
					// initialize Kalman state:
					double initialThisState[2];
					for (i = 0; i < n_sources; ++i) {
						angle2state(DOA_kmean[i], initialThisState);
						
						kalmanState[0][i] = initialThisState[0];
						kalmanState[1][i] = initialThisState[1];
						
						printf("SoundLoc: kalman initialization[%d]: %1.1f\n", i, DOA_kmean[i]);
					}
				}
				
				// group similar DOAs into clusters using the k-means algorithm:
				kmeans (DOA_hist, DOA_class, DOA_kmean, counter, n_sources, hist_length);
				
				double measurement[2];
				double state[4];
				double cov[4][4];
				
				for (i = 0; i < n_sources; ++i)
				{
					//if (counter[i] > 0) {	// any DOA in this cluster?
					
						if (DOA_class[icounter%hist_length] == i) {
							angle2state (DOA_hist[icounter%hist_length], measurement);
							
							state[0] = kalmanState[0][i];	state[1] = kalmanState[1][i];	state[2] = kalmanState[2][i];	state[3] = kalmanState[3][i];
							
							for (j = 0; j < 4; ++j) {
								cov[j][0] = covMatrix[4*i+j][0];	cov[j][1] = covMatrix[4*i+j][1];	cov[j][2] = covMatrix[4*i+j][2];	cov[j][3] = covMatrix[4*i+j][3];
							}
							
							kalman (measurement, state, cov);
							
							//outputKalman << DOA_class[icounter%hist_length] << ' ';
							//outputKalman << setprecision(2) << DOA_hist[icounter%hist_length];
							//outputKalman << ' ';
							//outputKalman << setprecision(2) << state2angle (state);
							//outputKalman << endl;
							
							kalmanState[0][i] = state[0];	kalmanState[1][i] = state[1];	kalmanState[2][i] = state[2];	kalmanState[3][i] = state[3];
							for (j = 0; j < 4; ++j) {
								covMatrix[4*i+j][0] = cov[j][0];	covMatrix[4*i+j][1] = cov[j][1];	covMatrix[4*i+j][2] = cov[j][2];	covMatrix[4*i+j][3] = cov[j][3];
							}
						}
					//}
				}
				
				g_mutex_lock (&mutex_sources);
				sources.clear();
				for (i = 0; i < n_sources; ++i)
				{
					if (counter[i] > 0) {	// any DOA in this cluster?
						++dcounter[i];
						
						if (MOVING_AVERAGE == 2) { //enable moving average
							DOA_mean[i] = (DOA_mean[i]*(dcounter[i]-1) + DOA_kmean[i])/dcounter[i];		// moving average
							DOA_stdev[i] += pow(DOA_kmean[i]-DOA_mean[i], 2);							// standard deviation
						} else {
							DOA_mean[i] = (DOA_mean[i] + DOA_kmean[i])/2.0;								// moving average
							DOA_stdev[i] += pow(DOA_kmean[i]-DOA_mean[i], 2);							// standard deviation
						}
						
						if (abs(DOA_kmean[i]-DOA_mean[i]) < MOVING_FACTOR*sqrt(DOA_stdev[i]/dcounter[i])) {		// avoid outsiders
							++ecounter[i];
							if (MOVING_AVERAGE == 0) {
								DOA_valid[i] = DOA_kmean[i];
							} else {
								DOA_valid[i] = DOA_mean[i];
							}
							
							//printf("SoundLoc: DOA[%d] = %1.1f\n", i, DOA_valid[i]); fflush(stdout);
							source this_source;
							this_source.doa = DOA_valid[i];
							this_source.doa_time = time(NULL);
							this_source.confidence = counter[i];
							sources.push_back(this_source);
						}
						
					}
				}
				//printf("SoundLoc: ---\n"); fflush(stdout);
				g_mutex_unlock (&mutex_sources);
			}else{
				printf("found something, but shift register not full : %d (%d) ... need more data\n",icounter,hist_length);
			}
		}else{
			g_mutex_lock (&mutex_sources);
			sources.clear();
			g_mutex_unlock (&mutex_sources);
		}
	}else{
		frames_not_passed++;
	}
	
	//checking for change in activity
	mag_avg = 0;
	for(j=0; j<number_of_samples_of_noise; j++)
		mag_avg += mag_main[j];
	mag_avg /= number_of_samples_of_noise;
	
	if(in_silence && this_mag > mag_avg+noise_peak_change){
		in_silence = 0;
		noise_mag = mag_avg;
		
		//resetting magnitude frames
		for(j=0; j<number_of_samples_of_noise; j++){
			mag_main[j] = mag_avg;
		}
	}else if(!in_silence && (this_mag < mag_avg-noise_peak_change || frames_not_passed > 5)){
		frames_not_passed = 0;
		in_silence = 1;
		
		//resetting magnitude frames
		for(j=0; j<number_of_samples_of_noise; j++){
			mag_main[j] = noise_mag;
		}
		
	}else{
		//storing this magnitude for next frames
		for(j=1; j<number_of_samples_of_noise; j++){
			mag_main[j-1] = mag_main[j];
		}
		mag_main[number_of_samples_of_noise-1] = this_mag;
	}
	
	return 0;
}

void jack_shutdown (void *arg){
	exit (1);
}

void soundloc_init(double distance_between_mics_in, int max_number_sources, int connect_ports, int gcc_style, double gcc_th_in, double redundancy_th, int dynamic_gcc_th, int moving_average, int moving_factor, int memory_factor, int kmeans_min_dist_in, double noise_threshold_in, double noise_peak_change_in, int verbose){
	GCC_STYLE = gcc_style;
	GCC_TH = gcc_th_in;
	gcc_th = GCC_TH;
	REDUNDANCY_TH = redundancy_th;
	DYNAMIC_GCC_TH = dynamic_gcc_th;
	MOVING_AVERAGE = moving_average;
	MOVING_FACTOR = moving_factor;
	MEMORY_FACTOR = memory_factor;
	VERBOSE = verbose;
	mic_separation = distance_between_mics_in;
	n_sources = max_number_sources;
	kmeans_min_dist = kmeans_min_dist_in;
	noise_threshold = noise_threshold_in;
	noise_peak_change = noise_peak_change_in;
	
	unsigned int i, j;
	
	const char *client_name = "soundloc";
	jack_options_t options = JackNoStartServer;
	jack_status_t status;
	
	client = jack_client_open (client_name, options, &status);
	if (client == NULL){
		printf ("SoundLoc: jack_client_open() failed, status = 0x%2.0x\n", status);
		if (status & JackServerFailed) {
			printf ("SoundLoc: Unable to connect to JACK server.\n");
		}
		exit (1);
	}
	
	if (status & JackNameNotUnique){
		client_name = jack_get_client_name(client);
		printf ("SoundLoc: Warning, other agent with our name is running, `%s' has been assigned to us.\n", client_name);
	}
	
	jack_set_process_callback (client, jack_callback, 0);
	jack_on_shutdown (client, jack_shutdown, 0);
	
	// obtain here the delay from user and store it in 'delay' 
	nframes 	= (int) jack_get_buffer_size (client);
	sample_rate = (double) jack_get_sample_rate(client);
	
	nframes_2   = nframes/2;
	window_size = 4*nframes;
	window_size_2 = 2*nframes;
	kmin = (int) (f_min/sample_rate*window_size_2);
	kmax = (int) (f_max/sample_rate*window_size_2);
	dt_max = mic_separation/c;
	N_max = dt_max*sample_rate;
	
	hist_length = MEMORY_FACTOR*n_sources;
	
	// initialization of internal buffers
	// - overlap-add buffers
	X_gcc		= (std::complex<double> **) calloc(n_in_channels, sizeof(std::complex<double>*));
	Aux_gcc		= (std::complex<double> *) calloc(window_size_2, sizeof(std::complex<double>));
	DOA_hist	= (double *) calloc(hist_length, sizeof(double));
	DOA_class	= (unsigned int *) calloc(hist_length, sizeof(unsigned int));
	DOA_kmean	= (double *) calloc(n_sources, sizeof(double));
	DOA_mean	= (double *) calloc(n_sources, sizeof(double));
	DOA_stdev	= (double *) calloc(n_sources, sizeof(double));
	DOA_valid	= (double *) calloc(n_sources, sizeof(double));
	kalmanState = (double **) calloc(4, sizeof(double*));
	covMatrix   = (double **) calloc(4*n_sources, sizeof(double*));
	freqmask		= (double *) calloc(window_size_2, sizeof(double));
	freqs				= (double *) calloc(window_size_2, sizeof(double));
	for(i = 0; i < window_size_2; ++i) {
		freqmask[i] = 1.0;
	}
	f0 = sample_rate/window_size_2;
	freqs[0] = 0.0;
	for(i = 1; i < (unsigned int)nframes; ++i){
		freqs[i] = f0*((double)i);
		freqs[window_size_2-i] = freqs[i];
	}
	freqs[nframes] = f0*nframes;
	
	counter		= (unsigned int *) calloc(n_sources, sizeof(unsigned int));
	dcounter	= (int *) calloc(n_sources, sizeof(int));
	ecounter	= (int *) calloc(n_sources, sizeof(int));
	
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-180.0,180.0);
	
	for (i = 0; i < n_sources; ++i)
	{
		DOA_kmean[i] = 360.0/n_sources*i; //+ distribution(generator); 360.0/2.0/n_sources;	// "optimal" initialization for the k-means algorithm
		//DOA_kmean[i] = distribution(generator);	// random initialization for the k-means algorithm
		if (DOA_kmean[i] > 180.0) {
			DOA_kmean[i] -= 360.0;
		}
		//DOA_kmean[i] = distribution(generator);	// random initialization for the k-means algorithm
		DOA_stdev[i] = 360.0/2.0/n_sources;
	}
	
	for (i = 0; i < 4; ++i) {
		kalmanState[i] = (double *) calloc(n_sources, sizeof(double));
		for (j = 0; j < n_sources; ++j) {
			covMatrix[j*4+i] = (double *) calloc(4, sizeof(double));
		}
	}
	
	// initialize Kalman state:
	double initialState[2];
	for (i = 0; i < n_sources; ++i) {
		angle2state(DOA_kmean[i], initialState);
		
		kalmanState[0][i] = initialState[0];
		kalmanState[1][i] = initialState[1];
		
		/*
		covMatrix[i*4+0][0] = 0.000002493289818;	covMatrix[i*4+0][2] = 0.000116781724500;
		covMatrix[i*4+1][1] = 0.000002493289818;	covMatrix[i*4+1][3] = 0.000116781724500;
		covMatrix[i*4+2][0] = 0.000116781724500;	covMatrix[i*4+2][2] = 0.005469870000000;
		covMatrix[i*4+3][1] = 0.000116781724500;	covMatrix[i*4+3][3] = 0.005469870000000;
		*/
	}
	
	
	for (i = 0; i < n_in_channels; ++i) {
		X_gcc[i]	= (std::complex<double> *) calloc(window_size_2, sizeof(std::complex<double>));
	}
	
	// - FFTW3 buffers
	i_fft_2N = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * window_size_2);
	i_time_2N = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * window_size_2);
	o_fft_2N = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * window_size_2);
	o_time_2N = (std::complex<double>*) fftw_malloc(sizeof(std::complex<double>) * window_size_2);
	
	i_forward_2N = fftw_plan_dft_1d(window_size_2, reinterpret_cast<fftw_complex*>(i_time_2N), reinterpret_cast<fftw_complex*>(i_fft_2N), FFTW_FORWARD, FFTW_MEASURE);
	o_inverse_2N = fftw_plan_dft_1d(window_size_2, reinterpret_cast<fftw_complex*>(o_fft_2N), reinterpret_cast<fftw_complex*>(o_time_2N), FFTW_BACKWARD, FFTW_MEASURE);
	
	// - hann window
	hann = (jack_default_audio_sample_t *) calloc(nframes, sizeof(jack_default_audio_sample_t)); 
	for(i = 0; i < (unsigned int)nframes; ++i) {
		hann[i] = 0.5 - 0.5*cos(2.0*M_PI* ((double) i/(nframes-1)));
	}
	
	/* display the current sample rate. */
	printf ("SoundLoc: JACK Engine sample rate: %.0f\n", sample_rate);
	printf ("SoundLoc: JACK Window size: %d\n\n", nframes);
	
	char portname[13];
	input_ports = (jack_port_t**) malloc(n_in_channels*sizeof(jack_port_t*));
	for(i = 0; i < n_in_channels; ++i) {
		sprintf(portname, "input_%d", i+1);
		input_ports[i] = jack_port_register (client, portname, JACK_DEFAULT_AUDIO_TYPE, JackPortIsInput, 0);
		if (input_ports[i] == NULL) {
			printf("SoundLoc: No more JACK ports available after creating input port number %d\n",i);
			exit (1);
		}
	}	
	
	if (jack_activate (client)) {
		printf ("SoundLoc: Cannot activate client.");
		exit (1);
	}
	
	printf ("SoundLoc: Agent activated.\n");
	
	if(connect_ports){
		std::cout << "SoundLoc: Connecting input ports.\n";fflush(stdout);
		const char **port_names = jack_get_ports (client, NULL, NULL, JackPortIsPhysical|JackPortIsOutput);
		if (port_names == NULL) {
			printf("SoundLoc: no physical capture ports\n");
			exit (1);
		}
		for(unsigned int i = 0;i <n_in_channels;i++){
			printf("SoundLoc: connecting %s to %s\n",port_names[i],jack_port_name (input_ports[i]));fflush(stdout);
			if (jack_connect (client, port_names[i], jack_port_name (input_ports[i]))) {
				printf("SoundLoc: cannot connect ports: %s <> %s\n",port_names[i],jack_port_name (input_ports[i]));
			}
		}
		free (port_names);
	}
}
