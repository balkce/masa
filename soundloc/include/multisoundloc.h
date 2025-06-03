/*
	Source coded by Caleb Rascon and Luis Miguel Gato
	IIMAS, UNAM
	México
	
	This library, using JACK, obtains the angle of different sources in the range of [-180 180].
	
	It requires three channels of independent directional microphones on top of the robot, set at 0 dB gain.
	The following is a simple diagram of their assumed setup:
	
                                                    (front mic)
                                                         ^
                                                         |
                                                         |
                                                        / \
                                                       /   \
                                                      ˇ     ˇ
                                             (left mic)     (right mic)


	The microphones should be set up in a triangle, with an equal distance between their ends and
	a 60 degree angle between left-front, front-right, and right-left.
	
	The angle of one source detected from each pair is compared with the other two to decide which
	provides the best resolution (e.g. the one most in front of a pair, e.g. the smallest angle of all).
	Then, a redundancy check is carried out to see if all the pairs are detecting the source coming from
	the same direction.

	The angle given starts from the line in which the front mic lies. A negative angle is to the
	left; a positive angle is to the right, all the way to the back of the front mic.
	
	If more than source is active, because the sample size is small, it is likely that one sample is able
	to get through redundancy check and provide an adequate angle of one source, and the next redundant
	angle of another source. For this purpose, an expectation system is built on top of the DOA estimator
	to mantain in memory the number of sources and each of their angles, which provides a basic CASA system.

	When running, it will output to standard out its estimate of the angle, and the current angle of the robot
	simulation until it reaches a good enough angle.
	
	Requirements:
		- A soundcard with at least three channels of input, and set to be the system's default.
		- A running JACK Server.
*/
#ifndef _LIBMULTISOUNDLOC_H_
#define _LIBMULTISOUNDLOC_H_

#include <time.h>
#include <vector>

//Glib, for threading (GThread)
#include <glib.h>

// Eigen: C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
#include <Eigen/Eigen>


#define TDE_CC 1
#define TDE_GCCPHAT 2
#define TRACK_CLUSTER 1
#define TRACK_MCMCDA 2
#define TRACK_KALMAN 3
#define INVALID 999999
#define PI 3.141592653589793238462643383279502884

#ifndef M_PI
#define M_PI 3.1415926535897932384
#endif

#ifndef ROUND
#define ROUND(a) (PLINT)((a)<0. ? ((a)-0.5) : ((a)+0.5))
#endif

using namespace std;

// User description and registry variable
typedef struct{
	double doa;
	time_t doa_time;
	float confidence;
} source;

extern vector < source > sources;
extern vector < double > doafrequencies;

extern void millisleep(int milli);
extern void soundloc_finalize();
extern void soundloc_clear();
extern void soundloc_init(double distance_between_mics_in, int max_number_sources, int connect_ports, int gcc_style, double gcc_th_in, double redundancy_th, int dynamic_gcc_th, int moving_average, int moving_factor, int memory_factor, int kmeans_min_dist_in, double noise_threshold_in, double noise_peak_change_in, int verbose);
extern void doafrequencies_clear();
extern void build_freqmask(int freqselect_range);


// Tool functions
#define DEG2RAD 0.017453292519943		// useful to convert from degrees to radians
#define RAD2DEG 57.295779513082323f		// useful to convert from radians to degrees

#define TAU 0.0213333	// 1024/48000: observation time
#define ACC 1.0			// modeled acceleration
#define NVAR 0.005	// measurement noise variance
//#define ACC 0.05		// modeled acceleration
//#define NVAR 0.000001	// measurement noise variance

// state transition model:
const Eigen::MatrixXd F = (Eigen::MatrixXd(4,4) << 1.0, 0.0, TAU, 0.0, 0.0, 1.0, 0.0, TAU, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();
const Eigen::MatrixXd Q = (Eigen::MatrixXd(4,4) << ACC*TAU*TAU*TAU*TAU/4, 0.0, ACC*TAU*TAU*TAU/2, 0.0, 0.0, ACC*TAU*TAU*TAU*TAU/4, 0.0, ACC*TAU*TAU*TAU/2, ACC*TAU*TAU*TAU/2, 0.0, ACC*TAU*TAU, 0.0, 0.0, ACC*TAU*TAU*TAU/2, 0.0, ACC*TAU*TAU).finished();
// measurement model:
const Eigen::MatrixXd H = (Eigen::MatrixXd(2,4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0).finished();
const Eigen::MatrixXd R = (Eigen::MatrixXd(2,2) << NVAR, 0.0, 0.0, NVAR).finished();

// 4x4 identity matrix:
const Eigen::MatrixXd I4x4 = (Eigen::MatrixXd(4,4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished();

double state2angle (double* state);
void angle2state (double angle, double* state);
double angleRedundancy (double*, double*, double);
void angleTranslation (double *, float *);
void angleTranslation (double *);

void kalman (double y_array[2], double *x_array, double (*P_array)[4]);

void kmeans (double* DOA_hist, unsigned int *DOA_class, double* DOA_mean, unsigned int* class_count, unsigned int n_sources, unsigned int hist_length);

int max (complex<double>*, int, int, double*);
double mean (double*, int);

void phat (complex<double>*, complex<double>*, complex<double>*, int, int, int, int);

double unwrap (int, int, double);

#endif
