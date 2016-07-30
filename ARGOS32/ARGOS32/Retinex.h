// Retinex.cpp
// Retinex PDE algorithm implementation for OpenCV
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   16.09.2013
// Description: This class implements the Retinex PDE algorithm for OpenCV
// References: Nicolas Limare, Ana Belén Petro, Catalina Sbert, Jean-Michel Morel (2011)
//             Retinex Poisson Equation: a Model for Color Perception
//             In IPOL Journal - Image Processing On Line
//             http://www.ipol.im/pub/art/2011/lmps_rpe/
//             OpenCV 2.4.6 - http://docs.opencv.org/index.html
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "opencv2/opencv.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "..\ExtLibraries\fftw-3.3.3\include\fftw3.h"

using namespace cv;
using namespace std;

// M_PI is a POSIX definition
#ifndef M_PI
// Macro definition for Pi
#define M_PI 3.14159265358979323846
#endif

class CRetinex {
public:
	// Algorithm configuration and control
	bool   IsImageLoaded;		// Indicates if an image has been loaded for processing
	bool   IsResultAvailable;	// Indicates if the resulting processed image is available
	float  Threshold;			// Retinex threshold
	// Image data
	int                   it;			// Image type
	size_t                nx, ny, nc;	// Image size
	Mat                   SrcImage;		// Source image to be processed
	vector<Mat>           SrcChannels;
	vector<vector<float>> SrcFChannels;	// Source channels splitted (BGR order)
	Mat                   ResImage;		// Resulting image after processing
	vector<Mat>           ResChannels;
	vector<vector<float>> ResFChannels;	// Resulting channels (BGR order)
	// Internal data
	vector<float *> data;
	vector<float *> data_rtnx;
	vector<float>   vdata_fft;
	vector<float>   vdata_tmp;
	float *         data_fft;
	float *         data_tmp;
	float *         data_res;			// Pointer to buffer where DFT result is stored
	fftwf_plan      dct_fw;
	fftwf_plan      dct_bw;
	bool            IsPlanCreated;		// Flag indicating if plans are defined
	// Construction and destruction
	CRetinex(void);
	~CRetinex(void);
	// Algorithm control methods
	void  LoadImage(Mat &im);		// Load the image to be processed and prepare algorithm data
	void  SplitImage();				// Split the source image into separate channels
	void  MergeImage();				// Merge resulting channels into resulting image
	void  SetThreshold(float t);	// Set the Retinex threshold
	float GetThreshold();			// Get the currently active Retinex threshold
	void  CreateFFTWPlans();		// Create plans for FFTW before actually calculating DFT
	void  RemoveFFTWPlans();		// Free FFTW allocated resources for plans
	void  Run();					// Execute algorithm
	// Algorithm implementation methods
	float *RetinexPDE(float *data, size_t nx, size_t ny, float t);
	static void MeanDt(const float *data, size_t size, double *mean_p, double *dt_p);
	void NormalizeMeanDt(float *data, const float *ref, size_t size);
	static float *DiscreteLaplacianThreshold(float *data_out, const float *data_in, size_t nx, size_t ny, float t);
	static double *CosTable(size_t size);
	static float *RetinexPoissonDCT(float *data, size_t nx, size_t ny, double m);
};

