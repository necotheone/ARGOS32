// DMMSQI.h
// Landing Pad detection method based on modified Self-Quotient Image
// ------------------------------------------------------------------------------------------------
// Author: David P�rez-Pi�ar
// Rev.:   1.0
// Date:   22.09.2013
// Description: This class implements a landing pad detection method based on based on modified
//              Self-Quotient Image and contour extraction.
//              See DMMSQI.cpp for the detailed description and implementation of this detection
//              method.
//              See DetectionEngine.cpp for a more thorough description of classes and detection
//              method architecture used.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// -- External libraries --------------------------------------------------------------------------

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

// -- ARGOS Vision libraries ----------------------------------------------------------------------

#include "Definitions.h"
#include "DMIdentifiers.h"
#include "Profiling.h"
#include "Marker.h"
#include "DetectionEngine.h"

// -- Configuration parameters definitions --------------------------------------------------------
// The following definitions are the configuration parameters indexes in the Parameters array.
// The parameters indexes must start in 0 and must follow the order established by the
// parameter structure defined in the Config union.

#define DMCFG_MS_MSQITHRESHOLD      0
#define DMCFG_MS_MSQISIGMA          1
#define DMCFG_MS_CTRMINAREA         2
#define DMCFG_MS_CTRMINROUNDNESS    3
#define DMCFG_MS_CTRCENTERMAXERR    4

#define DMCFG_MS_PARNUMBER          5

// == CDetectionMethod: detection method base class ===============================================

class CDMMSQI : CDetectionMethod {
public:
	// Configuration parameters
	typedef union tConfig {					// This union defines two ways of accessing configuration parameters
		struct {								// Access by variable name (internal access)
			float    MSQIThreshold;				// Threshold used for generating binary frame from SQI image
			float    MSQISigma;					// Sigma value (kernel size) for high-pass Gaussian filtering
			float    ContourMinArea;			// Threshold used for removing small details from processing
			float    ContourMinRoundness;		// Threshold used for removing contours with low roundness
			float    RingCenterMaxErr;			// Maximum error (ratio to outer radius) between inner and outer centers
		};
		float Parameters[DMCFG_MS_PARNUMBER];	// Access by index (external access: use DMCFG_* definitions as index)
	};
	tConfig        Config;					// Configuration parameters
	tConfig        ConfigMax;				// Configuration maximum parameter values
	tConfig        ConfigMin;				// Configuration minimum parameter values
	vector<string> ConfigNames;				// Configuration parameter names
	vector<bool>   ConfigTypes;				// Configuration parameter types (true for boolean flags)
	int            ConfigNum;				// Number of configuration parameters
	// Processing variables
	Mat                    cameraFrame;		// Frame captured from camera/file
	Mat                    pframe;			// Preprocessing resultant frame
	Mat                    gframe;			// Gray scale converted frame
	Mat                    bframe;			// Binary frame obtained by thresholding gray scale frame
	vector<vector<Point>>  contours;		// Set of contours found in the binary frame
	vector<Vec4i>          hierarchy;		// Hierarchy of contours
	vector<Moments>        mu;				// Contour moments for area and length calculations
	vector<Point2f>        mc;				// Contour mass centers
	vector<float>          perimeter;		// Contour perimeters
	vector<float>          roundness;		// Contour roundness
	vector<bool>           valid;			// Indicates if contour is valid (detected as circle)
	vector<bool>           smallSize;		// Indicates if contour is too small
	vector<bool>           notcircle;		// Indicates if contour roundness is enough
	vector<bool>           nohole;			// Indicates if contour has no child
	vector<bool>           outerring;		// Indicates if contour is outer ring circle
	vector<bool>           innerring;		// Indicates if contour is inner ring circle
	// Detection method statistics
	int                    nContours;		// Total number of contours found
	int                    nCandidates;		// Number of valid contour candidates
	int                    nSmall;			// Number of small rejected contours
	int                    nNotCircle;		// Number of not circular rejected contours
	int                    nNoHole;			// Number of top-level without children rejected contours
	int                    nOuterRing;		// Number of outer ring circles detected
	int                    nInnerRing;		// Number of inner ring circles detected
	int                    nRings;			// Number of rings detected
	// Construction and destruction
	CDMMSQI(void);
	~CDMMSQI(void);
	// Detection interface (implementation of virtual base methods)
	Mat    Process(Mat &frame);					// Frame processing function
	void   CheckKeyboard();						// Keyboard input handling
	void   ShowInfo();							// Results and information presentation on frame
	void   LogInfo();							// Results and information logging on console and file
	void   ResetStatistics();					// Reset statistics variables
	void   LogFrameStatistics();				// Log frame processing statistics
	int    GetParNumber();						// Get the number of configuration parameters
	float  GetParValue(int p);					// Get the value of parameter p (index)
	string GetParName(int p);					// Get the name of parameter p (index)
	bool   GetParType(int p);					// Get the type of parameter p (index)
	float  GetParMax(int p);					// Get the maximum allowed value of parameter p (index)
	float  GetParMin(int p);					// Get the minimum allowed value of parameter p (index)
	void   SetParValue(int p, float v);			// Set the value for specified parameter 
private:
	// Helper functions for algorithm implementation
	void SetDefaultConfig();					// Set default values for configuration parameters
	void ResetDMStatistics();					// Reset method-specific statistics
	void ResizeProcessVar(int s);				// Resize internal processing variables to the size s
	void InitDetectionFlags();					// Initialize detection flags
	Mat  Normalize255(const Mat& src);			// Normalize a given image into a value range between 0 and 255
	Mat  MSQIPreprocessing(InputArray src, int th=3, int sigma0=1);
};

