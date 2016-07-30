// DetectionEngine.h
// Landing Pad marker detection generic interface
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   22.09.2013
// Description: This class implements a common, generic interface for several landing pad marker
//              detection methods, implemented as derived classes through inheritance. The generic
//              interface includes both the framework class which holds all detection methods and
//              the detection method base class from which specific detection methods are derived
//              for implementation.
//              See DetectionEngine.cpp for a more thorough description of classes and detection
//              method architecture used.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

// -- External libraries --------------------------------------------------------------------------

#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
//  using namespace cv;
//  using namespace std;

// -- ARGOS Vision libraries ----------------------------------------------------------------------

#include "Definitions.h"
#include "DMIdentifiers.h"
#include "Profiling.h"
#include "Marker.h"
#include "Log.h"

// -- Detection methods ---------------------------------------------------------------------------

#include "DetectionMethod.h"
#include "DMThresholdContour.h"
#include "DMMSQI.h"

// == CDetectionEngine: detection methods framework ===============================================

class CDetectionEngine {
public:
	// Detection engine specifications
	vector<CDetectionMethod *> DetectionMethods;		// List of available detection methods
	int                        ActiveDM;				// Index of active detection method
	// Construction and destruction
	CDetectionEngine(void);
	~CDetectionEngine(void);
	// Engine configuration
	void   ResetEngine();								// Set initial default configuration
	void   AddDetectionMethod(CDetectionMethod &dm);	// Add a new detection method to the engine
	// Engine interface
	void   ShowEngineInfo();							// Show engine information on output frame
	int    GetAvailableDMNum();							// Get the number of available detection methods
	void   SetActiveDM(int IdDM);						// Set the active detection method from its id
	int    GetActiveDM();								// Get the active detection method id
	string GetActiveDMName();							// Get the active detection method name
	int    GetActiveDMParNumber();						// Get the number of configuration parameters of active detection method
	// Detection interface
	Mat    Process(Mat &frame);							// Frame processing function
	void   CheckKeyboard();								// Keyboard input handling
	void   ShowInfo();									// Results and information generation on output frame
	void   LogInfo();									// Results and information logging (console and/or file)
	Mat    GetResultsFrame();							// Get the results frame of the active detection method
	Mat    GetProcessFrame();							// Get the process frame of the active detection method
};

