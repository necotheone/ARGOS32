// DetectionMethod.h
// Landing Pad marker detection method base class
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   25.09.2013
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
using namespace std;

// -- ARGOS Vision libraries ----------------------------------------------------------------------

#include "Definitions.h"
#include "DMIdentifiers.h"
#include "Profiling.h"
#include "Marker.h"
#include "Log.h"

// -- Graphical interface definitions -------------------------------------------------------------

#define DMGUI_LINEORIGIN   20
#define DMGUI_LINEHEIGHT   20
#define DMGUI_LEFTMARGIN   20
#define DMGUI_FONTSCALE    0.8     

// == CDetectionMethod: detection method base class ===============================================

class CDetectionMethod {
public:
	// Detection method specification
	int         Id;								// Detection method identifier
	string      DMName;
	string      DMVersion;
	// Detected landing pad object
	CLandingPad LandingPad;						// Landing pad detection results
	// Common statistics
	long        nFrames;						// Total number of processed frames
	float       TotProcTime;					// Total processing time
	// Profiling
	CProfiling  Profiling;
	// Frames to show
    cv::Mat     ResultsFrame;					// Frame image to be shown with results of detection method
    cv::Mat     ProcessFrame;					// Frame image to be shown with intermediate processing
	// Log configuration
protected:
	string      LogFileName;					// Active log file name
	string      LogFileMode;					// Log file access mode
	bool        LogToConsole;					// Indicates if log is also sent to console
	int         LogLevel;						// Active log level
	FILE       *LogFile;						// Log file pointer
	// Construction and destruction
public:
	CDetectionMethod(void);
	~CDetectionMethod(void);
	// Log control interface
public:
	bool SetupLog(string fn,string m,bool c);	// Setup logging: filename, file access mode and console output
	void SetLogLevel(TLogLevel n);				// Set active log level
protected:
	bool OpenLogFile();							// Open log file and configure logging engine based on log config members
	void CloseLogFile();						// Close log file if opened
	// Detection method interface
public:
	virtual cv::Mat Process(cv::Mat &frame);      // Frame processing function
	virtual void    CheckKeyboard();              // Keyboard input handling
	virtual void    ShowInfo();                   // Results and information presentation on frame
	virtual void    LogInfo();                    // Results and information logging on console and file
	virtual int     GetParNumber();               // Get the number of configuration parameters
	virtual float   GetParValue(int p);           // Get the value of parameter p (index)
	virtual string  GetParName(int p);            // Get the name of parameter p (index)
	virtual bool    GetParType(int p);            // Get the type of parameter p (index)
	virtual float   GetParMax(int p);             // Get the maximum allowed value of parameter p (index)
	virtual float   GetParMin(int p);             // Get the minimum allowed value of parameter p (index)
	virtual void    SetParValue(int p, float v);  // Set the value for specified parameter 
protected:
	virtual void ResetStatistics();				// Reset statistics variables, called only from derived classes
public:
	// Common helper functions
	void drawPointMarker(cv::Mat &f, cv::Point c,int s, cv::Scalar &color,int thickness=1,bool rot=false);
	void drawInfoText(cv::Mat &f,int line,string text,cv::Scalar color,bool shadow=false);
	void drawInfoText(cv::Mat &f,int line,vector<string> &text,cv::Scalar color,bool shadow=false);
};
