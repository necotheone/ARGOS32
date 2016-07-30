// DetectionMethod.cpp
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

// -- Class declarations --------------------------------------------------------------------------

#include "DetectionMethod.h"

// == CDetectionMethod: detection method base class ===============================================

// -- Construction and destruction ----------------------------------------------------------------

CDetectionMethod::CDetectionMethod(void) {
	LogFileName = "";
	LogFileMode = "w";
	LogToConsole = false;
	FILELog::ReportingLevel() = logDEBUG3;
}

CDetectionMethod::~CDetectionMethod(void) {
	CloseLogFile();
}

// -- Log control interface -----------------------------------------------------------------------

bool CDetectionMethod::SetupLog(string fn,string m,bool c) {
	// Setup logging: filename, file access mode and console output
	LogFileName = fn;
	LogFileMode = m;
	LogToConsole = c;
	bool res = OpenLogFile();
	return res;
}

void CDetectionMethod::SetLogLevel(TLogLevel n) {
	// Set active log level
	FILELog::ReportingLevel() = n;
}

bool CDetectionMethod::OpenLogFile() {
	// Open log file and configure logging engine based on log config members
	FILELog::ReportingLevel() = logDEBUG3;
	LogFile = fopen(LogFileName.c_str(),LogFileMode.c_str());
	if (LogFile!=NULL)
		Output2FILE::Stream() = LogFile;
	//FILELog::ConsoleActive() = LogToConsole;
	return (LogFile!=NULL);
}

void CDetectionMethod::CloseLogFile() {
	// Close log file if opened
	fclose(LogFile);
	LogFile = NULL;
}

// -- Common implementation of virtual interface --------------------------------------------------

// -- Detection interface -------------------------------------------------------------------------

Mat  CDetectionMethod::Process(Mat &frame) {
	// Frame processing function
	return ResultsFrame;
}

void CDetectionMethod::CheckKeyboard() {
	// Keyboard input handling
}

void CDetectionMethod::ShowInfo() {
	// Results and information generation on output frame
}

void CDetectionMethod::LogInfo() {
	// Results and information logging (console and/or file)
}

int CDetectionMethod::GetParNumber() {
	// Get the number of configuration parameters
	return 0;
}

float  CDetectionMethod::GetParValue(int p) {
	// Get the value of parameter p (index)
	return 0.0;
}

string CDetectionMethod::GetParName(int p) {
	// Get the name of parameter p (index)
	return string("");
}

bool CDetectionMethod::GetParType(int p) {
	// Get the type of parameter p (index)
	return false;
}

float  CDetectionMethod::GetParMax(int p) {
	// Get the maximum allowed value of parameter p (index)
	return 0.0;
}

float  CDetectionMethod::GetParMin(int p) {
	// Get the minimum allowed value of parameter p (index)
	return 0.0;
}

void CDetectionMethod::ResetStatistics() {
	// Reset common statistics variables, called from derived classes only
	nFrames = 0;
	TotProcTime = 0.0;
}

void CDetectionMethod::SetParValue(int p, float v) {
	// Set the value for specified parameter 
}

// -- Common helper functions ---------------------------------------------------------------------

void CDetectionMethod::drawPointMarker(Mat &f,Point c,int s,Scalar &color,int thickness,bool rot) {
	Point c1, c2, c3, c4;
	int d1, d2;
	if (rot) {
		d1 = (int) ((float) s*0.707);
		d2 = d1;
	}
	else {
		d1 = 0;
		d2 = s;
	}
	c1.x = c.x-d1; c1.y = c.y-d2;
	c2.x = c.x+d1; c2.y = c.y+d2;
	c3.x = c.x-d2; c3.y = c.y+d1;
	c4.x = c.x+d2; c4.y = c.y-d1;
	line(f,c1,c2,color,thickness);
	line(f,c3,c4,color,thickness);
}

void CDetectionMethod::drawInfoText(Mat &f,int line,string text,cv::Scalar color,bool shadow) {
	if (shadow)
		putText(f,text,Point(DMGUI_LEFTMARGIN+1,DMGUI_LINEORIGIN+line*DMGUI_LINEHEIGHT+1),FONT_HERSHEY_COMPLEX_SMALL, DMGUI_FONTSCALE, cvScalar(0,0,0), 1, CV_AA);
	putText(f,text,Point(DMGUI_LEFTMARGIN,DMGUI_LINEORIGIN+line*DMGUI_LINEHEIGHT),FONT_HERSHEY_COMPLEX_SMALL, DMGUI_FONTSCALE, color, 1, CV_AA);
}

void CDetectionMethod::drawInfoText(Mat &f,int line,vector<string> &text,cv::Scalar color,bool shadow) {
	// plot several consecutive lines of text with content defined in string vector
	for (int i=0; i<text.size(); i++)
		drawInfoText(f,line+i,text[i],color,shadow);
}

