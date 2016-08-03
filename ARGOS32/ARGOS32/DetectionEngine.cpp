// DetectionEngine.cpp
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
//              The framework class and the method base class define the same functional interface,
//              allowing an external program to use any of them without changes. The framework
//              interface includes additional functions for selecting the active detection method
//              and other configuration options (for example, it is possible to run two or more
//              individual detection methods in parallel).
//              The interface defined for both classes also includes a keyboard handling function
//              and result presentation functions.
//              The specific detection methods are defined in classes derived from CDetectionMethod
//              in separate files for each method. These files are named with the prefix "DM" for
//              easy identification of the algorithms implementation. Each detection method is
//              uniquely identified by an integer constant, which is set in the detection method
//              constructor. Available identifiers are defined externally in DMIdentifiers.h for
//              convenience.
//              The detection process for each detection method is carried out in a single function
//              called Process(). Each detection method can define more private helper methods
//              to be used for processing. However, the interface is kept in a single function to
//              give more flexibility in the internal implementation of each method and to get a
//              more accurate measurement when profiling each method.
//              The CDetectionMethod base class implements several helper functions that provide
//              common functionality for derived classes. As an example, the code for drawing a
//              dot marker is implemented here once instead of repeating it in each derived class.
//              The CDetectionMethod base class defines also the required members for logging
//              debug messages, warnings and other information to a log file. Logging is done using
//              the Log.h library (refer to this file for configuration and usage information).
//              The class exposes log configuration functions to define the log file and the log
//              level, and to configure if logging is duplicated on screen (console) or not.
//              Being a base class for each implemented detection method, log functions will be
//              called from each derived class, and thus logging can be independently configured
//              for each detection method (for example, using different log files).
//              The base class declares also two Mat members to be used for referencing frames
//              to be shown in external, main program windows. This is done this way to allow for
//              a common reference to images to be shown; the algorithm implementation will set
//              these images as references to the frame variables required.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

// -- Class declarations --------------------------------------------------------------------------

#include "DetectionEngine.h"
using namespace cv;

// == CDetectionEngine: detection methods framework ===============================================

// -- Construction and destruction ----------------------------------------------------------------

CDetectionEngine::CDetectionEngine(void) {
	// Add and configure available detection methods
	DetectionMethods.resize(DM_COUNT);
	DetectionMethods[DM_THRESHOLD] = (CDetectionMethod *) new CDMThresholdContour();
	DetectionMethods[DM_THRESHOLD]->SetupLog("DMThreshold");
	DetectionMethods[DM_MSQI] = (CDetectionMethod *) new CDMMSQI();
	DetectionMethods[DM_MSQI]->SetupLog("DMMSQI");
	ActiveDM = DM_THRESHOLD;
}

CDetectionEngine::~CDetectionEngine(void) {
	// Free allocated detection methods
	for (int i=0; i<(int) DetectionMethods.size(); i++)
		delete DetectionMethods[i];
}

// -- Engine configuration ------------------------------------------------------------------------

void CDetectionEngine::ResetEngine() {
	// Set initial default configuration
}

void CDetectionEngine::AddDetectionMethod(CDetectionMethod &dm) {
	// Add a new detection method to the engine
}

int CDetectionEngine::GetAvailableDMNum() {
	// Get the number of available detection methods
	return DetectionMethods.size();
}

void CDetectionEngine::SetActiveDM(int IdDM) {
	// Set the active detection method from its id
	ActiveDM = IdDM;
}

int CDetectionEngine::GetActiveDM() {
	// Get the active detection method id
	return ActiveDM;
}

string CDetectionEngine::GetActiveDMName() {
	// Get the active detection method name
	return DetectionMethods[ActiveDM]->DMName;
}

int CDetectionEngine::GetActiveDMParNumber() {
	// Get the number of configuration parameters of active detection method
	return DetectionMethods[ActiveDM]->GetParNumber();
}

// -- Detection interface -------------------------------------------------------------------------

Mat CDetectionEngine::Process(Mat &frame) {
	// Frame processing function
	Mat res;
	res = DetectionMethods[ActiveDM]->Process(frame);
	return res;
}

void CDetectionEngine::CheckKeyboard() {
	// Keyboard input handling
	DetectionMethods[ActiveDM]->CheckKeyboard();
}

void CDetectionEngine::ShowInfo() {
	// Results and information generation on output frame
	DetectionMethods[ActiveDM]->ShowInfo();
}

void CDetectionEngine::LogInfo() {
	// Results and information logging (console and/or file)
	DetectionMethods[ActiveDM]->LogInfo();
}

Mat CDetectionEngine::GetResultsFrame() {
	// Get the results frame of the active detection method
	return DetectionMethods[ActiveDM]->ResultsFrame;
}

Mat CDetectionEngine::GetProcessFrame() {
	// Get the process frame of the active detection method
	return DetectionMethods[ActiveDM]->ProcessFrame;
}