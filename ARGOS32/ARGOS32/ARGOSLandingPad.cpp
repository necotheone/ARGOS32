// ARGOSLandingPad.cpp
// Landing Pad marker detection
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   02.09.2013
// Description: This software tests the vision-based landing pad recognition algorithms which
//              will be used later for automatic landing of ARGOS vehicles with several goals,
//              including their ability to land and stow automatically to their charge base.
//              Detection algorithms are implemented using a modular detection engine defined in
//              DetectionEngine.cpp, DetectionMethod.cpp and DM* source files.
// References: Lange, S., Sünderhauf, N., & Protzel, P. (2008)
//             Autonomous Landing for a Multirotor UAV Using Vision
//             In Workshop Proceedings of SIMPAR 2008 Intl. Conf. on SIMULATION, MODELING and
//             PROGRAMMING for AUTONOMOUS ROBOTS, Venice, Italy, 2008 Nov, 3-4, pp. 482-491
//             ISBN 978-88-95872-01-8
//             OpenCV 2.4.6 - http://docs.opencv.org/index.html
///////////////////////////////////////////////////////////////////////////////////////////////////

// -- External libraries --------------------------------------------------------------------------

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

// -- ARGOS Vision libraries ----------------------------------------------------------------------

#include "DMIdentifiers.h"
#include "Definitions.h"
#include "Profiling.h"
#include "Marker.h"
#include "DetectionEngine.h"
//#include "Retinex.h"

// -- ARGOS Landing Pad Version -------------------------------------------------------------------

#include "ALPVersion.h"

// -- Namespaces ----------------------------------------------------------------------------------

using namespace cv;
using namespace std;

// -- Console help --------------------------------------------------------------------------------

static void ShowAppInfo() {
	string txt = format("ARGOS Landing Pad - Version %02d.%02d.%02d.%04d",ALP_VERSION_MAJOR,ALP_VERSION_MINOR,ALP_VERSION_BLDHI,ALP_VERSION_BLDLW);
	cout << txt << endl;
	cout << "------------------------------------------" << endl;
	cout << "Software for testing ARGOS Landing Pad detection algorithms" << endl << endl;
}

static void ShowLiveCaptureHelp() {
	cout << "Hot-keys for live video camera input:" << endl;
	cout << "  <ESC>, q - quit the program" << endl;
	cout << "  g        - start/stop processing images" << endl;
	cout << "  m        - change processing mode" << endl;
}

static void ShowVideoFileHelp() {
	cout << "Hot-keys for video file input:" << endl;
	cout << "  <ESC>, q - quit the program" << endl;
	cout << "  g        - start/stop processing images" << endl;
}

static void ShowParKeyHelp() {
	cout << "Hot-keys for detection method handling:" << endl;
	cout << "  F1..F12   - Select active detection method" << endl;
	cout << "  1..9      - Select active configuration parameter" << endl;
	cout << "  Left/Down - Decrement active parameter value (small step)" << endl;
	cout << "  Up/Right  - Increment active parameter value (small step)" << endl;
	cout << "  PrvPage   - Decrement active parameter value (big step)" << endl;
	cout << "  AdvPage   - Increment active parameter value (big step)" << endl;
	cout << "  Home      - Set active parameter to minimum value" << endl;
	cout << "  Del       - Set active parameter to default value" << endl;
	cout << "  End       - Set active parameter to maximum value" << endl;
}

static void ShowAppHelp() {
	cout << "Usage: ARGOSLandingPad [input_source]" << endl;
	cout << "Parameters:\n" << endl;
	cout << "  [input_source]  Input source: Name of video file or number of camera" << endl;
	cout << "                  If not specified, live view from the default camera is used" << endl << endl;
	ShowLiveCaptureHelp();
	ShowVideoFileHelp();
	ShowParKeyHelp();
}

// -- Capture variables and definitions -----------------------------------------------------------

#define ALP_RESULTSWND "ResultsWindow"
#define ALP_PROCESSWND "ProcessWindow"

bool         IsOpenedWndRes = false;	// Indicates if the results window is created
bool         IsOpenedWndPrc = false;	// Indicates if the process window is created
int          wx = 0;					// Results window width (pixels)
int          wy = 0;					// Results window height (pixels)
int          wlines = 0;				// Results window number of lines

VideoCapture Capture;					// Capture device (camera or video) for retrieving frames
Mat          cframe;					// Captured frame
bool         liveCapture = false;		// Indicates if input comes from camera (true) or file
bool         goProcessing = false;		// Indicates if processing is active (true)
bool         Quit = false;				// Flag to trigger finishing the application

// -- Detection engine ----------------------------------------------------------------------------

CProfiling Profiling;
CDetectionEngine DetectionEngine;

// -- Detection methods interface -----------------------------------------------------------------
// These functions get the available detection methods and configure the application for using
// them. Run-time selection of the active detection method is possible through keyboard strokes.
// Key-strokes are assigned to available methods starting at key 'F1' and topping at 'F12'; thus,
// a maximum of 12 detection methods can be available simultaneously.
// Two windows from showing results and processed frames are created. They will show the
// results frame and process frame objects of the active detection method. What image is actually
// stored in those frames depends on the internal implementation of the detection method itself.

int           ProcModNumber = 0;
int           ProcModeActive = 0;
vector<uchar> ProcModKeys;
int           ProcModParNum = 0;
int           ProcModParActive = 0;
float         ProcModParValue = 0.0;
float         ProcModParDefault = 0.0;
float         ProcModParMax = 0.0;
float         ProcModParMin = 0.0;
string        ProcModParName = "";
bool          ProcModParType = false;
float         ProcModParBigStep = 0.0;
float         ProcModParStep = 0.0;

void SetActiveParameter(int i) {
	ProcModParActive = i;
	ProcModParValue = DetectionEngine.DetectionMethods[ProcModeActive]->GetParValue(i);
	ProcModParName = DetectionEngine.DetectionMethods[ProcModeActive]->GetParName(i);
	ProcModParMin = DetectionEngine.DetectionMethods[ProcModeActive]->GetParMin(i);
	ProcModParMax = DetectionEngine.DetectionMethods[ProcModeActive]->GetParMax(i);
	ProcModParType = DetectionEngine.DetectionMethods[ProcModeActive]->GetParType(i);
	if (ProcModParType) {
		ProcModParBigStep = 1.0;
		ProcModParStep = 1.0;
	}
	else {
		ProcModParBigStep = (ProcModParMax-ProcModParMin)/100.0;
		ProcModParStep = ProcModParBigStep/10.0;
	}
	ProcModParDefault = ProcModParValue;
	// 	cout << "Activated DM Parameter " << i << endl;
	// 	cout << "  Name:     " << ProcModParName << endl;
	// 	cout << "  Is flag:  " << ProcModParType << endl;
	// 	cout << "  Value:    " << ProcModParValue << endl;
	// 	cout << "  Min:      " << ProcModParMin << endl;
	// 	cout << "  Max:      " << ProcModParMax << endl;
	// 	cout << "  Default:  " << ProcModParDefault << endl;
	// 	cout << "  Step:     " << ProcModParStep << endl;
	// 	cout << "  Big step: " << ProcModParBigStep << endl;
}

void ConfigDetectionEngine() {
	ProcModNumber = DetectionEngine.GetAvailableDMNum();
	ProcModeActive = DetectionEngine.GetActiveDM();
	ProcModParNum = DetectionEngine.GetActiveDMParNumber();
	ProcModKeys.resize(ProcModNumber);
	for (int i=0; i<ProcModNumber; i++)
		ProcModKeys[i] = 0x70+(uchar) i;
	SetActiveParameter(0);
	// 	cout << format("Detection engine: %d available methods",ProcModNumber) << endl;
}

void CreateFrameWindows(bool isProcActive) {
	// Creates frame windows for showing processing results
	int flags = CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_EXPANDED;
	if (!IsOpenedWndRes) {
		namedWindow(ALP_RESULTSWND,flags);
		Capture.read(cframe);
		int ix = cframe.cols;
		int iy = cframe.rows;
		wlines = (int) (iy/DMGUI_LINEHEIGHT)-1;
		float imf = (float) iy/(float) ix;
		wx = (CAPTUREWINDOW_WIDTH == -1 ? ix : CAPTUREWINDOW_WIDTH);
		wy = (CAPTUREWINDOW_WIDTH == -1 ? iy : (int) ((float) wx*imf));
		resizeWindow(ALP_RESULTSWND,wx,wy);
		moveWindow(ALP_RESULTSWND,10,10);
		IsOpenedWndRes = true;
	}
	if (isProcActive && !IsOpenedWndPrc) {
		if (wx==0 || wy==0) {
			Capture.read(cframe);
			int ix = cframe.cols;
			int iy = cframe.rows;
			wlines = (int) (iy/DMGUI_LINEHEIGHT)-1;
			float imf = (float) iy/(float) ix;
			wx = (CAPTUREWINDOW_WIDTH == -1 ? ix : CAPTUREWINDOW_WIDTH);
			wy = (CAPTUREWINDOW_WIDTH == -1 ? iy : (int) ((float) wx*imf));
		}
		namedWindow(ALP_PROCESSWND,flags);
		resizeWindow(ALP_PROCESSWND,wx,wy);
		moveWindow(ALP_PROCESSWND,10+(wx+25),10);
		IsOpenedWndPrc = true;
	}
}

void CloseResultsWindow() {
	if (IsOpenedWndRes) {
		destroyWindow(ALP_RESULTSWND);
		IsOpenedWndRes = false;
	}
}

void CloseProcessWindow() {
	if (IsOpenedWndPrc) {
		destroyWindow(ALP_PROCESSWND);
		IsOpenedWndPrc = false;
	}
}

void CloseFrameWindows() {
	CloseResultsWindow();
	CloseProcessWindow();
}

void ShowControlInfo(Mat &f) {
	// Shows information of current active detection method and parameters
	string txt;
	txt = format("Detection Method: %s",DetectionEngine.GetActiveDMName().c_str());
	DetectionEngine.DetectionMethods[ProcModeActive]->drawInfoText(f,wlines-ProcModParNum-1,txt,Scalar(250,200,200),true);
	Scalar color;
	for (int i=0; i<ProcModParNum; i++) {
		string name = DetectionEngine.DetectionMethods[ProcModeActive]->GetParName(i);
		float value = DetectionEngine.DetectionMethods[ProcModeActive]->GetParValue(i);
		txt = format("%s: %3.3f",name.c_str(),value);
		if (ProcModParActive==i)
			color = Scalar(250,250,250);
		else
			color = Scalar(250,200,200);
		DetectionEngine.DetectionMethods[ProcModeActive]->drawInfoText(f,wlines-ProcModParNum+i,txt,color,true);
	}
}

bool CheckKeyboard() {
	// Check general keyboard strokes
	// OpenCV gets key strokes as a 4 byte number
	// Common keys (including ESC, value 27) return the 8 bit ASCII code (byte 0)
	// Special keys return an 8 bit value in the third byte (byte 2)
	// - FX keys range from 0x70 (F1) to 0x7B (F12)
	// - Arrows: Left (0x25), Up (0x26), Right (0x27), Down (0x28)
	// - Insert: 0x2D
	// - Del: 0x2E
	// - Home: 0x24
	// - Prev. Page: 0x21
	// - Next Page: 0x22
	// - End: 0x23
	// Special keys return the same code for Ctrl-Shift-Alt combined strokes
	bool parChanged = false;

	int k0 = waitKey(30);
	int k1 = 0xff & k0;
	int k2 = (0xff00 & k0) >> 8;
	int k3 = (0xff0000 & k0) >> 16;
	int k4 = (0xff000000 & k0) >> 24;
	// DEBUG
	if (k0!=-1) {
		// cout << format("Key %02X %02X %02X %02X - %d",k4,k3,k2,k1,k0) << endl;
	}
	switch (k1) {
	case 27:
	case 'q':
	case 'Q':
		Quit = true;
		break;
	case 'g':
	case 'G':
		goProcessing = !goProcessing;
		if (!goProcessing)
			CloseProcessWindow();
		else
			CreateFrameWindows(goProcessing);
		break;
	case 'm':
	case 'M':
		ProcModeActive++;
		if (ProcModeActive>=ProcModNumber)
			ProcModeActive=0;
		DetectionEngine.SetActiveDM(ProcModeActive);
		break;
	case '+':
		break;
	case '-':
		break;
	default:
		break;
	}
	// Active parameter value modification
	// Reduce small step: left/down arrow
	// Increment small step: right/up arrow
	// Reduce big step: Prev. Page
	// Increment big step: Next Page
	// Set to min: Home
	// Set to max: End
	// Set to default: Del
	switch (k3) {
	case 0x25:
	case 0x28:
		ProcModParValue -= ProcModParStep;
		parChanged = true;
		break;
	case 0x26:
	case 0x27:
		ProcModParValue += ProcModParStep;
		parChanged = true;
		break;
	case 0x21:
		ProcModParValue -= ProcModParBigStep;
		parChanged = true;
		break;
	case 0x22:
		ProcModParValue += ProcModParBigStep;
		parChanged = true;
		break;
	case 0x24:
		ProcModParValue = ProcModParMin;
		parChanged = true;
		break;
	case 0x23:
		ProcModParValue = ProcModParMax;
		parChanged = true;
		break;
	case 0x2E:
		ProcModParValue = ProcModParDefault;
		parChanged = true;
		break;
	}
	if (parChanged) {
		// Check value limits and save to parameter
		if (ProcModParValue>ProcModParMax)
			ProcModParValue = ProcModParMax;
		if (ProcModParValue<ProcModParMin)
			ProcModParValue = ProcModParMin;
		DetectionEngine.DetectionMethods[ProcModeActive]->SetParValue(ProcModParActive,ProcModParValue);
	}
	// Check detection method key strokes (function keys)
	for (int i=0; i<ProcModNumber; i++) {
		if (k3==ProcModKeys[i]) {
			DetectionEngine.SetActiveDM(i);
			ProcModeActive = i;
			ProcModParNum = DetectionEngine.GetActiveDMParNumber();
			// cout << "Activated DM " << i << endl;
			break;
		}
	}
	// Check detection method parameters key strokes (number keys)
	for (int i=0; i<DetectionEngine.GetActiveDMParNumber(); i++) {
		if (k1=='1'+(uchar) i)
			SetActiveParameter(i);
	}
	return Quit;
}

// -- Detection method parameters interface -------------------------------------------------------
// The available detection method parameters can be configured on the fly when the video input is
// obtained from a camera. Available parameters depend on the active detection algorithm. Functions
// implemented here allow for the run-time creation of the required track bars.
// When the input is obtained from video, track bars define the parameter values for processing the
// whole frame sequence.

// void onThresholdTrackbar(int value,void* userdata) {
// 	switch (ProcModeActive) {
// 	case PROCMOD_STDTHRESHOLD:
// 		ProcessPar.GrayThreshold=value;
// 		break;
// 	case PROCMOD_MSQI:
// 		ProcessPar.MSQIThreshold=value;
// 		break;
// 	}
// }
// 
// void onRoundnessThresholdTrackbar(int value,void* userdata) {
// 	switch (ProcModeActive) {
// 	case PROCMOD_STDTHRESHOLD:
// 		ProcessPar.ContourMinRoundness=(float) value/100;
// 		break;
// 	case PROCMOD_MSQI:
// 		ProcessPar.MSQISigma=value;
// 		break;
// 	}
// }

// -- Main program --------------------------------------------------------------------------------

int main( int argc, char** argv ) {
    const char* inputFilename = 0;
	int i;
	int cameraId = 0;

	// -- Show application information --------------------
	ShowAppInfo();
	// -- Command line arguments --------------------------
	if (argc < MIN_ARGS || argc > MAX_ARGS) {
        ShowAppHelp();
        return 0;
    }
    for (i = 1; i < argc; i++ ) {
        const char* s = argv[i];
		if (s[0] != '-') {
            if (isdigit(s[0]))
                sscanf(s, "%d", &cameraId);
            else
                inputFilename = s;
        }
		else {
			ShowAppHelp();
			cerr << format("ERROR: Unknown option %s",s) << endl;
			return -1;
		}
    }
	// -- Open video source -------------------------------
    if (inputFilename) {
        Capture.open(inputFilename);
		liveCapture = false;
	}
    else {
        Capture.open(cameraId);
		liveCapture = true;
	}
    if (!Capture.isOpened()) {
		cerr << format("Could not initialize video (%d) capture\n",cameraId) << endl;
		return -2;
	}
	if (liveCapture)
		ShowLiveCaptureHelp();
	else
		ShowVideoFileHelp();
	ShowParKeyHelp();
	// TODO: Change following output to log
	cout << endl;
	cout << "Profiling is applied to frame processing, excluding results presentation" << endl;
	cout << endl;
	cout << "Video source opened" << endl << endl;
	// -- Configure detection engine ----------------------
	ConfigDetectionEngine();
	// -- Create windows ----------------------------------
	CreateFrameWindows(goProcessing);
	// -- Main loop ---------------------------------------
	while (true) {
		if (goProcessing) {
			// If input comes from camera, run continuously
			if (liveCapture) {
				Capture.read(cframe);
				DetectionEngine.Process(cframe);
				DetectionEngine.ShowInfo();
				DetectionEngine.LogInfo();
				ShowControlInfo(cframe);
				imshow(ALP_RESULTSWND,DetectionEngine.GetResultsFrame());
				imshow(ALP_PROCESSWND,DetectionEngine.GetProcessFrame());
			}
			// If input comes from video file, run while there are frames available
			else {
				if (Capture.read(cframe)) {
					DetectionEngine.Process(cframe);
					DetectionEngine.ShowInfo();
					DetectionEngine.LogInfo();
					ShowControlInfo(cframe);
					imshow(ALP_RESULTSWND,DetectionEngine.GetResultsFrame());
					imshow(ALP_PROCESSWND,DetectionEngine.GetProcessFrame());
				}
				// No more frames available
				else {
					goProcessing = false;
				}
			}
		}
		// If processing is not active
		else {
			// If input comes from camera, show it live
			if (liveCapture) {
				Capture.read(cframe);
				ShowControlInfo(cframe);
				imshow(ALP_RESULTSWND,cframe);
			}
		}
		// Check keyboard input
		CheckKeyboard();
		if (Quit)
			break;
	}
	return 0;
}