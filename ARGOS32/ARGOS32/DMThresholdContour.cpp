// DMThresholdContour.cpp
// Landing Pad detection method based on thresholding and contour extraction
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   22.09.2013
// Description: This class implements a landing pad detection method based on simple thresholding
//              and contour extraction. The captured frame is first converted to grayscale and
//              equalized to maximize global contrast. The resulting frame is thresholded to get
//              a binary image which is used to generate contours. These contours are filtered
//              out following several criteria (size, roundness, etc.) and finally used to detect
//              rings, markers and the landpad using geometric conditions. Each element (rings,
//              markers and landpad) is assigned a set of confidence measures which can be used
//              later for tracking and classification.
//              See DetectionEngine.cpp for a more thorough description of classes and detection
//              method architecture used.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

// -- Class declarations --------------------------------------------------------------------------

#include "DMThresholdContour.h"
using namespace cv;

// -- Construction and destruction ----------------------------------------------------------------

CDMThresholdContour::CDMThresholdContour(void) {
	Id = DM_THRESHOLD;
	DMName = "Threshold Contour";
	DMVersion = "1.0";
	ResetStatistics();
	SetDefaultConfig();
}

CDMThresholdContour::~CDMThresholdContour(void) {
}

// -- Detection interface -------------------------------------------------------------------------

Mat CDMThresholdContour::Process(Mat &frame) {
	unsigned int i;

	Profiling.Start(PROFILE_TIMER1);
	// Convert captured frame to gray scale
	cvtColor(frame,gframe,CV_RGB2GRAY);
	// Equalize frame to normalize brightness and increase contrast
	equalizeHist(gframe,pframe);
	// Threshold the image
	if (Config.FlagAdaptiveThreshold==0)
		threshold(pframe,bframe,Config.GrayThreshold,255,THRESH_BINARY);
	else
		adaptiveThreshold(pframe,bframe,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,(int) Config.AdaptiveBlockSize,0);
	// Find contours organized in a two-level hierarchy
	findContours(bframe, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	// Reset processing variables based on found contours
	nContours = contours.size();
	ResizeProcessVar(nContours);
	ResetDMStatistics();
	// Initialize detection flags
	InitDetectionFlags();
	// Get the moments, perimeters and roundness of contours and filter out undesired ones
	for (i=0; i<(unsigned int) nContours; i++) {
		// Get contour moments
		mu[i] = moments(contours[i],false);
		// Filter out small contours
		if (mu[i].m00>=Config.ContourMinArea) {
			// Get perimeter and calculate roundness
			perimeter[i] = (float) arcLength(contours[i],true);
			roundness[i] = (float) (4*M_PI*mu[i].m00/(perimeter[i]*perimeter[i]));
			// Filter out low roundness (not circular) contours
			if (roundness[i]>=Config.ContourMinRoundness) {
				// Filter out top level contours without children (holes)
				if TOPLEVEL_NO_HOLE(hierarchy[i]) {	// No parent, no child
					nohole[i]=true;
					nNoHole++;
				}
				else {
					// Only large enough, nearly circular contours with holes if top-level are valid
					valid[i]=true;
					nCandidates++;
				}
			}
			else {
				notcircle[i]=true;
				nNotCircle++;
				// TODO: Report contour selection and get processing statistics
			}
		}
		else {
			smallSize[i]=true;
			nSmall++;
			// TODO: Report contour selection and get processing statistics
		}
	}
	//  Get mass centers
	mc.resize(contours.size());
	for (i=0; i<contours.size(); i++)
		mc[i] = Point2f((float) (mu[i].m10/mu[i].m00),(float) (mu[i].m01/mu[i].m00));
	// Traverse hierarchy to retrieve top-level contours with holes
	LandingPad.Reset();
	for (i=0; i<contours.size(); i++) {
		if (valid[i]) {
			if TOPLEVEL_WITH_HOLE(hierarchy[i]) {
				int ih = hierarchy[i][2];
				do {
					// Check if child contour is valid
					if (valid[ih]) {
						// TODO: Check if parent and child contours share a common mass center
						outerring[i]=true;
						innerring[ih]=true;
						nInnerRing++;
						nOuterRing++;
						nRings++;
						// Store detected ring
						// TODO: Study how to boost performance
						CCircle innercircle(contours[ih],mc[ih], (float) mu[ih].m00,perimeter[ih],roundness[ih]);
						CCircle outercircle(contours[i],mc[i], (float) mu[i].m00,perimeter[i],roundness[i]);
						CMarkerRing MarkerRing(outercircle,innercircle);
						// TODO: Check if ring center is inside the marker area
						// Add the ring to the marker
						bool r = LandingPad.AddRing(MarkerRing);
						// == LOG INFO ===========================================
						string msg = format("RING FOUND %02d (%3.0f,%3.0f) - %02d (%3.0f,%3.0f)",i,mc[i].x,mc[i].y,ih,mc[ih].x,mc[ih].y);
						FILE_LOG(logDEBUG2) << msg;
						// =======================================================
					}
					ih = hierarchy[ih][0];
				} while (ih!=-1);
			}
		}

	}
// TODO: Remove this definition
#ifdef RINGDETECTION_OPTIMIZED
	Profiling.Start(PROFILE_TIMER2);
	LandingPad.OrganizeRings();
	Profiling.Stop(PROFILE_TIMER2);
#endif
	Profiling.Stop(PROFILE_TIMER1);
	nFrames++;
	TotProcTime = TotProcTime+Profiling.GetTime(PROFILE_TIMER1);
	//ShowFrameStatistics();
	// Save frames for showing information
	ResultsFrame = frame;
	ProcessFrame = bframe;
	return bframe;
}

void CDMThresholdContour::CheckKeyboard() {
	// Keyboard input handling
}

void CDMThresholdContour::ShowInfo() {
	// Results and information presentation on results frame
	int i,j;
	// Colors are defined as Scalar(B,G,R)
	Scalar cValid = Scalar(0,255,0);
	Scalar cSmall = Scalar(100,0,0);
	Scalar cNotCircle = Scalar(0,0,100);
	Scalar cContour = Scalar(0,255,255);
	Scalar cContourCenter = Scalar(255,255,255);
	Scalar cOuterRing = Scalar(0,255,0);
	Scalar cInnerRing = Scalar(0,0,255);
	Scalar cMarkerCenter = Scalar(255,255,255);
	Scalar cRing = Scalar(0,0,255);
	Scalar cInfoText = Scalar(250,200,200);
	Mat rframe;
	vector<string> info;

	rframe=ResultsFrame;
	// Draw detected markers
	for (j=0; j<LandingPad.NumMarkers; j++) {
		for (i=0; i<(int) LandingPad.Markers[j].Rings.size(); i++) {
			if (LandingPad.Markers[j].ValidRings[i]) {
				// Draw filled ring
				vector<Point> oc = LandingPad.Markers[j].Rings[i].OuterCircle.Contour;
				vector<Point> ic = LandingPad.Markers[j].Rings[i].InnerCircle.Contour;
				const Point* p[2] = { &oc[0], &ic[0] };
				int npts[2];
				npts[0] = (int) oc.size();
				npts[1] = (int) ic.size();
				fillPoly(rframe,p,npts,2,cRing);
			}
		}
		// Draw marker center
		drawPointMarker(rframe,LandingPad.Markers[j].Position,3,cMarkerCenter);
	}
	// Draw contours
	for (i=0; i<(int) contours.size(); i++) {
		if (valid[i])     drawContours(rframe,contours,i,cValid,1,8);
		if (smallSize[i]) drawContours(rframe,contours,i,cSmall,1,8);
		if (notcircle[i]) drawContours(rframe,contours,i,cNotCircle,1,8);
		if (innerring[i]) drawContours(rframe,contours,i,cInnerRing,1,8);
		if (outerring[i]) drawContours(rframe,contours,i,cOuterRing,1,8);
		//if (innerring[i] || outerring[i]) drawPointMarker(rframe,mc[i],2,cContourCenter);
	}
	info.push_back(format("Detection method: %s %s",DMName.c_str(),DMVersion.c_str()));
	info.push_back(format("Valid contours: %d of %d",nCandidates,nContours));
	info.push_back(format("Detected markers: %d",LandingPad.NumMarkers));
	info.push_back(format("Processing time: %3.0f ms (%2.0f ms)",Profiling.GetTimeMs(PROFILE_TIMER1),Profiling.GetTimeMs(PROFILE_TIMER2)));
	drawInfoText(rframe,0,info,cInfoText);
}

void CDMThresholdContour::LogInfo() {
	// Results and information logging on console and file
	LandingPad.ShowLandingPadInfo();
}

void CDMThresholdContour::ResetStatistics() {
	// Reset detection method statistics variables
	nContours = 0;
	ResetDMStatistics();
	// Reset common statistics
	CDetectionMethod::ResetStatistics();
}

void CDMThresholdContour::LogFrameStatistics() {
	string msg;

	msg = format("%05d : %3.1f - Contours: %04d - V(%03d) S(%03d) C(%03d) N(%03d) IR(%02d) OR(%02d)",nFrames,TotProcTime*1000.0/nFrames,nContours,nCandidates,nSmall,nNotCircle,nNoHole,nInnerRing,nOuterRing);
	FILE_LOG(logINFO) << msg;
}

int CDMThresholdContour::GetParNumber() {
	// Get the number of configuration parameters
	return ConfigNum;
}

float CDMThresholdContour::GetParValue(int p) {
	// Get the value of parameter p (index)
	float res;
	if (p>=0 && p<DMCFG_TC_PARNUMBER)
		res = Config.Parameters[p];
	else
		res = -1.0;
	return res;
}

string CDMThresholdContour::GetParName(int p) {
	// Get the name of parameter p (index)
	string res;
	if (p>=0 && p<DMCFG_TC_PARNUMBER)
		res = ConfigNames[p];
	else
		res = "";
	return res;
}

bool CDMThresholdContour::GetParType(int p) {
	// Get the type of parameter p (index)
	bool res;
	if (p>=0 && p<DMCFG_TC_PARNUMBER)
		res = ConfigTypes[p];
	else
		res = "";
	return res;
}

float CDMThresholdContour::GetParMax(int p) {
	// Get the maximum allowed value of parameter p (index)
	float res;
	if (p>=0 && p<DMCFG_TC_PARNUMBER)
		res = ConfigMax.Parameters[p];
	else
		res = -1.0;
	return res;
}

float CDMThresholdContour::GetParMin(int p) {
	// Get the minimum allowed value of parameter p (index)
	float res;
	if (p>=0 && p<DMCFG_TC_PARNUMBER)
		res = ConfigMin.Parameters[p];
	else
		res = -1.0;
	return res;
}

void CDMThresholdContour::SetParValue(int p, float v) {
	// Set the value for specified parameter 
	Config.Parameters[p] = v;
}

// -- Detection method helper functions -----------------------------------------------------------

void CDMThresholdContour::SetDefaultConfig() {
	ConfigNum = DMCFG_TC_PARNUMBER;
	// Parameter values
	Config.GrayThreshold = 200;
	Config.FlagAdaptiveThreshold = 0;
	Config.AdaptiveBlockSize = 100;
	Config.ContourMinArea = 25;
	Config.ContourMinRoundness = 0.83f;
	Config.RingCenterMaxErr = 0.1f;
	// Parameter ranges
	ConfigMin.GrayThreshold = 1;
	ConfigMax.GrayThreshold = 255;
	ConfigMin.FlagAdaptiveThreshold = 0;
	ConfigMax.FlagAdaptiveThreshold = 1;
	ConfigMin.AdaptiveBlockSize = 1;
	ConfigMax.AdaptiveBlockSize = 300;
	ConfigMin.ContourMinArea = 1;
	ConfigMax.ContourMinArea = 1000;
	ConfigMin.ContourMinRoundness = 0.1f;
	ConfigMax.ContourMinRoundness = 0.99f;
	ConfigMin.RingCenterMaxErr = 0.01f;
	ConfigMax.RingCenterMaxErr = 10;
	// Parameter names
	ConfigNames.resize(DMCFG_TC_PARNUMBER);
	ConfigNames[DMCFG_TC_GRAYTHRESHOLD] = "Threshold";
	ConfigNames[DMCFG_TC_FLAGADAPTIVETHR] = "FlagAdaptiveThreshold";
	ConfigNames[DMCFG_TC_ADAPTIVETHRSIZE] = "AdaptiveThresholdSize";
	ConfigNames[DMCFG_TC_CTRMINAREA] = "ContourMinArea";
	ConfigNames[DMCFG_TC_CTRMINROUNDNESS] = "ContourMinRoundness";
	ConfigNames[DMCFG_TC_CTRCENTERMAXERR] = "ContourCenterMaxErr";
	// Parameter types
	ConfigTypes.resize(DMCFG_TC_PARNUMBER);
	ConfigTypes[DMCFG_TC_GRAYTHRESHOLD] = false;
	ConfigTypes[DMCFG_TC_FLAGADAPTIVETHR] = true;
	ConfigTypes[DMCFG_TC_ADAPTIVETHRSIZE] = false;
	ConfigTypes[DMCFG_TC_CTRMINAREA] = false;
	ConfigTypes[DMCFG_TC_CTRMINROUNDNESS] = false;
	ConfigTypes[DMCFG_TC_CTRCENTERMAXERR] = false;
}

void CDMThresholdContour::ResetDMStatistics() {
	nCandidates = 0;
	nSmall = 0;
	nNotCircle = 0;
	nNoHole = 0;
	nInnerRing = 0;
	nOuterRing = 0;
	nRings = 0;
}

void CDMThresholdContour::ResizeProcessVar(int s) {
	mu.resize(s);
	perimeter.resize(s);
	roundness.resize(s);
	valid.resize(s);
	smallSize.resize(s);
	notcircle.resize(s);
	nohole.resize(s);
	innerring.resize(s);
	outerring.resize(s);
}

void CDMThresholdContour::InitDetectionFlags() {
	for (int i=0; i<nContours; i++) {
		valid[i]=false;
		smallSize[i]=false;
		notcircle[i]=false;
		nohole[i]=false;
		outerring[i]=false;
		innerring[i]=false;
	}
}
