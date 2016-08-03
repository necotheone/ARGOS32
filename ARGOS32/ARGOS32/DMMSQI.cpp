// DMMSQI.cpp
// Landing Pad detection method based on modified Self-Quotient Image
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   22.09.2013
// Description: This class implements a landing pad detection method based on modified
//              Self-Quotient Image and contour extraction. The captured frame is first converted
//              to gray scale and processed with the modified Self-Quotient Image method to get
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

#include "DMMSQI.h"

// -- Logging library -----------------------------------------------------------------------------

#include "EasyLogging\easylogging++.h"

using namespace cv;

// -- Construction and destruction ----------------------------------------------------------------

CDMMSQI::CDMMSQI(void) {
	Id = DM_MSQI;
	DMName = "Modified Self-Quotient Image";
	DMVersion = "1.0";
	ResetStatistics();
	SetDefaultConfig();
}

CDMMSQI::~CDMMSQI(void) {
}

// -- Detection interface -------------------------------------------------------------------------

Mat CDMMSQI::Process(Mat &frame) {
	unsigned int i;

    CLOG(INFO, Logger()) << format("Processing frame %05d", nFrames+1) << " with " << DMName;
    Profiling.Start(PROFILE_TIMER1);
	// Convert captured frame to gray scale
	cvtColor(frame,gframe,CV_RGB2GRAY);
	// Get binary image from MSQI processing
	bframe = MSQIPreprocessing(gframe,(int) Config.MSQIThreshold,(int) Config.MSQISigma);
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
		mc[i] = Point2f((float) (mu[i].m10/mu[i].m00), (float) (mu[i].m01/mu[i].m00));
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
						//FILE_LOG(logDEBUG2) << msg;
                        CLOG(TRACE, Logger()) << msg;
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
	// Save frames for showing information
	ResultsFrame = frame;
	ProcessFrame = bframe;
	return bframe;
}

void CDMMSQI::CheckKeyboard() {
	// Keyboard input handling
}

void CDMMSQI::ShowInfo() {
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

void CDMMSQI::LogInfo() {
	// Results and information logging on console and file
    LogFrameStatistics();
	LandingPad.ShowLandingPadInfo();
}

void CDMMSQI::ResetStatistics() {
	// Reset detection method statistics variables
	nContours = 0;
	ResetDMStatistics();
	// Reset common statistics
	CDetectionMethod::ResetStatistics();
}

void CDMMSQI::LogFrameStatistics() {
	string msg;

	msg = format("%05d : %3.1f - Contours: %04d - V(%03d) S(%03d) C(%03d) N(%03d) IR(%02d) OR(%02d)",nFrames,TotProcTime*1000.0/nFrames,nContours,nCandidates,nSmall,nNotCircle,nNoHole,nInnerRing,nOuterRing);
    CLOG(INFO, Logger()) << msg;
}

int CDMMSQI::GetParNumber() {
	// Get the number of configuration parameters
	return ConfigNum;
}

float CDMMSQI::GetParValue(int p) {
	// Get the value of parameter p (index)
	float res;
	if (p>=0 && p<DMCFG_MS_PARNUMBER)
		res = Config.Parameters[p];
	else
		res = -1.0;
	return res;
}

string CDMMSQI::GetParName(int p) {
	// Get the name of parameter p (index)
	string res;
	if (p>=0 && p<DMCFG_MS_PARNUMBER)
		res = ConfigNames[p];
	else
		res = "";
	return res;
}

bool CDMMSQI::GetParType(int p) {
	// Get the type of parameter p (index)
	bool res;
	if (p>=0 && p<DMCFG_MS_PARNUMBER)
		res = ConfigTypes[p];
	else
		res = "";
	return res;
}

float CDMMSQI::GetParMax(int p) {
	// Get the maximum allowed value of parameter p (index)
	float res;
	if (p>=0 && p<DMCFG_MS_PARNUMBER)
		res = ConfigMax.Parameters[p];
	else
		res = -1.0;
	return res;
}

float CDMMSQI::GetParMin(int p) {
	// Get the minimum allowed value of parameter p (index)
	float res;
	if (p>=0 && p<DMCFG_MS_PARNUMBER)
		res = ConfigMin.Parameters[p];
	else
		res = -1.0;
	return res;
}

void CDMMSQI::SetParValue(int p, float v) {
	// Set the value for specified parameter 
	Config.Parameters[p] = v;
}

// -- Detection method helper functions -----------------------------------------------------------

void CDMMSQI::SetDefaultConfig() {
	ConfigNum = DMCFG_MS_PARNUMBER;
	// Parameter values
	Config.MSQIThreshold = 20;
	Config.MSQISigma = 9;
	Config.ContourMinArea = 30;
	Config.ContourMinRoundness = 0.83f;
	Config.RingCenterMaxErr = 0.1f;
	// Parameter ranges
	ConfigMin.MSQIThreshold = 1;
	ConfigMax.MSQIThreshold = 255;
	ConfigMin.MSQISigma = 1;
	ConfigMax.MSQISigma = 100;
	ConfigMin.ContourMinArea = 1;
	ConfigMax.ContourMinArea = 1000;
	ConfigMin.ContourMinRoundness = 0.1f;
	ConfigMax.ContourMinRoundness = 0.99f;
	ConfigMin.RingCenterMaxErr = 0.01f;
	ConfigMax.RingCenterMaxErr = 10;
	// Parameter names
	ConfigNames.resize(DMCFG_MS_PARNUMBER);
	ConfigNames[DMCFG_MS_MSQITHRESHOLD] = "Threshold";
	ConfigNames[DMCFG_MS_MSQISIGMA] = "Sigma";
	ConfigNames[DMCFG_MS_CTRMINAREA] = "ContourMinArea";
	ConfigNames[DMCFG_MS_CTRMINROUNDNESS] = "ContourMinRoundness";
	ConfigNames[DMCFG_MS_CTRCENTERMAXERR] = "ContourCenterMaxErr";
	// Parameter types
	ConfigTypes.resize(DMCFG_MS_PARNUMBER);
	ConfigTypes[DMCFG_MS_MSQITHRESHOLD] = false;
	ConfigTypes[DMCFG_MS_MSQISIGMA] = false;
	ConfigTypes[DMCFG_MS_CTRMINAREA] = false;
	ConfigTypes[DMCFG_MS_CTRMINROUNDNESS] = false;
	ConfigTypes[DMCFG_MS_CTRCENTERMAXERR] = false;
}

void CDMMSQI::ResetDMStatistics() {
	nCandidates = 0;
	nSmall = 0;
	nNotCircle = 0;
	nNoHole = 0;
	nInnerRing = 0;
	nOuterRing = 0;
	nRings = 0;
}

void CDMMSQI::ResizeProcessVar(int s) {
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

void CDMMSQI::InitDetectionFlags() {
	for (int i=0; i<nContours; i++) {
		valid[i]=false;
		smallSize[i]=false;
		notcircle[i]=false;
		nohole[i]=false;
		outerring[i]=false;
		innerring[i]=false;
	}
}

Mat  CDMMSQI::Normalize255(const Mat& src) {
	// Normalize a given image into a value range between 0 and 255
	Mat dst;
	switch(src.channels()) {
	case 1:
		cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC1);
		break;
	case 3:
		cv::normalize(src, dst, 0, 255, NORM_MINMAX, CV_8UC3);
		break;
	default:
		src.copyTo(dst);
		break;
	}
	return dst;
}

Mat CDMMSQI::MSQIPreprocessing(InputArray src, int th, int sigma0) {
	// Convert to floating point
	Mat Ilp;
	Mat Ihp;
	Mat Ith;
	//int pv;

	Mat I = src.getMat();
	int Itype = I.type();
	I.convertTo(I, CV_32FC1);
	// Get kernel size from sigma (make it odd for OpenCV)
	int kernel_sz0 = (3*sigma0);
	kernel_sz0 += ((kernel_sz0 % 2) == 0) ? 1 : 0;
	GaussianBlur(I, Ihp, Size(kernel_sz0,kernel_sz0), sigma0, sigma0, BORDER_CONSTANT);
	subtract(I, Ihp, Ilp);
	I = Normalize255(Ilp);
	Scalar Imean = mean(I);
	//double Imin;
	//double Imax;
	//minMaxLoc(I,&Imin,&Imax);
	//cout << format("I   Avg %3d Max %3d Min %3d",(int) Imean[0],(int) Imax,(int) Imin) << endl;
	I.convertTo(I, Itype);
	// 	imshow("pr2",I);
	// 	Ihp = norm_0_255(Ihp);
	Ihp.convertTo(Ihp, Itype);
	int thmin = (int) Imean[0] - th;
	int thmax = (int) Imean[0] + th;
	uchar *ip = (uchar *) I.data;
	int c1 = I.step[0];
	int c2 = I.step[1];
	threshold(I,Ith,(double) thmax,255,THRESH_BINARY);
	// Eliminate isolated pixels
	int morph_elem=0;
	int morph_size=3;
	//Mat element = getStructuringElement(MORPH_RECT,Size(morph_size,morph_size),Point(morph_size,morph_size));
	// Apply the specified morphology operation
	//erode(Ith,Ith,element);
	//morphologyEx(Ith,Ith,MORPH_OPEN,element);
	//filter2D(I,I,I.depth());
	//Ith.create(I.rows,I.cols,CV_8UC3);
	//for(int r = 0; r < I.rows; r++) {
	//	for(int c = 0; c < I.cols; c++) {
	//		pv = (int) ip[r*c1+c*c2];
	//		if (pv>thmax) {
	//			line(Ith,Point(c,r),Point(c,r),Scalar(0,0,255),1,8);
	//		}
	//		else if (pv<thmin) {
	//			line(Ith,Point(c,r),Point(c,r),Scalar(0,255,0),1,8);
	//		}
	//		else {
	//			line(Ith,Point(c,r),Point(c,r),Scalar(255,0,0),1,8);
	//		}
	//	}
	//}
	//Scalar Ihpmean = mean(Ihp);
	//double Ihpmin;
	//double Ihpmax;
	//minMaxLoc(Ihp,&Ihpmin,&Ihpmax);
	//cout << format("Ihp Avg %3d Max %3d Min %3d",(int) Ihpmean[0],(int) Ihpmax,(int) Ihpmin) << endl;
	//imshow("pr2",Ith);
	//normalize(Ihp,I);
	return Ith;
}
