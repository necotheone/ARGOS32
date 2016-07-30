// ARGOSLandingPad.cpp
// Landing Pad marker detection
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   02.09.2013
// Description: This software defines the vision-based landing pad recognition algorithms which
//              will be used later for automatic landing of ARGOS vehicles with several goals,
//              including their ability to land and stow automatically to their charge base.
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

#include "Definitions.h"
#include "Profiling.h"
#include "Marker.h"
//#include "Retinex.h"

// -- Namespaces ----------------------------------------------------------------------------------

using namespace cv;
using namespace std;

// -- Console help --------------------------------------------------------------------------------

const char* liveCaptureHelp =
    "When the live video from camera is used as input, the following hot-keys may be used:\n"
        "  <ESC>, q - quit the program\n"
        "  g        - start/stop processing images\n"
		"  m        - change processing mode\n";

static void help() {
    printf( "ARGOSLandingPad\n"
		"Software for testing ARGOS Landing Pad detection\n"
		"\n"
        "Usage: ARGOSLandingPad\n"
        "     [input_data]             # name of video file to be processed, or number of camera to use\n"
        "                              # if not specified, a live view from the default camera is used\n"
        "\n" );
    printf( "\n%s", liveCaptureHelp );
}

// -- Processing variables ------------------------------------------------------------------------
// TODO: Several global variables are defined to track the process; final implementation
//       can use local variables to get more memory if required.

#define PROCMOD_STDTHRESHOLD 0
#define PROCMOD_MSQI         1
#define PROCMOD_MODESNUM     2			// Defines the number of available modes for cycling through keyboard

int ProcessingMode = PROCMOD_STDTHRESHOLD;
unsigned char ProcModNames[PROCMOD_MODESNUM][20] = {"Std Threshold","MSQI"};

#define MSQI_THRESHOLD 10

Mat cameraFrame;					// Frame captured from camera/file
Mat pframe;							// Preprocessing resultant frame
Mat gframe;							// Grayscale converted frame
Mat bframe;							// Binary frame obtained by thresholding grayscale frame
Mat bf2;							// Copy of binary frame to be shown

vector<vector<Point> > contours;	// Set of contours found int the binary frame
vector<Vec4i>          hierarchy;	// Hierarchy of contours
vector<Moments>        mu;			// Contour moments for area and length calculations
vector<Point2f>        mc;			// Contour mass centers
vector<float>          perimeter;	// Contour perimeters
vector<float>          roundness;	// Contour roundness
vector<bool>           valid;		// Indicates if contour is valid (detected as circle)
vector<bool>           small;		// Indicates if contour is too small
vector<bool>           notcircle;   // Indicates if contour roundness is enough
vector<bool>           nohole;		// Indicates if contour has no child
vector<bool>           outerring;	// Indicates if contour is outer ring circle
vector<bool>           innerring;	// Indicates if contour is inner ring vircle
vector<vector<Point>>  vContours;	// Set of contours after filtering out
vector<Vec4i>          vHierarchy;	// Contour hierarchy after filtering out
vector<Moments>        vMu;			// Contour moments after filtering out
vector<float>          vPerimeter;	// Contour perimeters after filtering out
vector<float>          vRoundness;	// Contour roundness after filtering out

struct tStatistics {
	long  nFrames;					// Total number of processed frames
	float TotProcTime;				// Total processing time
	int   nContours;				// Total number of contours found
	int   nCandidates;				// Number of valid contour candidates
	int   nSmall;					// Number of small rejected contours
	int   nNotCircle;				// Number of not circular rejected contours
	int   nNoHole;					// Number of top-level without childs rejected contours
	int   nOuterRing;				// Number of outer ring circles detected
	int   nInnerRing;				// Number of inner ring circles detected
	int   nRings;					// Number of rings detected
} Statistics;

void ResetFrameStatistics() {
	Statistics.nCandidates=0;
	Statistics.nContours=0;
	Statistics.nSmall=0;
	Statistics.nNotCircle=0;
	Statistics.nNoHole=0;
	Statistics.nInnerRing=0;
	Statistics.nOuterRing=0;
	Statistics.nRings=0;
}

void ResetStatistics() {
	Statistics.nFrames=0;
	Statistics.TotProcTime=0.0;
	ResetFrameStatistics();
}

void ShowFrameStatistics() {
	string msg;

	msg = format("%05d : %3.1f - Contours: %04d - V(%03d) S(%03d) C(%03d) N(%03d) IR(%02d) OR(%02d)",Statistics.nFrames,Statistics.TotProcTime*1000.0/Statistics.nFrames,Statistics.nContours,Statistics.nCandidates,Statistics.nSmall,Statistics.nNotCircle,Statistics.nNoHole,Statistics.nInnerRing,Statistics.nOuterRing);
	cout << msg << endl;
}

CLandingPad LandingPad;

struct tProcessPar {				// Structure with all required parameters for processing
	int   GrayThreshold;			// Threshold used for generating binary frame from grayscale (standard threshold mode)
	int   MSQIThreshold;			// Threshold used for generating binary image from SQI image (Modified SQI mode)
	int   MSQISigma;				// Sigma value (kernel size) for high-pass Gaussian filtering in MSQI algorithm
	float ContourMinArea;			// Threshold used for removing small details from processing
	float ContourMinRoundness;		// Threshold used for removing contours with low roundness
	int   MarkerRingNum;			// Number of concentric rings in marker
	vector<float> RingRadius;		// List of outer ring radius for each marker ring
	vector<float> RingRadiusRatio;	// List of ring radius ratios
	float RingCenterMaxErr;			// Maximum error (ratio to outer radius) between inner and outer centers
} ProcessPar;

void SetDefaultPar() {
	ProcessPar.GrayThreshold = 200;
	ProcessPar.MSQIThreshold = 3;
	ProcessPar.MSQISigma = 2;
	ProcessPar.ContourMinArea = 25;
	ProcessPar.ContourMinRoundness = 0.83;
	ProcessPar.MarkerRingNum = 4;
	ProcessPar.RingRadius.resize(ProcessPar.MarkerRingNum);
	ProcessPar.RingRadiusRatio.resize(ProcessPar.MarkerRingNum);
	ProcessPar.RingRadius[0] = 180.0;
	ProcessPar.RingRadius[1] = 120.0;
	ProcessPar.RingRadius[2] = 72.0;
	ProcessPar.RingRadius[3] = 30.0;
	ProcessPar.RingRadiusRatio[0] = 0.85;
	ProcessPar.RingRadiusRatio[1] = 0.75;
	ProcessPar.RingRadiusRatio[2] = 0.65;
	ProcessPar.RingRadiusRatio[3] = 0.50;
	ProcessPar.RingCenterMaxErr = 0.1;
}

// -- Processing ----------------------------------------------------------------------------------
// NOTES
// * First processing steps will prepare the frame for detecting circles, and will clean it up
//   using processing parameters (mainly eliminating detected contours with small area).
// * Tests will show if some additional preprocessing is required.
// * Anular marks detection is done through fitting ellipses to detected contours. Other possible
//   processing, as convexity checks, is not used to allow for partially occluded mark recognition.

// Preprocessing tasks:
// - Equalize frame to get the best possible contrast

void Preprocess(Mat &frame) {
	// Equalize frame to normalize brightness and increase contrast
	equalizeHist(frame,pframe);
}

// == Tan-Triggs Preprocessing ==============================
#ifdef PROCESSING_TANTRIGGS

// Normalizes a given image into a value range between 0 and 255.
Mat norm_0_255(const Mat& src) {
	// Create and return normalized image
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

Mat TanTriggsPreprocessing(InputArray src, float alpha = 0.1, float tau = 10.0, float gamma = 0.2, int sigma0 = 1, int sigma1 = 2) {
	// Convert to floating point
	Mat X = src.getMat();
	X.convertTo(X, CV_32FC1);
	// Start preprocessing
	Mat I;
	pow(X, gamma, I);
	imshow("pr2",I);
	// Calculate the DOG Image
	Mat gaussian0, gaussian1;
	// Kernel Size:
	int kernel_sz0 = (3*sigma0);
	int kernel_sz1 = (3*sigma1);
	// Make them odd for OpenCV
	kernel_sz0 += ((kernel_sz0 % 2) == 0) ? 1 : 0;
	kernel_sz1 += ((kernel_sz1 % 2) == 0) ? 1 : 0;
	GaussianBlur(I, gaussian0, Size(kernel_sz0,kernel_sz0), sigma0, sigma0, BORDER_CONSTANT);
	GaussianBlur(I, gaussian1, Size(kernel_sz1,kernel_sz1), sigma1, sigma1, BORDER_CONSTANT);
	subtract(gaussian0, gaussian1, I);
	imshow("pr2",I);
	double meanI = 0.0;
	Mat tmp;
	pow(abs(I), alpha, tmp);
	meanI = mean(tmp).val[0];
	I = I / pow(meanI, 1.0/alpha);
	//imshow("pr2",I);
	pow(min(abs(I), tau), alpha, tmp);
	meanI = mean(tmp).val[0];
	I = I / pow(meanI, 1.0/alpha);
	//imshow("pr2",I);
	// Squash into the tanh
	for(int r = 0; r < I.rows; r++) {
		for(int c = 0; c < I.cols; c++) {
			I.at<float>(r,c) = tanh(I.at<float>(r,c) / tau);
		}
	}
	I = tau * I;
	I = norm_0_255(I);
	I.convertTo(I, CV_8UC3);
	return I;
}

#endif
// ==========================================================

// == Tan-Triggs Preprocessing ==============================
#ifdef PROCESSING_SELFQUOTIENT

Mat SQIPreprocessing(InputArray src, float alpha = 0.1, float tau = 10.0, float gamma = 0.2, int sigma0 = 1) {
	// Convert to floating point
	Mat I = src.getMat();
	I.convertTo(I, CV_32FC1);
	// Start preprocessing
	Mat Ilp;
	pow(I, gamma, I);
	//imshow("pr2",I);
	Mat Ihp;
	int kernel_sz0 = (3*sigma0);
	// Make them odd for OpenCV
	kernel_sz0 += ((kernel_sz0 % 2) == 0) ? 1 : 0;
	GaussianBlur(I, Ihp, Size(kernel_sz0,kernel_sz0), sigma0, sigma0, BORDER_CONSTANT);
	subtract(Ihp, I, Ilp);
	imshow("pr2",Ilp);
	for(int r = 0; r < I.rows; r++) {
		for(int c = 0; c < I.cols; c++) {
			I.at<float>(r,c) = Ihp.at<float>(r,c) / Ilp.at<float>(r,c);
		}
	}
	//I = tau * I;
	I.convertTo(I, CV_8UC3);
	return I;
}

#endif

// == Tan-Triggs Preprocessing ==============================
#ifdef PROCESSING_MODSELFQUOTIENT

// Normalizes a given image into a value range between 0 and 255.
Mat norm_0_255(const Mat& src) {
	// Create and return normalized image
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

// Modified Selft-Quotient Image Algorithm
// Image is required to be grayscale, and processing consists of applying a spatial high-pass filter
// using a gaussian kernel. This high-pass image is substracted from the original image to give a
// low-pass filtered image, which is finally normalized to the [0,255] interval.
// The resulting image has a constant gray level for smooth areas, with high and low values in areas
// of intensity changes.
// The average image intensity is obtained averaging pixel values, and is used as a reference for
// thresholding, which generates the binary image to be used for contour detection.
// The threshold value is therefore automatically extracted from the low-pass filtered image, and
// this makes the processing invariant to lighting changes.

Mat MSQIPreprocessing(InputArray src, int th=3, int sigma0=1) {
	// Convert to floating point
	Mat Ilp;
	Mat Ihp;
	Mat I = src.getMat();
	int Itype = I.type();
	I.convertTo(I, CV_32FC1);
	// Start preprocessing
	int kernel_sz0 = (3*sigma0);
	// Make them odd for OpenCV
	kernel_sz0 += ((kernel_sz0 % 2) == 0) ? 1 : 0;
	GaussianBlur(I, Ihp, Size(kernel_sz0,kernel_sz0), sigma0, sigma0, BORDER_CONSTANT);
	subtract(I, Ihp, Ilp);
	I = norm_0_255(Ilp);
	Scalar Imean = mean(I);
	//double Imin;
	//double Imax;
	//minMaxLoc(I,&Imin,&Imax);
	//cout << format("I   Avg %3d Max %3d Min %3d",(int) Imean[0],(int) Imax,(int) Imin) << endl;
	I.convertTo(I, Itype);
	imshow("pr2",I);
	Ihp = norm_0_255(Ihp);
	Ihp.convertTo(Ihp, Itype);
	int thmin = (int) Imean[0] - th;
	int thmax = (int) Imean[0] + th;
	int pv;
	uchar *ip = (uchar *) I.data;
	int c1 = I.step[0];
	int c2 = I.step[1];
	Mat Ith;
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

#endif

// ==========================================================

#ifdef PROCESSING_OPTIMIZED

vector<int>         tparent;					// Contour hierarchy tree parent index
vector<int>         tchildnum;					// Contour hierarchy tree child number
vector<vector<int>> tchildren;					// COntour hierarchy tree children index list
vector<int>         rCandidateMarkerIdx;		// Index of ring parent landmark candidate
vector<int>         rInnerContourIdx;			// Index of ring inner contour
vector<int>         rOuterContourIdx;			// Index of ring outer contour
vector<bool>        rContourInRing;				// Indicates if contour is already part of a ring

void CreateLandmarkCandidates(int lmCandidateIdx, int parentIdx) {
	// Create a ring if conditions are met
	// Two contours create a ring if parent contour has only one child inside
	// Contours assigned to previous rings as child (inner) cannot be parent of other ring
	if (tchildnum[parentIdx]==1 && rContourInRing[parentIdx]==false) {
		rCandidateMarkerIdx.push_back(lmCandidateIdx);
		rOuterContourIdx.push_back(parentIdx);
		rInnerContourIdx.push_back(tchildren[parentIdx][0]);
		// Set statistical information
		innerring[tchildren[parentIdx][0]]=true;
		outerring[parentIdx]=true;
		Statistics.nInnerRing++;
		Statistics.nOuterRing++;
		Statistics.nRings++;
	}
	// Check childs with recursion
	for (int i=0; i<tchildren[parentIdx].size(); i++) {
		int Cc = tchildren[parentIdx][i];
		if (tchildnum[parentIdx]>0)
			CreateLandmarkCandidates(lmCandidateIdx,Cc);
	}
}
#endif

void ProcessFrame(Mat &frame) {
	unsigned int i,j;
	// Convert captured frame to grayscale
	ProfilingStart(PROFILE_TIMER1);
	cvtColor(frame,gframe,CV_RGB2GRAY);
	// Preprocess to obtain binary image for subsequent contour detection
	switch (ProcessingMode) {
	case PROCMOD_STDTHRESHOLD:
		// Preprocessing
		Preprocess(gframe);
		// Threshold the image
		threshold(pframe,bframe,ProcessPar.GrayThreshold,255,THRESH_BINARY);
		bf2=bframe;
		break;
	case PROCMOD_MSQI:
		gframe = MSQIPreprocessing(gframe,ProcessPar.MSQIThreshold,ProcessPar.MSQISigma);
		bframe = gframe.clone();
		pframe = gframe.clone();
		break;
	default:
		break;
	}
#ifdef PROCESSING_OPTIMIZED
	// TODO: Move these declarations to a more appropriate location
	vector<int>           fIdx;
	vector<vector<Point>> fcontours;
	vector<float>         farea;
	vector<int>           gIdx;
	vector<vector<Point>> gcontours;
	vector<float>         garea;
	// Find contours without creating a hierarchy
	findContours(bframe, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
	// Initialize processing statistics and detection flags
	ResetFrameStatistics();
	Statistics.nContours=contours.size();
	valid.resize(contours.size());
	small.resize(contours.size());
	notcircle.resize(contours.size());
	nohole.resize(contours.size());
	innerring.resize(contours.size());
	outerring.resize(contours.size());
	for (i=0; i<contours.size(); i++) {
		valid[i]=false;
		small[i]=false;
		notcircle[i]=false;
		nohole[i]=false;
		outerring[i]=false;
		innerring[i]=false;
	}
	// Filter out small contours
	mu.resize(contours.size());
	for (i=0; i<contours.size(); i++) {
		// Get contour moments
		mu[i] = moments(contours[i],false);
		// Filter out small contours
		if (mu[i].m00<ProcessPar.ContourMinArea) {
			small[i]=true;
			Statistics.nSmall++;
		}
		else {
			fIdx.push_back(i);
			fcontours.push_back(contours[i]);
			farea.push_back(mu[i].m00);
		}
	}
	// Filter out not round contours
	perimeter.resize(fcontours.size());
	roundness.resize(fcontours.size());
	for (i=0; i<fcontours.size(); i++) {
		// Get perimeter and calculate roundness
		perimeter[i]=arcLength(fcontours[i],true);
		roundness[i]=4*M_PI*farea[i]/(perimeter[i]*perimeter[i]);
		// Filter out low roundness (not circular) contours
		if (roundness[i]<ProcessPar.ContourMinRoundness) {
			notcircle[fIdx[i]]=true;
			Statistics.nNotCircle++;
		}
		else {
			gIdx.push_back(i);
			gcontours.push_back(fcontours[i]);
			garea.push_back(farea[i]);
			valid[fIdx[i]]=true;
			Statistics.nCandidates++;
		}
	}
	/*
	// Build up hierarchy of accepted contours
	vector<int>         levels;
	vector<vector<int>> childs;		// Contours contained inside [index]
	vector<vector<int>> parents;
	vector<vector<int>> levelIdx;	// Indexes of contours in [index] level
	levels.resize(gcontours.size());
	childs.resize(gcontours.size());
	parents.resize(gcontours.size());
	// Initially all contours are top-level (no parent, no child)
	for (i=0; i<gcontours.size(); i++)
		levels[i]=0;
	// Get the nest level of each contour
	for (i=0; i<gcontours.size(); i++) {
		for (j=0; j<gcontours.size(); j++) {
			if (i!=j) {
				int tInside = (int) pointPolygonTest(gcontours[j],gcontours[i][0],false);
				if (tInside>0) {
					levels[i]++;
					// Contour i is inside contour j
					childs[j].push_back(i);
					// Contour j is predecessor of contour i
					parents[i].push_back(j);
				}
			}
		}
		// Update indexes of contours by level
		if (levels[i]>=levelIdx.size())
			levelIdx.resize(levels[i]+1);
		levelIdx[levels[i]].push_back(i);
	}
	// Create contour tree
	// Each contour is assigned 3 indexes:
	// Contour level (already done in previous step)
	// Direct parent contour index
	// Direct children contour index list
	tparent.resize(gcontours.size());
	tchildnum.resize(gcontours.size());
	tchildren.resize(gcontours.size());
	for (i=0; i<gcontours.size(); i++) {
		tparent[i] = -1;
		tchildnum[i] = 0;
		tchildren[i].clear();
	}
	// Check all levels except the last one (contours without more childs)
	for (i=0; i<levelIdx.size()-1; i++) {
		// Check all contours in current level
		for (j=0; j<levelIdx[i].size(); j++) {
			// Select current contour
			int Cp = levelIdx[i][j];
			// Check contours in the next level inside current contour
			for (int k=0; k<levelIdx[i+1].size(); k++) {
				int Cc = levelIdx[i+1][k];
				// Check if contour levelIdx[i+1][k] is child of contour levelIdx[i][j]
				for (int n=0; n<childs[Cp].size(); n++) {
					if (childs[Cp][n]==Cc) {
						tchildnum[Cp]++;
						tchildren[Cp].push_back(Cc);
						// TODO: Remove this check when verified
						if (tparent[Cc]!=-1)
							cout << "WARNING: this child contour has a parent already" << endl;
						tparent[Cc]=Cp;
					}
				}
			}
		}
	}
	// Create landpad candidates for each top-level contour with at least one child
	int CandidateMarkersNum = 0;
	vector<int> CandidateTopLevelContourIdx;
	vector<CMarker> CandidateMarkers;
	for (i=0; i<levelIdx[0].size(); i++) {
		int Cp = levelIdx[0][i];
		if (tchildnum[Cp]>1) {
			CandidateMarkersNum++;
			CandidateTopLevelContourIdx.push_back(Cp);
		}
	}
	CandidateMarkers.resize(CandidateMarkersNum);
	// Create ring candidates for each landmark candidate (recursive procedure)
	rContourInRing.resize(gcontours.size());
	for (i=0; i<CandidateMarkersNum; i++) {
		// Set initial contour (landmark top-level parent contour)
		int Cp = levelIdx[0][j];
		CreateLandmarkCandidates(i,Cp);
	}
	//  Get mass centers
	mc.resize(gcontours.size());
	for (i=0; i<gcontours.size(); i++) {
			mc[i] = Point2f(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00);
	}
	// Create markers
	for (i=0; i<rCandidateMarkerIdx.size(); i++) {
		int icIdx = rInnerContourIdx[i];
		int ocIdx = rOuterContourIdx[i];
		int lmIdx = rCandidateMarkerIdx[i];
		int icgIdx = gIdx[icIdx];
		int ocgIdx = gIdx[ocIdx];
		CCircle innercircle(gcontours[icIdx],mc[icIdx],mu[icgIdx].m00,perimeter[icgIdx],roundness[icgIdx]);
		CCircle outercircle(gcontours[ocIdx],mc[ocIdx],mu[ocgIdx].m00,perimeter[ocgIdx],roundness[ocgIdx]);
		CMarkerRing MarkerRing(outercircle,innercircle);
		// TODO: Check if ring center is inside the marker area
		// TODO: Handle more than one marker
		// Add the ring to the marker
		bool r = CandidateMarkers[lmIdx].AddRing(MarkerRing);
	}
	*/
#else
	// Find contours organized in a two-level hierarchy
	findContours(bframe, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	// Get the moments, perimeters and roundness of contours and filter out undesired ones
	mu.resize(contours.size());
	perimeter.resize(contours.size());
	roundness.resize(contours.size());
	valid.resize(contours.size());
	small.resize(contours.size());
	notcircle.resize(contours.size());
	nohole.resize(contours.size());
	innerring.resize(contours.size());
	outerring.resize(contours.size());
	ResetFrameStatistics();
	Statistics.nContours=contours.size();
	for (i=0; i<contours.size(); i++) {
		// Initialize detection flags
		valid[i]=false;
		small[i]=false;
		notcircle[i]=false;
		nohole[i]=false;
		outerring[i]=false;
		innerring[i]=false;
		// Get contour moments
		mu[i] = moments(contours[i],false);
		// Filter out small contours
		if (mu[i].m00>=ProcessPar.ContourMinArea) {
			// Get perimeter and calculate roundness
			perimeter[i]=arcLength(contours[i],true);
			roundness[i]=4*M_PI*mu[i].m00/(perimeter[i]*perimeter[i]);
			// Filter out low roundness (not circular) contours
			if (roundness[i]>=ProcessPar.ContourMinRoundness) {
				// Filter out top level contours without childs (holes)
				if TOPLEVEL_NO_HOLE(hierarchy[i]) {	// No parent, no child
					nohole[i]=true;
					Statistics.nNoHole++;
				}
				else {
					valid[i]=true;
					Statistics.nCandidates++;
				}
			}
			else {
				notcircle[i]=true;
				Statistics.nNotCircle++;
				// TODO: Report contour selection and get processing statistics
			}
		}
		else {
			small[i]=true;
			Statistics.nSmall++;
			// TODO: Report contour selection and get processing statistics
		}
	}
	//  Get mass centers
	mc.resize(contours.size());
	for (i=0; i<contours.size(); i++) {
		mc[i] = Point2f(mu[i].m10/mu[i].m00,mu[i].m01/mu[i].m00);
	}
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
						Statistics.nInnerRing++;
						Statistics.nOuterRing++;
						Statistics.nRings++;
						// Store detected ring
						// TODO: Study how to boost performance
						CCircle innercircle(contours[ih],mc[ih],mu[ih].m00,perimeter[ih],roundness[ih]);
						CCircle outercircle(contours[i],mc[i],mu[i].m00,perimeter[i],roundness[i]);
						CMarkerRing MarkerRing(outercircle,innercircle);
						// TODO: Check if ring center is inside the marker area
						// TODO: Handle more than one marker
						// Add the ring to the marker
						bool r = LandingPad.AddRing(MarkerRing);
						//cout << "RING FOUND ";
						//cout << i << " (" << mc[i].x << "," << mc[i].y << ") - ";
						//cout << ih << " (" << mc[ih].x << "," << mc[ih].y << ")" << endl;
					}
					ih = hierarchy[ih][0];
				} while (ih!=-1);
			}
		}
	}
#ifdef RINGDETECTION_OPTIMIZED
	ProfilingStart(PROFILE_TIMER2);
	LandingPad.OrganizeRings();
	ProfilingStop(PROFILE_TIMER2);
#endif
#endif
	ProfilingStop(PROFILE_TIMER1);

	Statistics.nFrames++;
	Statistics.TotProcTime = Statistics.TotProcTime+ProfilingGetTime(PROFILE_TIMER1);
	//ShowFrameStatistics();
	LandingPad.ShowLandingPadInfo();
}

// -- Display and parameter interface -------------------------------------------------------------

void onThresholdTrackbar(int value,void* userdata) {
	switch (ProcessingMode) {
	case PROCMOD_STDTHRESHOLD:
		ProcessPar.GrayThreshold=value;
		break;
	case PROCMOD_MSQI:
		ProcessPar.MSQIThreshold=value;
		break;
	}
}

void onRoundnessThresholdTrackbar(int value,void* userdata) {
	switch (ProcessingMode) {
	case PROCMOD_STDTHRESHOLD:
		ProcessPar.ContourMinRoundness=(float) value/100;
		break;
	case PROCMOD_MSQI:
		ProcessPar.MSQISigma=value;
		break;
	}
}

void drawCenter(Mat &f,Point c,int s,Scalar &color,int thickness=1,int type=8,int shift=0) {
	Point c1(c.x,c.y-s);
	Point c2(c.x,c.y+s);
	Point c3(c.x-s,c.y);
	Point c4(c.x+s,c.y);
	line(f,c1,c2,color,thickness,type,shift);
	line(f,c3,c4,color,thickness,type,shift);
}

void ShowResults() {
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
	Mat rframe;
	string lbl;

	rframe=cameraFrame;
	// Draw detected markers
	for (j=0; j<LandingPad.NumMarkers; j++) {
		for (i=0; i<LandingPad.Markers[j].Rings.size(); i++) {
			if (LandingPad.Markers[j].ValidRings[i]) {
				vector<Point> oc = LandingPad.Markers[j].Rings[i].OuterCircle.Contour;
				vector<Point> ic = LandingPad.Markers[j].Rings[i].InnerCircle.Contour;
				const Point* p[2] = { &oc[0], &ic[0] };
				int npts[2];
				npts[0] = (int) oc.size();
				npts[1] = (int) ic.size();
				fillPoly(rframe,p,npts,2,cRing);
			}
		}
		drawCenter(rframe,LandingPad.Markers[j].Position,3,cMarkerCenter);
	}
	// Draw contours
	for (i=0; i<contours.size(); i++) {
		if (valid[i])     drawContours(rframe,contours,i,cValid,1,8);
		if (small[i])     drawContours(rframe,contours,i,cSmall,1,8);
		if (notcircle[i]) drawContours(rframe,contours,i,cNotCircle,1,8);
		if (innerring[i]) drawContours(rframe,contours,i,cInnerRing,1,8);
		if (outerring[i]) drawContours(rframe,contours,i,cOuterRing,1,8);
		//if (innerring[i] || outerring[i]) drawCenter(rframe,mc[i],2,cContourCenter);
	}
	lbl = format("Processing mode: %s",ProcModNames[ProcessingMode]);
	putText(rframe,lbl,Point(20,20),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,200,200), 1, CV_AA);
	lbl = format("Candidates: %d of %d",Statistics.nCandidates,Statistics.nContours);
	putText(rframe,lbl,Point(20,35),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,200,200), 1, CV_AA);
	lbl = format("Processing time: %3.0f ms (%2.0f ms)",ProfilingGetTimeMs(PROFILE_TIMER1),ProfilingGetTimeMs(PROFILE_TIMER2));
	putText(rframe,lbl,Point(20,50),FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(250,200,200), 1, CV_AA);
	imshow("cam",rframe);
	imshow("pr1",pframe);
}

// -- Main program --------------------------------------------------------------------------------

int main( int argc, char** argv ) {
    const char* outputFilename = "out_camera_data.yml";
    const char* inputFilename = 0;
	int i;
	bool quit = false;
	bool liveCapture = false;
	bool goprocessing = false;
    VideoCapture capture;
	int cameraId = 0;

	// Retrieve command line arguments
	// TODO: Add command line parameter to define processing parameters input file
	if (argc < MIN_ARGS || argc > MAX_ARGS) {
        help();
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
        else
            return fprintf( stderr, "Unknown option %s", s ), -1;
    }
	// Open video source (file or camera)
    if (inputFilename) {
        capture.open(inputFilename);
	}
    else {
        capture.open(cameraId);
	}
    if (!capture.isOpened())
        return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
    cout << format( "%s", liveCaptureHelp ) << endl;
	cout << "Profiling is applied to frame processing, excluding results presentation" << endl;
	cout << endl;
	cout << "Video source opened" << endl << endl;
	// TEST ==========================
	// Marker metrics
	cout << "Testing marker type metrics" << endl;
	CMarkerType mt;
	float mm1t = 0;
	float mm1;
	string ms1 = "Cr [ ";
	string ms2 = "Dr [ ";
	string ms3 = "Mr [ ";
	for (int n=0; n<mt.NumTypes; n++) {
		for (int r=0; r<mt.RingNum[n]; r++) {
			mm1 = mt.Dr[n][r]+mt.Cr[n][r];
			mm1t += mm1;
			ms1 += format("%3.2f ",mt.Cr[n][r]);
			ms2 += format("%3.2f ",mt.Dr[n][r]);
			ms3 += format("%3.2f ",mm1);
		}
		ms1 += "]";
		ms2 += "]";
		ms3 += "]";
		cout << "  Maker type " << n << " - Metric: " << mm1t << endl;
		cout << "  " << ms1 << endl;
		cout << "  " << ms2 << endl;
		cout << "  " << ms3 << endl;
		for (int m=0; m<mt.RingNum[n]; m++) {
			cout << "  Test for Dr matrix " << m << endl;
			mm1t = 0;
			ms1 = "Cr [ ";
			ms2 = "Dr [ ";
			ms3 = "Mr [ ";
			for (int r=0; r<mt.RingNum[n]; r++) {
				mm1 = mt.Drm[n][m][r]+mt.Cr[n][r];
				mm1t += mm1;
				ms1 += format("%3.2f ",mt.Cr[n][r]);
				ms2 += format("%3.2f ",mt.Drm[n][m][r]);
				ms3 += format("%3.2f ",mm1);
			}
			ms1 += "]";
			ms2 += "]";
			ms3 += "]";
			cout << "  Maker type " << n << " - Metric: " << mm1t << endl;
			cout << "  " << ms1 << endl;
			cout << "  " << ms2 << endl;
			cout << "  " << ms3 << endl;
		}
	}
	// ===============================
	// Create windows
	int flags = CV_WINDOW_NORMAL | CV_WINDOW_KEEPRATIO | CV_GUI_NORMAL;
	namedWindow("cam",flags);
	namedWindow("pr1",flags);
	namedWindow("pr2",flags);
	createTrackbar("Gray Threshold","pr1",0,255,onThresholdTrackbar);
	setTrackbarPos("Gray Threshold","pr1",ProcessPar.GrayThreshold);
	createTrackbar("Roundness Threshold","pr1",0,100,onRoundnessThresholdTrackbar);
	setTrackbarPos("Roundness Threshold","pr1",(int) ProcessPar.ContourMinRoundness);
	capture.read(cameraFrame);
	int ix = cameraFrame.cols;
	int iy = cameraFrame.rows;
	float imf = (float) iy/(float) ix;
	int wx = (CAPTUREWINDOW_WIDTH == -1 ? ix : CAPTUREWINDOW_WIDTH);
	int wy = (CAPTUREWINDOW_WIDTH == -1 ? iy : (int) ((float) wx*imf));
	resizeWindow("cam",wx,wy);
	resizeWindow("pr1",wx,wy);
	moveWindow("cam",10,10);
	moveWindow("pr1",10+(wx+25),10);
	// Set processing parameters
	// TODO: Add code to read parameters from input file if specified in command line
	// TODO: Add code to store and retreive last used parameter values from file
	SetDefaultPar();
	ResetStatistics();
	// == Using Retinex ===============================
	#ifdef PROCESSING_RETINEX
	CRetinex retinex;
	#endif
	// ================================================
	// Main loop
	while (true) {
		if (goprocessing) {
			// Capture, process and show frame and result
			capture.read(cameraFrame);
			ProcessFrame(cameraFrame);
			ShowResults();
		}
		else {
			// == Using Retinex ===============================
			#ifdef PROCESSING_RETINEX
			capture.read(cameraFrame);
			retinex.LoadImage(cameraFrame);
			retinex.SetThreshold(100.0);
			ProfilingStart(PROFILE_TIMER1);
			retinex.Run();
			ProfilingStop(PROFILE_TIMER1);
			cout << format("Retinex time: %3.0f ms",ProfilingGetTimeMs(PROFILE_TIMER1)) << endl;
			gframe = retinex.ResImage;
			cout << "Result type: " << retinex.ResImage.type() << endl;
			imshow("cam",cameraFrame);
			imshow("pr1",retinex.ResImage);
			#endif
			// ================================================
			// == Using Tan-Triggs Preprocessing ==============
			#ifdef PROCESSING_TANTRIGGS
			capture.read(cameraFrame);
			ProfilingStart(PROFILE_TIMER1);
			gframe = TanTriggsPreprocessing(cameraFrame);
			ProfilingStop(PROFILE_TIMER1);
			cout << format("Tan-Triggs time: %3.0f ms",ProfilingGetTimeMs(PROFILE_TIMER1)) << endl;
			imshow("cam",cameraFrame);
			imshow("pr1",gframe);
			#endif
			// ================================================
			// == Using Self-Quotient Image Preprocessing =====
			#ifdef PROCESSING_SELFQUOTIENT
			capture.read(cameraFrame);
			ProfilingStart(PROFILE_TIMER1);
			cvtColor(cameraFrame,gframe,CV_RGB2GRAY);
			gframe = SQIPreprocessing(gframe);
			ProfilingStop(PROFILE_TIMER1);
			cout << format("SQI time: %3.0f ms",ProfilingGetTimeMs(PROFILE_TIMER1)) << endl;
			imshow("cam",cameraFrame);
			imshow("pr1",gframe);
			#endif
			// ================================================
			// == Using Mod Self-Quotient Image Preprocessing =
			#ifdef PROCESSING_MODSELFQUOTIENT
			capture.read(cameraFrame);
			ProfilingStart(PROFILE_TIMER1);
			cvtColor(cameraFrame,gframe,CV_RGB2GRAY);
			gframe = MSQIPreprocessing(gframe,ProcessPar.MSQIThreshold,ProcessPar.MSQISigma);
			ProfilingStop(PROFILE_TIMER1);
			cout << format("MSQI time: %3.0f ms",ProfilingGetTimeMs(PROFILE_TIMER1)) << endl;
			imshow("cam",cameraFrame);
			imshow("pr1",gframe);
			#endif
			// ================================================
		}
		// Check keyboard input
		int k = 0xff & waitKey(30);
		switch (k) {
		case 27:
		case 'q':
		case 'Q':
			quit = true;
			break;
		case 'g':
		case 'G':
			goprocessing = !goprocessing;
			break;
		case 'm':
		case 'M':
			ProcessingMode++;
			if (ProcessingMode>=PROCMOD_MODESNUM)
				ProcessingMode=0;
		}
		if (quit)
			break;
	}
	return 0;
}