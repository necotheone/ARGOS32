// Marker.h
// Landing Pad marker header
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   07.09.2013
// Description: Landing Pad marker class
// References: See full description and references in ARGOSLandingPad.cpp
///////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
using namespace cv;
using namespace std;

// -- Helper definitions --------------------------------------------------------------------------

#define tvContour   vector<vector<Point>>
#define tvHierarchy vector<Vec4i>
#define tvMoments   vector<Moments>
#define tvMCenters  vector<Point2f>
#define tvFloat     vector<float>
#define tvvFloat    vector<vector<float>>
#define tContour    vector<Point>
#define tHierarchy  Vec4i

#define RINGDETECTION_OPTIMIZED
//#define RINGDETECTION_DEBUG
#define RINGDETECTION_RADIUSTHRESHOLD 1.0

// -- Landmark definitions ------------------------------------------------------------------------

#define LANDMARK_CONFIGVER 2	// Current marker type configuration version
#define LANDMARK_RINGNUM 4		// Number of rings included in default landmark

// -- Marker structure variables ------------------------------------------------------------------

class CCircle {					// Class holding recognized circles
public:
	// Member variables
	vector<Point> Contour;			// Contour of circle
	Point2f       MassCenter;		// Mass center of contour
	float         Area;				// Contour area
	float         Perimeter;		// Contour length
	float         Roundness;		// Contour roundness calculated from length and area
	RotatedRect   Ellipse;			// Bounding rectangle containing best-fitting ellipse
	float         CircleRadius;		// Circle radius obtained from ellipse major semiaxis
	float         Eccentricity;		// Ellipse eccentricity
	// Construction
	CCircle();
	CCircle(tContour &contour,Point2f &masscenter,float area,float perimeter,float roundness);
	// Operator overloading
	CCircle& operator=(const CCircle& c);
	// Methods
	void CalcEllipse();
};

class CMarkerRing {				// Class holding recognized anular markers
public:
	// Member variables
	CCircle    InnerCircle;		// Inner circle
	CCircle    OuterCircle;		// Outer circle
	float      Area;			// Anular ring area (difference between outer and inner areas)
	Point2f    Center;			// Ring center (mean location of inner and outer centers)
	float      CenterDist;		// Distance between inner and outer circle centers
	float      Cr;				// Ratio of inner to outer circle radius
	// Construction
	CMarkerRing();
	CMarkerRing(CCircle &outercircle,CCircle & innercircle);
	// Comparison operators (rings are considered equal if they overlap)
	bool operator<(CMarkerRing r)  { return OuterCircle.CircleRadius < r.InnerCircle.CircleRadius; }
	bool operator>(CMarkerRing r)  { return InnerCircle.CircleRadius > r.OuterCircle.CircleRadius; }
	bool operator==(CMarkerRing r) { return !(OuterCircle.CircleRadius < r.InnerCircle.CircleRadius) && !(InnerCircle.CircleRadius > r.OuterCircle.CircleRadius); }
};

// -- Marker type class ---------------------------------------------------------------------------
// This class encapsulates the marker type characteristics. Every individual marker will be given
// a type through the use of a type identifier. Data related to marker type must be accessed using
// the marker-independent instance of this class.
// TODO: This class needs to be converted to landpad, which represents the complete landpad as a
// marker set, including the defined marker types.

class CMarkerType {
public:
	// Marker type characteristics
	int                 ConfVersion;	// Configuration version
	int                 NumTypes;		// Number of marker types defined
	string              ConfFile;		// Marker type configuration file name
	vector<int>         RingNum;		// Number of rings to be detected for each marker type
	vector<tvFloat>     Cr;				// List of ring radius ratios for each marker type
	vector<tvFloat>     CrThr;			// Thresholds for Cr to get the ring id from radius ratio
	vector<tvFloat>     Dr;				// List of relative diameter ratios for each marker type
	vector<tvvFloat>    Drm;			// Matrix of relative diameter ratios for each marker type
	vector<tvFloat>     DrThr;			// Thresholds for Dr to organize detected rings in the marker
	// Construction
	CMarkerType();
	CMarkerType(string fname);
	// Initialization
	void DefaultType();					// Set default marker type configuration
	void PopulateDrMatrix();			// Calculate and populate Dr matrix from Dr values
	bool Load();						// Load marker types from config file
	bool Load(string fname);			// Load marker types from specified file
	bool Save();						// Save marker types from config file
	bool Save(string fname);			// Save marker types to specified file
	// Getting properties
	// Warning: these functions don't check vector limits
	int      GetRingNum(int t)             { return RingNum[t]; }
	tvFloat  GetCr(int t)                  { return Cr[t]; }
	float    GetCr(int t,int r)            { return Cr[t][r]; }
	tvFloat  GetCrThr(int t)               { return CrThr[t]; }
	float    GetCrThrMin(int t,int r)      { return CrThr[t][r]; }
	float    GetCrThrMax(int t,int r)      { return CrThr[t][r+1]; }
	tvFloat  GetDr(int t)                  { return Dr[t]; }
	float    GetDr(int t,int r)            { return Dr[t][r]; }
	tvFloat  GetDrThr(int t)               { return DrThr[t]; }
	float    GetDrThrMin(int t,int r)      { return DrThr[t][r]; }
	float    GetDrThrMax(int t,int r)      { return DrThr[t][r+1]; }
	tvvFloat GetDrM(int t)                 { return Drm[t]; }
	tvFloat  GetDrMRow(int t,int r)        { return Drm[t][r]; }
	float    GetDrMVal(int t,int r,int c)  { return Drm[t][r][c]; }
};

// -- Landing Pad marker class --------------------------------------------------------------------
// This class encapsulates an individual fiducial marker, which is a set of concentric white rings
// on a dark background with distinct inner/outer radius ratios that identify each ring.
// A marker is characterized by its type, which is defined by the set of radius ratios and by the
// number of rings, and by its location.

// Some thoughts on different markers
// When rings are detected, they are associated with an individual marker using only its location.
// If a ring is detected in a location where no previous marker exists, a new marker will be
// created and the ring will be associated to it. Conversely, if a ring is detected with a location
// that is inside an already detected marker, an association to that marker will be tried taking
// into account the ring Cr and radius: a marker cannot contain more than one ring with the same Cr
// or with the same radius. Here, 'same' is quantified as similar magnitudes. For Cr, thresholding
// determines the ring identification. For radius, the outer ring circle radius is used and also
// thresholded for identification.
// Different markers require a type definition that minimizes misclassified rings. This is done by
// defining a general, landpad-wide Cr set with its corresponding threshold set. Each unique marker
// will contain rings with a unique Cr subset.

// Notes on confidence measures for marker identification
// The most important confidence measure comes from three parameters:
// 1. Number of rings detected
// 2. Ring radius ratio variation from the reference ratio defined by the marker template
// 3. Center position variation from the various circles contained in the marker
// All confidence measurements are calculated in the Landing Pad marker class, which holds
// the required information (all the centers for ring circles, the reference ratios, the
// rings detected for this marker).

class CMarker {
public:
	// Marker type characteristics
	CMarkerType*        MarkerType;		// Marker type information
	int                 TypeId;			// Marker type index
	// Marker detected characteristics
	int                 RingNum;		// Number of detected rings in marker
	vector<CMarkerRing> Rings;			// Rings (index is the ring Id, 0 is the innermost)
	vector<bool>        ValidRings;		// Flag indicating if ring has been detected
	Point2f             Position;		// Marker calculated center (averaged center location of rings)
	float               Radius;			// Marker calculated radius (radius of the outmost ring outer circle)
	// Marker detection confidence measures
	float               ConfNumRings;	// Conerafidence of marker identification based on number of detected rings
	float               ConfCr;			// Confidence of marker identification based on Cr
	float               ConfDr;			// Confidence of marker identification based on Dr
	float               ConfCenter;		// Confidence of marker identification based on ring centers
	// Helper members
	int                 TmpRingsNum;	// Number of rings added but not assigned
	vector<CMarkerRing> TmpRings;		// Rings added to the marker, but not assigned to marker structure
	// Construction
	CMarker(CMarkerType* mt);
	// Methods
private:
	int   RingCrId(CMarkerRing &r);				// Get the ring id based on its radius ratio Cr
public:
	// Marker structure
	bool  AddRing(CMarkerRing &r);				// Calculates the ring id and adds it to the marker
	void  RemoveRings();						// Removes all rings stored in the marker
	bool  CheckPointInside(Point2f p);			// Check if p is inside all the rings contained in marker
	bool  CheckPointInsideTmp(Point2f p);		// Check if p is inside all the temporal rings stored in marker
	// Confidence measures and characteristics
	float CalcRingCrCM(int i);					// Calculates Cr confidence measure for the ring identified by its index
	float CalcRingCrCM(int i,float Cr);			// Calculates Cr confidence measure for the ring position i with specified Cr value
	float CalcRingDrCM(int i);					// Calculates Dr confidence measure for the ring identified by its index
	float CalcRingDrCM(int i,float Dr);			// Calculates Dr confidence measure for the ring position i with specified Dr value
	float CalcRingDrCMNorm(int i,float Dr);		// Calculates Dr confidence measure for the ring position i with supplied normalized Dr value
	void  CalcConfMeasures();					// Calculates confidence measures for the current detected characteristics
	void  CalcPosition();						// Calculates marker position as the averaged position of detected ring centers
	void  CalcRadius();							// Calculates the marker radius as the outmost ring outer circle radius
	// Getting ring properties
	// Warning: these functions don't check vector limits
	float    GetRingRadius(int idx)    { return Rings[idx].OuterCircle.CircleRadius; }
	float    GetTmpRingRadius(int idx) { return TmpRings[idx].OuterCircle.CircleRadius; }
	// Getting marker type properties
	// Warning: these functions don't check vector limits
	// TODO: Optimize speed by directly retrieving data instead of calling CMarkerType function
	int      GetTRingNum()             { return MarkerType->GetRingNum(TypeId); }
	tvFloat  GetTCr()                  { return MarkerType->GetCr(TypeId); }
	float    GetTCr(int r)             { return MarkerType->GetCr(TypeId,r); }
	tvFloat  GetTCrThr()               { return MarkerType->GetCrThr(TypeId); }
	float    GetTCrThrMin(int r)       { return MarkerType->GetCrThrMin(TypeId,r); }
	float    GetTCrThrMax(int r)       { return MarkerType->GetCrThrMax(TypeId,r); }
	tvFloat  GetTDr()                  { return MarkerType->GetDr(TypeId); }
	float    GetTDr(int r)             { return MarkerType->GetDr(TypeId,r); }
	tvFloat  GetTDrThr()               { return MarkerType->GetDrThr(TypeId); }
	float    GetTDrThrMin(int r)       { return MarkerType->GetDrThrMin(TypeId,r); }
	float    GetTDrThrMax(int r)       { return MarkerType->GetDrThrMax(TypeId,r); }
	tvvFloat GetTDrM()                 { return MarkerType->GetDrM(TypeId); }
	tvFloat  GetTDrMRow(int r)         { return MarkerType->GetDrMRow(TypeId,r); }
	float    GetTDrMVal(int r,int c)   { return MarkerType->GetDrMVal(TypeId,r,c); }
	// Marker information display
	void  ShowMarkerInfo();						// Shows information on marker characteristics
};

// -- Landpad class -------------------------------------------------------------------------------
// This class encapsulates the landing pad object, which is made up of the landpad marker types
// used, the landing pad marker geometry and the detected landpad markers.

class CLandingPad {
public:
	// Landing pad definition
	CMarkerType        MarkerTypeDef;	// Marker type definitions
	int                NumMarkersDef;	// Number of markers of this landpad
	vector<int>        MarkerTypeIds;	// List of marker type indexes of markers in this landpad
	// Landing pad detection
	int                NumMarkers;		// Number of detected markers
	vector<CMarker>    Markers;			// List of detected markers
	// Construction
	CLandingPad();
	// Methods
	void Reset();						// Reinitialize landing pad instance
	bool AddRing(CMarkerRing &r);		// Add a new ring checking hierarchy
	void  OrganizeRings();				// Organize added rings into the marker structure
	void ShowLandingPadInfo();			// Show information on landing pad detected characteristics
};

