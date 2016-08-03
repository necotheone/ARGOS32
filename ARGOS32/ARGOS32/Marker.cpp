// Marker.cpp
// Landing Pad marker implementation
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   07.09.013
// Description: Landing Pad marker class
// References: See full description and references in ARGOSLandingPad.cpp
///////////////////////////////////////////////////////////////////////////////////////////////////

// -- Class declarations --------------------------------------------------------------------------

#include "Marker.h"

// -- Logging library -----------------------------------------------------------------------------

#include "EasyLogging\easylogging++.h"

using namespace cv;

// == Circle implementation =======================================================================

// -- Construction --------------------------------------------------------------------------------

CCircle::CCircle() {
	Area = 0.0;
	Perimeter = 0.0;
	Roundness = 0.0;
	Eccentricity = 0.0;
	CircleRadius = 0.0;
}

CCircle::CCircle(tContour &contour,Point2f &masscenter,float area,float perimeter,float roundness) {
	Contour = contour;
	MassCenter = masscenter;
	Area = area;
	Perimeter = perimeter;
	Roundness = roundness;
	CalcEllipse();
}

// -- Operator overloading ------------------------------------------------------------------------

CCircle& CCircle::operator=(const CCircle& c) {
	if (this!=&c) {
		Contour = c.Contour;
		MassCenter = c.MassCenter;
		Area = c.Area;
		Perimeter = c.Perimeter;
		Roundness = c.Roundness;
		CalcEllipse();
	}
	return *this;
}

// -- Methods -------------------------------------------------------------------------------------

void CCircle::CalcEllipse() {
	float a, b;
	Ellipse = fitEllipse(Contour);
	a = Ellipse.size.height/2;		// Major semiaxis
	b = Ellipse.size.width/2;		// Minor semiaxis
	// == LOG DEBUG ===========================
	//CLOG(TRACE, "default") << "ELLIPSE " << a << "," << b;
	// ========================================
	Eccentricity = sqrt(1-(b/a)*(b/a));
	CircleRadius = a;
}


// == Ring implementation =========================================================================

// -- Construction --------------------------------------------------------------------------------

CMarkerRing::CMarkerRing() {
	Area = 0.0;
	CenterDist = 0.0;
	Cr = 0.0;
}

CMarkerRing::CMarkerRing(CCircle &outercircle,CCircle &innercircle) {
	float cdx,cdy;

	OuterCircle = outercircle;
	InnerCircle = innercircle;
	Area = OuterCircle.Area - InnerCircle.Area;
	cdx = OuterCircle.MassCenter.x+InnerCircle.MassCenter.x;
	cdy = OuterCircle.MassCenter.y+InnerCircle.MassCenter.y;
	Center = Point2f(cdx/2.0f,cdy/2.0f);
	cdx = OuterCircle.MassCenter.x-InnerCircle.MassCenter.x;
	cdy = OuterCircle.MassCenter.y-InnerCircle.MassCenter.y;
	CenterDist = sqrt(cdx*cdx + cdy*cdy);
	Cr = sqrt(innercircle.Area/outercircle.Area);
}

// -- Methods -------------------------------------------------------------------------------------


// == Landing Pad marker implementation ===========================================================

// -- Construction --------------------------------------------------------------------------------

CMarker::CMarker(CMarkerType* mt, string logger) {
	// Initialize marker characteristics
	MarkerType = mt;
	TypeId = 0;
	RingNum = 0;
	Rings.resize(MarkerType->RingNum[TypeId]);
	ValidRings.resize(MarkerType->RingNum[TypeId]);
	for (int i=0; i<MarkerType->RingNum[TypeId]; i++)
		ValidRings[i]=false;
	// Initialize detection confidence measures
	ConfNumRings = 0.0;
	ConfCr = 0.0;
	ConfCenter = 0.0;
#ifdef RINGDETECTION_OPTIMIZED
	TmpRingsNum=0;
#endif
    // Setup logging
    LoggerName = logger;
}

// -- Landing pad marker methods ------------------------------------------------------------------

// Ring identification obtained from inner/outer radius ratio
int CMarker::RingCrId(CMarkerRing &r) {
	int res=-1;

	// Identification uses cr thresholds averaged from adyacent cr default values
	for (int i=0; i<MarkerType->RingNum[TypeId]; i++) {
		if (r.Cr>=MarkerType->CrThr[TypeId][i] && r.Cr<MarkerType->CrThr[TypeId][i+1]) {
			res = i;
			break;
		}
	}
	return res;
}

// Adding a new ring to the marker after calculating its id and confidence
// The optimized version stores all rings in a temporal buffer inside the marker.
// When all rings have been added, the marker is organized from the landing pad
// object: each ring is given an id depending on relative diameters and Cr, 
// ring conflicts are solved and rings are stored in the marker ring vector.
bool CMarker::AddRing(CMarkerRing &r) {
	bool res = false;
#ifdef RINGDETECTION_OPTIMIZED
	TmpRings.push_back(r);
	TmpRingsNum++;
#else
    int id;
    id = RingCrId(r);
	if (id>=0) {
		// Check if ring was previously stored
		if (ValidRings[id]) {
			// TODO: Handle error messages properly
			// == LOG DEBUG ===========================
            CLOG(TRACE, Logger()) << format("DEL RING - ID[%d] CR[%01.3f]",id,r.Cr);
			// ========================================
		}
		else {
			RingNum++;
			ValidRings[id] = true;
			Rings[id] = r;
			CalcPosition();
			CalcRadius();
			CalcConfMeasures();
			res = true;
			// TODO: Remove when done
			// == LOG DEBUG ===========================
            CLOG(TRACE, Logger()) << format("ADD RING - ID[%d] RN[%d] CR[%01.3f]",id,RingNum,Rings[id].Cr);
			// ========================================
		}
	}
#endif
	return res;
}

// Removes all rings stored in the marker
void CMarker::RemoveRings() {
	RingNum=0;
	for (int i=0; i<MarkerType->RingNum[TypeId]; i++)
		ValidRings[i]=false;
}

// Calculates Cr confidence measure for the ring identified by its index
float CMarker::CalcRingCrCM(int i) {
	float CrCM=0.0;
	if (ValidRings[i])
		CrCM = CalcRingCrCM(i,Rings[i].Cr);
	return CrCM;
}

// Calculates Cr confidence measure for the ring position i with specified Cr value
float CMarker::CalcRingCrCM(int i,float Cr) {
	float DiffCr;
	float NormCr;
	float CrCM=0.0;
	if (ValidRings[i]) {
		DiffCr = Cr - MarkerType->Cr[TypeId][i];
		if (DiffCr>0) {
			NormCr = MarkerType->CrThr[TypeId][i+1] - MarkerType->Cr[TypeId][i];
		}
		else {
			NormCr = MarkerType->CrThr[TypeId][i] - MarkerType->Cr[TypeId][i];
		}
		CrCM = 1 - (DiffCr/NormCr);
	}
	return CrCM;
}

// Calculates Dr confidence measure for the ring identified by its index
float CMarker::CalcRingDrCM(int i) {
	float DrCM=0.0;
	if (ValidRings[i])
		DrCM = CalcRingDrCM(i,Rings[i].OuterCircle.CircleRadius);
	return DrCM;
}

// Calculates Dr confidence measure for the ring position i with specified Dr value
float CMarker::CalcRingDrCM(int i,float Dr) {
	float DiffDr;
	float NormDr;
	float DrCM=0.0;
	//if (ValidRings[i]) {
		DiffDr = (Dr/Radius) - MarkerType->Dr[TypeId][i];
		if (DiffDr>0) {
			NormDr = MarkerType->DrThr[TypeId][i+1] - MarkerType->Dr[TypeId][i];
		}
		else {
			NormDr = MarkerType->DrThr[TypeId][i] - MarkerType->Dr[TypeId][i];
		}
		DrCM = 1 - (DiffDr/NormDr);
	//}
	return DrCM;
}

// Calculates Dr confidence measure for the ring position i with supplied normalized Dr value
float CMarker::CalcRingDrCMNorm(int i,float Dr) {
	float DiffDr;
	float NormDr;
	float DrCM=0.0;
	DiffDr = Dr - MarkerType->Dr[TypeId][i];
	if (DiffDr>0) {
		NormDr = MarkerType->DrThr[TypeId][i+1] - MarkerType->Dr[TypeId][i];
	}
	else {
		NormDr = MarkerType->DrThr[TypeId][i] - MarkerType->Dr[TypeId][i];
	}
	DrCM = 1 - (DiffDr/NormDr);
	return DrCM;
}

// Calculates confidence measures for the current detected characteristics
void CMarker::CalcConfMeasures() {
	ConfNumRings = 0.0;
	ConfCr = 0.0;
	ConfDr = 0.0;
	ConfCenter = 0.0;
	for (int i=0; i<MarkerType->RingNum[TypeId]; i++) {
		if (ValidRings[i]) {
			ConfNumRings++;
			ConfCr += CalcRingCrCM(i,Rings[i].Cr);
			ConfDr += CalcRingDrCM(i,Rings[i].OuterCircle.CircleRadius);
		}
	}
	ConfNumRings = ConfNumRings/MarkerType->RingNum[TypeId];
	ConfCr = ConfCr/RingNum;
	ConfDr = ConfDr/RingNum;
	ConfCenter = ConfCenter/MarkerType->RingNum[TypeId];
}

// Calculates marker position as the averaged position of detected ring centers
void CMarker::CalcPosition() {
	Position = Point2f(0,0);
	for (int i=0; i<MarkerType->RingNum[TypeId]; i++) {
		if (ValidRings[i]) {
			Position.x += Rings[i].Center.x;
			Position.y += Rings[i].Center.y;
		}
	}
	Position.x /= RingNum;
	Position.y /= RingNum;
}

// Calculates the marker radius as the outmost ring outer circle radius
void CMarker::CalcRadius() {
	// Radius will be 0 if there is no ring in the marker
	Radius = 0.0;
	// The outmost ring is the last valid ring in the marker
	for (int i=MarkerType->RingNum[TypeId]-1; i>=0; i--) {
		if (ValidRings[i]) {
			Radius = Rings[i].OuterCircle.CircleRadius;
			break;
		}
	}
}

// Get the logger name
tCStr CMarker::Logger() {
    return LoggerName.c_str();
}

// Shows information on marker characteristics
void CMarker::ShowMarkerInfo() {
    CLOG(INFO, Logger()) << format("LPM INFO - N[%d] P(%1.3f,%1.3f) Conf { NR[%1.3f] Cr[%1.3f] Dr[%1.3f] MC[%1.3f] }", RingNum, Position.x, Position.y, ConfNumRings, ConfCr, ConfDr, ConfCenter);
	for (int i=0; i<MarkerType->RingNum[TypeId]; i++) {
		string msg = format("      RING %d ",i);
		string msg1, msg2;
		if (ValidRings[i]) {
			msg = msg + format("C(%1.3f,%1.3f) CD[%3.1f] Ar[%3.1f] Cr[%1.3f]",Rings[i].Center.x,Rings[i].Center.y,Rings[i].CenterDist,Rings[i].Area,Rings[i].Cr);
			msg1 = format("       INNER C(%1.3f,%1.3f) Rd[%3.1f] Pe[%3.1f] Ar[%3.1f] Rn[%1.3f] Ec[%1.3f]",Rings[i].InnerCircle.MassCenter.x,Rings[i].InnerCircle.MassCenter.y,Rings[i].InnerCircle.CircleRadius,Rings[i].InnerCircle.Perimeter,Rings[i].InnerCircle.Area,Rings[i].InnerCircle.Roundness,Rings[i].InnerCircle.Eccentricity);
			msg2 = format("       OUTER C(%1.3f,%1.3f) Rd[%3.1f] Pe[%3.1f] Ar[%3.1f] Rn[%1.3f] Ec[%1.3f]",Rings[i].OuterCircle.MassCenter.x,Rings[i].OuterCircle.MassCenter.y,Rings[i].OuterCircle.CircleRadius,Rings[i].OuterCircle.Perimeter,Rings[i].OuterCircle.Area,Rings[i].OuterCircle.Roundness,Rings[i].OuterCircle.Eccentricity);
            CLOG(INFO, Logger()) << msg;
            CLOG(INFO, Logger()) << msg1;
            CLOG(INFO, Logger()) << msg2;
        }
		else
            CLOG(INFO, Logger()) << msg;
    }
}

// Check if point p is inside all the rings contained in marker
bool CMarker::CheckPointInside(Point2f p) {
	int i,r;
	float d2,r2;
	bool res = true;

	for (i=0; i<(int) Rings.size(); i++) {
		//r = pointPolygonTest(Rings[i].InnerCircle.Contour,p,false);
		if (ValidRings[i]) {
			d2 = (p.x-Rings[i].Center.x)*(p.x-Rings[i].Center.x) + (p.y-Rings[i].Center.y)*(p.y-Rings[i].Center.y);
			r2 = Rings[i].InnerCircle.CircleRadius*Rings[i].InnerCircle.CircleRadius;
			r = (int) (r2 - d2);
			// == LOG DEBUG ===========================
            CLOG(TRACE, Logger()) << "R2 = " << r2;
            CLOG(TRACE, Logger()) << "R  = " << Rings[i].InnerCircle.CircleRadius;
            CLOG(TRACE, Logger()) << "D2 = " << d2;
			// ========================================
			if (r<=0) {
				res = false;
				break;
			}
		}
	}
	return res;
}

// Check if point p is inside all the temporal rings stored in marker
bool CMarker::CheckPointInsideTmp(Point2f p) {
	int i,r;
	float d2,r2;
	bool res = true;

	for (i=0; i<TmpRingsNum; i++) {
		d2 = (p.x-TmpRings[i].Center.x)*(p.x-TmpRings[i].Center.x) + (p.y-TmpRings[i].Center.y)*(p.y-TmpRings[i].Center.y);
		r2 = TmpRings[i].InnerCircle.CircleRadius*TmpRings[i].InnerCircle.CircleRadius;
		r = (int) (r2 - d2);
		if (r<=0) {
			res = false;
			break;
		}
	}
	return res;
}

// == Marker type implementation ==================================================================

// -- Construction --------------------------------------------------------------------------------

CMarkerType::CMarkerType() {
	ConfVersion = LANDMARK_CONFIGVER;
	ConfFile = "ARGOSMarkerTypesV2.yml";
	DefaultType();
	// TODO: Check if file exists
	Load(ConfFile);
	PopulateDrMatrix();
}

CMarkerType::CMarkerType(string fname) {
	ConfVersion = LANDMARK_CONFIGVER;
	ConfFile = fname;
	DefaultType();
	// TODO: Check if file exists
	Load(ConfFile);
	PopulateDrMatrix();
}

// -- Marker type methods -------------------------------------------------------------------------

void CMarkerType::DefaultType() {
	float crRef[LANDMARK_RINGNUM] = {0.50f,0.65f,0.75f,0.85f};
	float drRef[LANDMARK_RINGNUM] = {0.1667f,0.4000f,0.7078f,1.000f};
	NumTypes = 1;
	RingNum.resize(NumTypes);
	Cr.resize(NumTypes);
	CrThr.resize(NumTypes);
	Dr.resize(NumTypes);
	DrThr.resize(NumTypes);
	Drm.resize(NumTypes);
	RingNum[0]=LANDMARK_RINGNUM;
	// Initialize marker type Cr list
	Cr[0].resize(RingNum[0]);
	for (int i=0; i<RingNum[0]; i++)
		Cr[0][i] = crRef[i];
	// Initialize marker type Cr thresholds
	CrThr[0].resize(RingNum[0]+1);
	CrThr[0][0] = 0.0;
	CrThr[0][RingNum[0]] = 1.0;
	for (int i=1; i<RingNum[0]; i++)
		CrThr[0][i] = (Cr[0][i]+Cr[0][i-1])/2.0f;
	// Initialize marker type Dr list
	Dr[0].resize(RingNum[0]);
	for (int i=0; i<RingNum[0]; i++)
		Dr[0][i] = drRef[i];
	// Initialize marker type Dr thresholds
	DrThr[0].resize(RingNum[0]+1);
	DrThr[0][0] = 0.0;
	DrThr[0][RingNum[0]] = 1.0;
	for (int i=1; i<RingNum[0]; i++)
		DrThr[0][i] = (Dr[0][i]+Dr[0][i-1])/2.0f;
	// Initialize marker type Dr matrix
	PopulateDrMatrix();
}

void CMarkerType::PopulateDrMatrix() {
	Drm[0].resize(RingNum[0]);
	for (int i=0; i<RingNum[0]; i++) {
		Drm[0][i].resize(RingNum[0]);
		float baseDr = Dr[0][i];
		for (int j=0; j<RingNum[0]; j++)
			Drm[0][i][j] = Dr[0][j]/baseDr;
	}
}

bool CMarkerType::Load() {
	return Load(ConfFile);
}

bool CMarkerType::Load(string fname) {
	int v;
	try {
		FileStorage fs(fname,FileStorage::READ);
		fs["Version"] >> v;
		if (v!=ConfVersion) {
			LOG(WARNING) << "CONFIG ERROR: Configuration file version does not match current version";
			LOG(WARNING) << "              Loading default marker configuration";
			DefaultType();
			return false;
		}
		fs["NumTypes"] >> NumTypes;
		RingNum.resize(NumTypes);
		Cr.resize(NumTypes);
		CrThr.resize(NumTypes);
		Dr.resize(NumTypes);
		DrThr.resize(NumTypes);
		FileNode MarkerStructures = fs["MarkerStructures"];
		FileNodeIterator itb = MarkerStructures.begin();
		FileNodeIterator ite = MarkerStructures.end();
		int i = 0;
		for ( ; itb!=ite; ++itb, i++) {
			RingNum[i] = (int)(*itb)["RingNum"];
			Cr[i].resize(RingNum[i]);
			CrThr[i].resize(RingNum[i]+1);
			Dr[i].resize(RingNum[i]);
			DrThr[i].resize(RingNum[i]+1);
			(*itb)["Cr"] >> Cr[i];
			(*itb)["CrThr"] >> CrThr[i];
			(*itb)["Dr"] >> Dr[i];
			(*itb)["DrThr"] >> DrThr[i];
		}
		fs.release();
		// == LOG DEBUG ===========================
		// TODO: Remove when checking load finishes
        LOG(TRACE) << "Configuration loaded, version " << format("%02d",v);
        LOG(TRACE) << "Number of types: " << NumTypes;
		for (int i=0; i<NumTypes; i++) {
            LOG(TRACE) << "Structure " << i << ": " << RingNum[i] << " rings";
			string msg = "Cr:    ";
			for (int j=0; j<RingNum[i]; j++)
				msg += format("%1.3f ",Cr[i][j]);
            LOG(TRACE) << msg;
			msg = "CrThr: ";
			for (int j=0; j<RingNum[i]+1; j++)
				msg += format("%1.3f ",CrThr[i][j]);
            LOG(TRACE) << msg;
		}
		// ========================================
	}
	catch( cv::Exception& e ) {
		const char* err_msg = e.what();
		// == LOG DEBUG ===========================
        LOG(ERROR) << "Exception caught: " << err_msg;
		// ========================================
	}
	return true;
}

bool CMarkerType::Save() {
	return Save(ConfFile);
}

bool CMarkerType::Save(string fname) {
	FileStorage fs(fname, FileStorage::WRITE);
	fs << "Version" << ConfVersion;
	fs << "NumTypes" << NumTypes;
	fs << "MarkerStructures" << "[";
	for (int i=0; i<NumTypes; i++) {
		fs << "{:" << "RingNum" << RingNum[i];
		fs << "Cr" << "[:";
		for (int j=0; j<RingNum[i]; j++)
			fs << Cr[i][j];
		fs << "]" << "CrThr" << "[:";
		for (int j=0; j<RingNum[i]+1; j++)
			fs << CrThr[i][j];
		fs << "]" << "Dr" << "[:";
		for (int j=0; j<RingNum[i]; j++)
			fs << Dr[i][j];
		fs << "]" << "DrThr" << "[:";
		for (int j=0; j<RingNum[i]+1; j++)
			fs << DrThr[i][j];
		fs << "]" << "}";
	}
	fs << "]";
	fs.release();
	return true;
}

// == Landpad implementation ======================================================================

// -- Construction --------------------------------------------------------------------------------

CLandingPad::CLandingPad() {
	NumMarkers = 0;
	// TODO: Complete landing pad geometric definition
	// TODO: Handle landing pad configuration file from here (not from MarkerType)
}

// -- Methods -------------------------------------------------------------------------------------

// Reinitialize landing pad instance
void CLandingPad::Reset() {
	int i;
	for (i=0; i<NumMarkers; i++)
		Markers[i].RemoveRings();
	Markers.clear();
	NumMarkers = 0;
}

// Add a new ring to the most likely parent marker, or create a new marker if needed
bool CLandingPad::AddRing(CMarkerRing &r) {
	int i;
	int isInsideIdx = -1;
	bool res;
	// Check if the ring center is inside a marker
	for (i=0; i<NumMarkers; i++) {
		if (Markers[i].CheckPointInsideTmp(r.Center)) {
			isInsideIdx = i;
			// == LOG DEBUG ===========================
            CLOG(TRACE, Logger()) << format("RING C(%3.1f,%3.1f) INSIDE MARKER %d",r.Center.x,r.Center.y,i);
			// ========================================
			break;
		}
	}
	// If it is not inside any marker, create a new one
	if (isInsideIdx==-1) {
        Markers.push_back(CMarker(&MarkerTypeDef, LoggerName));
		NumMarkers++;
		res = Markers[NumMarkers-1].AddRing(r);
	}
	// If it is inside a marker, add the ring to it
	else
		res = Markers[isInsideIdx].AddRing(r);
	return res;
}

// Organize added rings into the marker structure
// Organization is done for each marker with the following procedure:
// 1. Sort temporal rings with ascending diameter
// 2. Remove overlapping rings keeping the one with the best Cr confidence
void CLandingPad::OrganizeRings() {
	int i,j;
	for (i=0; i<NumMarkers; i++) {
		// == LOG DEBUG ===========================
		#ifdef RINGDETECTION_DEBUG
        CLOG(TRACE, Logger()) << "ORGANIZING MARKER " << i;
        CLOG(TRACE, Logger()) << "  " << Markers[i].TmpRingsNum << " DETECTED";
		vector<CMarkerRing>::iterator its = Markers[i].TmpRings.begin();
		j=0;
		while (its!=Markers[i].TmpRings.end()) {
            CLOG(TRACE, Logger()) << format("  RING %d - R[%3.1f]",j,(*its).OuterCircle.CircleRadius);
			j++;
			its++;
		}
		#endif
		// ========================================
		// Sort temporal ring list in ascending order
		sort(Markers[i].TmpRings.begin(),Markers[i].TmpRings.end());
		// == LOG DEBUG ===========================
		#ifdef RINGDETECTION_DEBUG
        CLOG(TRACE, Logger()) << "  " << Markers[i].TmpRingsNum << " SORTED";
		its = Markers[i].TmpRings.begin();
		j=0;
		while (its!=Markers[i].TmpRings.end()) {
            CLOG(TRACE, Logger()) << format("  RING %d - R[%3.1f]",j,(*its).OuterCircle.CircleRadius);
			j++;
			its++;
		}
		#endif
		// ========================================
		// Check for equal rings (rings overlapping)
		// TODO: Verify that this situation can really happen; if not, remove this check
		vector<CMarkerRing>::iterator it = Markers[i].TmpRings.begin();
		vector<CMarkerRing>::iterator itn;
		j = 0;
		while (j<Markers[i].TmpRingsNum-1) {
			itn = it+1;
			if (*it==*itn) {
				// == LOG DEBUG ===========================
				#ifdef RINGDETECTION_DEBUG
                CLOG(TRACE, Logger()) << "  OVERLAPPING RINGS DETECTED";
				#endif
				// ========================================
				// Remove ring with lower Cr confidence
				float Cr1 = Markers[i].CalcRingCrCM(j,(*it).Cr);
				float Cr2 = Markers[i].CalcRingCrCM(j,(*itn).Cr);
				if (Cr1>Cr2) {
					Markers[i].TmpRings.erase(itn);
					Markers[i].TmpRingsNum--;
					it = Markers[i].TmpRings.begin();
					it = it+j+1;
				}
				else {
					Markers[i].TmpRings.erase(it);
					Markers[i].TmpRingsNum--;
					it = Markers[i].TmpRings.begin();
					it = it+j;
				}
			}
			j++;
		}
		// Check number of rings and compare it with expected maker rings
		const int markerExpRings = Markers[i].GetTRingNum();
		// == LOG DEBUG ===========================
		#ifdef RINGDETECTION_DEBUG
        CLOG(TRACE, Logger()) << "  EXPECTED RINGS: " << markerExpRings;
		#endif
		// ========================================
		if (Markers[i].TmpRingsNum == markerExpRings) {
			// == LOG DEBUG ===========================
			#ifdef RINGDETECTION_DEBUG
            CLOG(TRACE, Logger()) << "  ALL RINGS DETECTED: Copying to marker structure";
			#endif
			// ========================================
			// Number of rings as expected: copy rings to marker structure preserving order
			Markers[i].RingNum = Markers[i].TmpRingsNum;
			vector<CMarkerRing>::iterator it = Markers[i].TmpRings.begin();
			vector<CMarkerRing>::iterator itr = Markers[i].Rings.begin();
			vector<bool>::iterator itv = Markers[i].ValidRings.begin();
			for (; it!=Markers[i].TmpRings.end(); it++, itr++, itv++) {
				*itv = true;
				*itr = *it;
			}
			Markers[i].CalcPosition();
			Markers[i].CalcRadius();
			Markers[i].CalcConfMeasures();
		}
		if (Markers[i].TmpRingsNum < markerExpRings) {
			// One or more rings not detected: assign rings using Dr
			int numDiff = markerExpRings - Markers[i].TmpRingsNum;
			// == LOG DEBUG ===========================
			#ifdef RINGDETECTION_DEBUG
            CLOG(TRACE, Logger()) << format("  %d RINGS NOT DETECTED",numDiff);
			#endif
			// ========================================
			float ConfDr;
			float MaxCDr = -1000.0;
			int   MaxCDrIdx = 0;
			vector<vector<float>> TmpDr;
			vector<vector<int>>   TmpIdx;
			TmpDr.resize(numDiff+1);
			TmpIdx.resize(numDiff+1);
			// Assume all detected rings are from the marker
			// Thus, the largest detected ring can be the marker outmost ring or one of the inner rings
			// The number of possibilities depends on the number of detected rings
			// The Dr matrix stored in the marker template is used
			// Rows in the Dr matrix are normalized using the ring location indicated by the row index
			// Thus, the first row assumes the maximum diameter ring is the first one (the innermost marker ring)
			// Conversely, the last row gives the normalized Dr values using the outmost marker ring
			// == LOG DEBUG ===========================
			#ifdef RINGDETECTION_DEBUG
            CLOG(TRACE, Logger()) << "  CALCULATING NORMALIZED Dr - " << numDiff+1 << " COMBINATIONS";
			#endif
			// ========================================
			for (j=0; j<=numDiff; j++) {
				TmpDr[j].resize(Markers[i].TmpRingsNum);
				TmpIdx[j].resize(Markers[i].TmpRingsNum);
				// Calculate normalized Dr for detected rings
				int mtIdx = Markers[i].TypeId;
				int NormIdx = Markers[i].GetTRingNum()-1-j;
				int TmpNormIdx = Markers[i].TmpRingsNum-1;
				float NormDr = Markers[i].GetTmpRingRadius(TmpNormIdx);
				// == LOG DEBUG ===========================
				#ifdef RINGDETECTION_DEBUG
                CLOG(TRACE, Logger()) << format("  COMBINATION %d - NormRing Idx [%d] R [%3.1f]",j,NormIdx,NormDr);
				#endif
				// ========================================
				for (int n=0; n<Markers[i].TmpRingsNum; n++)
					TmpDr[j][n] = Markers[i].GetTmpRingRadius(n)/NormDr;
				// == LOG DEBUG ===========================
				#ifdef RINGDETECTION_DEBUG
				string msg = "  TmpDr  [";
				for (int n=0; n<Markers[i].TmpRingsNum; n++)
					msg += format(" %3.1f ",TmpDr[j][n]);
				msg += "]";
                CLOG(TRACE, Logger()) << msg;
                CLOG(TRACE, Logger()) << "  IDENTIFYING RINGS WITH Dr MATRIX (type " << mtIdx << ")";
				#endif
				// ========================================
				// Identify rings using the Dr matrix
				for (int m=0; m<Markers[i].TmpRingsNum; m++) {
					// == LOG DEBUG ===========================
					#ifdef RINGDETECTION_DEBUG
                    CLOG(TRACE, Logger()) << "  TmpRing " << m << " - TmpDr: " << TmpDr[j][m];
                    CLOG(TRACE, Logger()) << "  Cr Matrix:";
					for (int n=0; n<Markers[i].GetTRingNum(); n++) {
						string ms1 = "[ ";
						for (int r=0; r<Markers[i].GetTRingNum(); r++) {
							ms1 += format("%3.2f ",Markers[i].GetTDrMVal(n,r));
						}
						ms1 += "]";
                        CLOG(TRACE, Logger()) << "  " << ms1;
					}
					#endif
					// ========================================
					// Build thresholds from active Dr matrix row
					tvFloat DrmThr;
					DrmThr.resize(Markers[i].GetTRingNum()+1);
					DrmThr[0] = 0.0;
					DrmThr[Markers[i].GetTRingNum()] = 2*Markers[i].GetTDrMVal(NormIdx,Markers[i].GetTRingNum()-1);
					for (int n=0; n<Markers[i].GetTRingNum()-1; n++) {
						DrmThr[n+1] = (Markers[i].GetTDrMVal(NormIdx,n)+Markers[i].GetTDrMVal(NormIdx,n+1))/2.0f;
					}
					for (int n=0; n<Markers[i].GetTRingNum(); n++) {
						// TODO: Thresholds need to be calculated and used for identification
						float t1 = DrmThr[n];
						float t2 = DrmThr[n+1];
						// == LOG DEBUG ===========================
						#ifdef RINGDETECTION_DEBUG
						string msg = "";
						msg += format("  Checking Drm(%d,%d)=%1.3f - ",NormIdx,n,Markers[i].GetTDrMVal(NormIdx,n));
						msg += format("Thr(%1.3f,%1.3f) - ",t1,t2);
						#endif
						// ========================================
						if (TmpDr[j][m]>=t1 && TmpDr[j][m]<t2) {
							#ifdef RINGDETECTION_DEBUG
							msg += "IDENTIFIED: TmpRing(" << m << ") -> Ring(" << n << ")";
                            CLOG(TRACE, Logger()) << msg;
							#endif
							TmpIdx[j][m] = n;
							break;
						}
						// == LOG DEBUG ===========================
						#ifdef RINGDETECTION_DEBUG
						else {
                            CLOG(TRACE, Logger()) << "NO";
						}
						#endif
						// ========================================
					}
				}
				// == LOG DEBUG ===========================
				#ifdef RINGDETECTION_DEBUG
				string msg = "  TmpIdx [";
				for (int n=0; n<Markers[i].TmpRingsNum; n++)
					msg += format(" %d ",TmpIdx[j][n]);
				msg += "]";
                CLOG(TRACE, Logger()) << msg;
				#endif
				// ========================================
				// Calculate Dr confidence and update maximum index
				ConfDr = 0.0;
				for (int m=0; m<Markers[i].TmpRingsNum; m++)
					ConfDr += Markers[i].CalcRingDrCMNorm(TmpIdx[j][m],TmpDr[j][m]);
				ConfDr = ConfDr/Markers[i].TmpRingsNum;
				if (ConfDr>MaxCDr) {
					MaxCDr = ConfDr;
					MaxCDrIdx = j;
				}
				// == LOG DEBUG ===========================
				#ifdef RINGDETECTION_DEBUG
                CLOG(TRACE, Logger()) << format("  ConfDr [ Conf %1.3f Max %1.3f MaxIdx %d ]",ConfDr,MaxCDr,MaxCDrIdx);
				#endif
				// ========================================
			}
			// Copy the most likely configuration depending on Dr confidence
			Markers[i].RingNum = Markers[i].TmpRingsNum;
			for (j=0; j<Markers[i].TmpRingsNum; j++) {
				Markers[i].Rings[TmpIdx[MaxCDrIdx][j]] = Markers[i].TmpRings[j];
				Markers[i].ValidRings[TmpIdx[MaxCDrIdx][j]] = true;
			}
			Markers[i].CalcPosition();
			Markers[i].CalcRadius();
			Markers[i].CalcConfMeasures();
		}
		if (Markers[i].TmpRingsNum > markerExpRings) {
			// TODO: Implement this case
			// == LOG DEBUG ===========================
			#ifdef RINGDETECTION_DEBUG
            CLOG(TRACE, Logger()) << "TOO MUCH RINGS ASSOCIATED WITH MARKER: DISCARDED";
			#endif
			// ========================================
		}
	}
}

// Get the logger name
tCStr CLandingPad::Logger() {
    return LoggerName.c_str();
}

// Show information on landing pad detected characteristics
void CLandingPad::ShowLandingPadInfo() {
	int i;
    CLOG(INFO, Logger()) << format("LP INFO: %d Markers ===============================", NumMarkers);
	for (i=0; i<NumMarkers; i++) {
        CLOG(INFO, Logger()) << format("MARKER   %d ---------------------------------------", i);
		Markers[i].ShowMarkerInfo();
	}
}