// Definitions.h
// ARGOS Vision software application-wide definitions
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   19.09.2013
// Description: This header file includes all the application-wide definitions which configure
//              the application. Most of these definitions are temporarily used before actually
//              implementing the related functionality in classes.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

// -- Application configuration -------------------------------------------------------------------

#define CAPTUREWINDOW_WIDTH -1			// Video frame window width. Set to -1 to set to the frame size

// -- Command-line interface ----------------------------------------------------------------------

#define MIN_ARGS 0						// Minimum allowed number of command-line input arguments
#define MAX_ARGS 1						// Maximum allowed number of command-line input arguments

// -- Processing options --------------------------------------------------------------------------

//#define PROCESSING_OPTIMIZED			// Define (uncomment) to use the optimized procedure for filtering contours
//#define PROCESSING_RETINEX			// Define (uncomment) to apply Retinex algorithm for preprocessing
//#define PROCESSING_TANTRIGGS			// Define (uncomment) to use Tan-Triggs preprocessing algorithm
//#define PROCESSING_SELFQUOTIENT		// Define (uncomment) to use Self-Quotient Image preprocessing algorithm
#define PROCESSING_MODSELFQUOTIENT		// Define (uncomment) to use Self-Quotient Image preprocessing algorithm

// -- Processing helper definitions ---------------------------------------------------------------

// M_PI is a POSIX definition
#ifndef M_PI
// Macro definition for Pi
#define M_PI 3.14159265358979323846
#endif

// -- OpenCV helper definitions -------------------------------------------------------------------

#define CONTOURINDEX_PREV   1			// Hierarchy index storing previous same-level contour
#define CONTOURINDEX_NEXT   0			// Hierarchy index storing next same-level contour
#define CONTOURINDEX_CHILD  2			// Hierarchy index storing first child contour
#define CONTOURINDEX_PARENT 3			// Hierarchy index storing parent contour

// Contour hierarchy testing definitions
// h[0] - Next same level contour
// h[1] - Previous same level contour
// h[2] - First child contour
// h[3] - Parent contour

// Contours with no parent (top-level) and no child (no hole)
#define TOPLEVEL_NO_HOLE(h) ((((h[2]==-1) && (h[3]==-1)) ? 1 : 0))
// Contours with childs (holes) but no parent (top-level)
#define TOPLEVEL_WITH_HOLE(h) ((((h[2]!=-1) && (h[3]==-1)) ? 1 : 0))
// ???
#define IS_CC(h) ((h[3]==-1 ? 0 : 1 ))
// Contours that are unique holes without inner contours (no siblings, no child)
#define ONLY_HOLE(h) ((((h[0]==-1)  &&(h[1]==-1) && (h[2]==-1)) ? 1 : 0))

