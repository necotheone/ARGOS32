// DMIdentifiers.h
// Unique detection method identifiers list
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   22.09.2013
// Description: This header file defines the detection method identifiers available. Identifiers
//              are assigned to detection methods in their constructors; therefore, they are made
//              available here just to simplify the use of the detection engine (for example,
//              to select the detection method to be used from the main code).
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __DMIDENTIFIERS_H__
#define __DMIDENTIFIERS_H__

// Definition value corresponds to the detection method index in the CDetectionMethod vector.
// First identifier must be 0, and subsequent identifiers must be correlative. The DM_COUNT
// definition is the total number of available methods.

#define DM_COUNT             2				// Total number of available detection methods

#define DM_THRESHOLD         0				// Simple threshold-contour detection method
#define DM_MSQI              1              // Modified Self-Quotient Image and contour detection method

#endif