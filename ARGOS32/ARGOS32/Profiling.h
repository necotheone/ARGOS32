// Profiling.h
// ARGOS Vision software prifiling helper functions
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   19.09.2013
// Description: This header file implements utility functions for profiling ARGOS Vision related
//              code. The implementation depends on target system for getting timing information
//              and conversion.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

// -- System-related timing libraries -------------------------------------------------------------

#include <ctime>

// -- Profiling definitions -----------------------------------------------------------------------

// Timers
#define PROFILE_TIMER1   0
#define PROFILE_TIMER2   1
#define PROFILE_TIMERNUM 2			// Number of timers available for profiling
// Timing variables
#define PROFILE_STARTTIME  0		// Index for variable holding starting time
#define PROFILE_STOPTIME   1		// Index for variable holding stop time
#define PROFILE_TIME       2		// Index for variable holding measured time

// -- Profiling variables -------------------------------------------------------------------------

float profiling_t[PROFILE_TIMERNUM][3];

// -- Profiling functions -------------------------------------------------------------------------

inline void ProfilingStart(int timer) {
	profiling_t[timer][PROFILE_STARTTIME] = (float) clock()/CLOCKS_PER_SEC;
}

inline void ProfilingStop(int timer) {
	profiling_t[timer][PROFILE_STOPTIME] = (float) clock()/CLOCKS_PER_SEC;
	profiling_t[timer][PROFILE_TIME] = profiling_t[timer][PROFILE_STOPTIME]-profiling_t[timer][PROFILE_STARTTIME];
}

inline float ProfilingGetTime(int timer) {
	return profiling_t[timer][PROFILE_TIME];
}

inline float ProfilingGetTimeMs(int timer) {
	return profiling_t[timer][PROFILE_TIME]*1000.0;
}


