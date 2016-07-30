// Log.h
// Logging library
// ------------------------------------------------------------------------------------------------
// Author: David Pérez-Piñar
// Rev.:   1.0
// Date:   22.09.2013
// Description:  This header file contains the required class templates for the implementation of a
//               lightweight, thread-safe, type-safe file logging library based on streams.
// ------------------------------------------------------------------------------------------------
// Instructions: Using the logging library requires configuring the log file and the log level.
//               The following statements show an initialization example:
//
//                 FILELog::ReportingLevel() = logDEBUG3;
//                 FILE* log_fd = fopen( "mylogfile.txt", "w" );
//                 Output2FILE::Stream() = log_fd;
//
//               If the stream is not configured, log is sent by default to console (stderr).
//               The library allows sending the output both to the console and to a file.
//               Console destination stream stderr by default, but it can be configured to
//               other standard streams; for example:
//
//                 Output2FILE::Console() = stdout;
//
//               The primary destination for log messages is the stream defined in Stream().
//               If both streams are set to the same target (for example, stderr) then only
//               one message is printed out. On the other hand, if they are different streams
//               the same log messages are sent to both destinations. It is possible to
//               enable and disable the console output; for example, the following statement
//               disables the console output:
//
//                 Output2FILE::ConsoleActive() = false;
//
//               The primary log output defined in Stream() is always active and cannot be
//               disabled.
//
//               Logging is active for the specified reporting level and for higher levels.
//               To add some logged information, the FILE_LOG() macro is used; for example:
//
//                 FILE_LOG(logDEBUG) << "A loop with " << count << " iterations";
//
//               The macro requires the log level as an argument. The log output includes the date
//               and time first, followed by the log level and the logged message fed through
//               stream operands. The library automatically adds a CR ('\n') at the end of each
//               FILE_LOG() call. As stated before, each FILE_LOG() statement will send the log
//               message to the Stream() destination and (if enabled) to the Console() destination.
//
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __LOG_H__
#define __LOG_H__

// -- External libraries --------------------------------------------------------------------------

#include <sstream>
#include <string>
#include <stdio.h>

// -- Definitions ---------------------------------------------------------------------------------

inline std::string NowTime();

enum TLogLevel {logERROR, logWARNING, logINFO, logINFO1, logINFO2, logINFO3, logINFO4, logDEBUG, logDEBUG1, logDEBUG2, logDEBUG3, logDEBUG4};

// -- Log template class --------------------------------------------------------------------------

template <typename T>
class Log {
public:
    Log();
    virtual ~Log();
    std::ostringstream& Get(TLogLevel level = logINFO);
public:
    static TLogLevel& ReportingLevel();
    static std::string ToString(TLogLevel level);
    static TLogLevel FromString(const std::string& level);
protected:
    std::ostringstream os;
private:
    Log(const Log&);
    Log& operator =(const Log&);
};

template <typename T>
Log<T>::Log() {
}

template <typename T>
std::ostringstream& Log<T>::Get(TLogLevel level) {
    os << "- " << NowTime();
    os << " " << ToString(level) << ": ";
    //os << std::string(level > logDEBUG ? level - logDEBUG : 0, '\t');
    return os;
}

template <typename T>
Log<T>::~Log() {
    os << std::endl;
    T::Output(os.str());
}

template <typename T>
TLogLevel& Log<T>::ReportingLevel() {
    static TLogLevel reportingLevel = logDEBUG4;
    return reportingLevel;
}

template <typename T>
std::string Log<T>::ToString(TLogLevel level) {
	static const char* const buffer[] = {"ERROR  ", "WARNING", "INFO   ", "INFO1  ", "INFO2  ", "INFO3  ", "INFO4  ", "DEBUG  ", "DEBUG1 ", "DEBUG2 ", "DEBUG3 ", "DEBUG4 "};
    return buffer[level];
}

template <typename T>
TLogLevel Log<T>::FromString(const std::string& level) {
    if (level == "DEBUG4")
        return logDEBUG4;
    if (level == "DEBUG3")
        return logDEBUG3;
    if (level == "DEBUG2")
        return logDEBUG2;
    if (level == "DEBUG1")
        return logDEBUG1;
    if (level == "DEBUG")
        return logDEBUG;
    if (level == "INFO")
        return logINFO;
    if (level == "WARNING")
        return logWARNING;
    if (level == "ERROR")
        return logERROR;
    Log<T>().Get(logWARNING) << "Unknown logging level '" << level << "'. Using INFO level as default.";
    return logINFO;
}

// -- Output2FILE class ---------------------------------------------------------------------------

class Output2FILE {
public:
    static FILE*& Stream();
	static FILE*& Console();
	static bool   ConsoleActive();
    static void   Output(const std::string& msg);
};

inline FILE*& Output2FILE::Stream() {
    static FILE* pStream = stderr;
    return pStream;
}

inline FILE*& Output2FILE::Console() {
    static FILE* pConsole = stderr;
    return pConsole;
}

inline bool Output2FILE::ConsoleActive() {
	static bool active = false;
	return active;
}

inline void Output2FILE::Output(const std::string& msg) {   
    FILE* pStream = Stream();
	FILE* pConsole = Console();
	bool  cActive = ConsoleActive();
    if (!pStream && !pConsole)
        return;
	if (pStream) {
		fprintf(pStream, "%s", msg.c_str());
		fflush(pStream);
	}
	if (pConsole && cActive && (pConsole!=pStream)) {
		fprintf(pConsole, "%s", msg.c_str());
		fflush(pConsole);
	}
}

// -- Definitions for system-dependent compilation ------------------------------------------------

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#   if defined (BUILDING_FILELOG_DLL)
#       define FILELOG_DECLSPEC   __declspec (dllexport)
#   elif defined (USING_FILELOG_DLL)
#       define FILELOG_DECLSPEC   __declspec (dllimport)
#   else
#       define FILELOG_DECLSPEC
#   endif // BUILDING_DBSIMPLE_DLL
#else
#   define FILELOG_DECLSPEC
#endif // _WIN32

// -- FILELog class based on Log template ---------------------------------------------------------

class FILELOG_DECLSPEC FILELog : public Log<Output2FILE> {};
//typedef Log<Output2FILE> FILELog;

#ifndef FILELOG_MAX_LEVEL
#define FILELOG_MAX_LEVEL logDEBUG4
#endif

#define FILE_LOG(level) \
    if (level > FILELOG_MAX_LEVEL) ;\
    else if (level > FILELog::ReportingLevel() || (!Output2FILE::Stream() && (!Output2FILE::Console() || !Output2FILE::ConsoleActive()))) ; \
    else FILELog().Get(level)

// -- System-dependent time functions -------------------------------------------------------------

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)

#include <windows.h>

inline std::string NowTime() {
    const int MAX_LEN = 200;
    char buffer[MAX_LEN];
    if (GetTimeFormatA(LOCALE_USER_DEFAULT, 0, 0, 
            "HH':'mm':'ss", buffer, MAX_LEN) == 0)
        return "Error in NowTime()";

    char result[100] = {0};
    static DWORD first = GetTickCount();
    std::sprintf(result, "%s.%03ld", buffer, (long)(GetTickCount() - first) % 1000); 
    return result;
}

#else

#include <sys/time.h>

inline std::string NowTime() {
    char buffer[11];
    time_t t;
    time(&t);
    tm r = {0};
    strftime(buffer, sizeof(buffer), "%X", localtime_r(&t, &r));
    struct timeval tv;
    gettimeofday(&tv, 0);
    char result[100] = {0};
    std::sprintf(result, "%s.%03ld", buffer, (long)tv.tv_usec / 1000); 
    return result;
}

#endif //WIN32

#endif //__LOG_H__
