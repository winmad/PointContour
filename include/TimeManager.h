#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <ctime>
#include <string>
#include <vector>
#include <cstdio>

using namespace std;

/*
 * Author:  David Robert Nadeau
 * Site:    http://NadeauSoftware.com/
 * License: Creative Commons Attribution 3.0 Unported License
 *          http://creativecommons.org/licenses/by/3.0/deed.en_US
 */
#if defined(_WIN32)
#include <Windows.h>

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>	/* POSIX flags */
#include <time.h>	/* clock_gettime(), time() */
#include <sys/time.h>	/* gethrtime(), gettimeofday() */

#if defined(__MACH__) && defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif

#else
#error "Unable to define getRealTime( ) for an unknown OS."
#endif

double getRealTime();

struct stopwatch_rec {
	double startTime;
	double cumulatedTime;
};

class TimeManager {
public:
	TimeManager();
	void PushCurrentTime();
	void PopAndDisplayTime(string textFormat);

	int StartStopWatch();
	void PauseStopWatch(int id);
	void ResumeStopWatch(int id);
	void DisplayStopWatch(int id, string textFormat);
private:
	vector<double> times;
	vector<stopwatch_rec> stopWatches;
};

#endif