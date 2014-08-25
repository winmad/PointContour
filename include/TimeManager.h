#ifndef TIME_MANAGER_H
#define TIME_MANAGER_H

#include <ctime>
#include <string>
#include <vector>
#include <cstdio>

using namespace std;

struct stopwatch_rec {
	clock_t startTime;
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
	vector<clock_t> times;
	vector<stopwatch_rec> stopWatches;
};

#endif