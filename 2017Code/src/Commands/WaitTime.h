#ifndef WaitTime_H
#define WaitTime_H

#include "../CommandBase.h"
#include "Timer.h"

class WaitTime : public CommandBase {
private:
	double timeToWait;

	frc::Timer* timer;
	double currentTimerValue = 0.0;

	bool doneWaiting = false;
public:
	WaitTime(double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // WaitTime_H
