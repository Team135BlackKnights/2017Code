#ifndef DriveDriveTrainCertainTime_H
#define DriveDriveTrainCertainTime_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveDriveTrainCertainTime : public CommandBase {
private:
	double desiredTimerValue;
	double motorPower;

	frc::Timer* timer;
	double currentTime = 0.0;

	bool restartTimer = false;

	bool doneDrivingWithTimer = false;
public:
	DriveDriveTrainCertainTime(double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveDriveTrainCertainTime_H
