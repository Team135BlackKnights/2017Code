#ifndef DriveDriveTrainCertainTime_H
#define DriveDriveTrainCertainTime_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveDriveTrainCertainTime : public CommandBase {
private:
	double desiredTimerValue;
	double motorPower;
	bool rammingIntoHopper;

	frc::Timer* timer;
	double currentTime = 0.0;

	bool restartTimer = false;

	bool doneDrivingWithTimer = false;

	double currentGyroAngle = 0.0;
	bool zeroGyro = false;


	bool initializeTimerForWaitingBeforeAdjustingAngle = false;
	bool waitBeforeAdjustingAngle = false;
	bool initializeTurnToAdjustAngle = false;
	bool turnRightToAdjustAngle = false;
	bool adjustedAngle = false;

	static constexpr double WAIT_TIME_BEFORE_ADJUSTING_ANGLE = .1;
	static constexpr double ADJUST_ANGLE_MOTOR_POWER = .6;
public:
	DriveDriveTrainCertainTime(double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveDriveTrainCertainTime_H
