#ifndef AutoDriveShooter_H
#define AutoDriveShooter_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveShooter : public CommandBase {
private:
	int setpointRPM;
	int currentShooterWheelRPM = 0;

	bool beginTimerToLevelOutSetpoint = false;
	bool moveAgitatorToShootFuel = false;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double WAIT_TIME_FOR_SHOOTER = .25;

	static constexpr double AGITATOR_MOTOR_POWER = .5;
	double agitatorCurrent = 0.0;
	static constexpr double THRESHOLD_AGITATOR_CURRENT_DRAW = 20.0;

	bool startDrivingAgitatorBackwards = false;
	static constexpr double TIME_TO_MOVE_AGITATOR_BACKWARDS = .5;
public:
	AutoDriveShooter(int);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveShooter_H
