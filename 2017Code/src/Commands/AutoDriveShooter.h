#ifndef AutoDriveShooter_H
#define AutoDriveShooter_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveShooter : public CommandBase {
private:
	int setpointRPM;

	int shooterWheelRPM = 0;
	bool beginTimerToLevelOutSetpoint = false;
	bool moveAgitatorToShootFuel = false;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double WAIT_TIME_FOR_SHOOTER = .25;

	static constexpr double AGITATOR_MOTOR_POWER = .5;
public:
	AutoDriveShooter(int);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveShooter_H
