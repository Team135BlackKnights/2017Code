#ifndef AutoDriveAgitator_H
#define AutoDriveAgitator_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveAgitator : public CommandBase {
private:
	bool shooterUpToSpeed = false;
	bool turnOffCollection = false;

	static constexpr double AGITATOR_MOTOR_POWER = .36;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double TIME_TO_WAIT_FOR_FUEL_TO_SHOOT = 7.0;

	bool startTimer = false;
	bool doneDrivingAgitator = false;
public:
	AutoDriveAgitator();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveAgitator_H
