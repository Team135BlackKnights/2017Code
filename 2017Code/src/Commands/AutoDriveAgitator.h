#ifndef AutoDriveAgitator_H
#define AutoDriveAgitator_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveAgitator : public CommandBase {
private:
	double timeToDriveAgitator;

	bool shooterUpToSpeed = false;
	bool turnOffCollection = false;

	static constexpr double AGITATOR_MOTOR_POWER = 1.0;
	static constexpr double AGITATOR_2_MOTOR_POWER = 1.0;

	frc::Timer* timer;
	double timerValue = 0.0;

	bool startTimer = false;
	bool doneDrivingAgitator = false;

	bool initializeContinueDrivingShooter = false;
public:
	AutoDriveAgitator(double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveAgitator_H
