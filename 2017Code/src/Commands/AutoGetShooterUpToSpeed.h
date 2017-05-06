#ifndef AutoGetShooterUpToSpeed_H
#define AutoGetShooterUpToSpeed_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoGetShooterUpToSpeed : public CommandBase {
private:
	double desiredShooterRPM;
	bool closeShotPID;

	bool initializePID = false;
	bool initializeVoltageMode = false;
	bool initializeBooleans = false;

	frc::Timer* timer;
	bool startTimer = false;
	double timerValue = 0.0;
	static constexpr double TIME_TO_WAIT_FOR_SHOOTER_TO_MAINTAIN_VELOCITY = .1;

	double currentShooterRPMValue = 0.0;

	bool maintainShooterRPM = false;
	bool shooterUpToSpeed = false;

	bool initializePIDSlot = false;
public:
	AutoGetShooterUpToSpeed(double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoGetShooterUpToSpeed_H
