#ifndef AutoDriveShooter_H
#define AutoDriveShooter_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoDriveShooter : public CommandBase {
private:
	int setpointRPM;
	int currentShooterWheelRPM = 0;
	bool shooterVoltageMode = false;
	bool shooterPIDMode = false;

	bool beginTimerToLevelOutSetpoint = false;
	bool moveAgitatorToShootFuel = false;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double WAIT_TIME_FOR_SHOOTER = .15;
	static constexpr double WAIT_TIME_FOR_FUEL_TO_SHOOT = 5.5;

	static constexpr double AGITATOR_MOTOR_POWER = .4;
	bool startTimerForAgitator = false;

	bool shotFuelIntoBoiler = false;
public:
	AutoDriveShooter(int);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveShooter_H
