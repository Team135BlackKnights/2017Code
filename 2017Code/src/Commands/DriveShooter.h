#ifndef DriveShooter_H
#define DriveShooter_H

#include "../CommandBase.h"

class DriveShooter : public CommandBase {
private:
	static constexpr double SHOOTER_BACKWARDS_VOLTAGE = 8.0;
	double setpointRPM = 0.0;
	double desiredShooterVoltage = 0.0;

	int shooterMotorRPM = 0;
	int shooterMotorNUPer100Ms = 0;

	bool initializePID = false;
	bool initializeVoltageMode = false;

	bool shooterForwardsButtonPressed = false;
	bool shooterBackwardsButtonPressed = false;

	static constexpr double SHOOTER_BACKWARDS_MOTOR_VOLTAGE = 7.0;

	static constexpr double MAX_PERCENT_OF_SETPOINT_TO_RAMP_VOLTAGE = .65;

	double shooterOutputCurrent = 0.0;
public:
	DriveShooter();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveShooter_H
