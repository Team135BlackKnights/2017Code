#ifndef DriveWithJoysticks_H
#define DriveWithJoysticks_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveWithJoysticks : public CommandBase {
private:
	double leftJoystickValue = 0.0;
	double rightJoystickValue = 0.0;

	double actualUsedLeftJoystickValue = 0.0;
	double actualUsedRightJoystickValue = 0.0;

	static constexpr double POV_DRIVE_TRAIN_MOTOR_POWER = .65;

	double gyroAngle = 0.0;

	double throttleValue = 0.0;
	double povMotorPower = 0.0;

	int stopLeftMotorCounter = 1;
	bool resetLeftStopCounter = false;
	bool leftBrakeDrive = false;

	int stopRightMotorCounter = 1;
	bool resetRightStopCounter = false;
	bool rightBrakeDrive = false;

	static constexpr double STOP_DRIVE_TRAIN_MOTOR_POWER = .20;
	static constexpr double ZERO_DRIVE_TRAIN_MOTOR_POWER = 0.0;
	static const int BRAKING_COUNTER_DIVIDER = 2;

	/*frc::Timer* leftMotorTimer;
	double leftMotorTimerValue = 0.0;

	frc::Timer* rightMotorTimer;
	double rightMotorTimerValue = 0.0; */

public:
	DriveWithJoysticks();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveWithJoysticks_H
