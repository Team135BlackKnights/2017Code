#ifndef DriveWithJoysticks_H
#define DriveWithJoysticks_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveWithJoysticks : public CommandBase {
private:
	double leftJoystickValue = 0.0;
	double rightJoystickValue = 0.0;

	double convertedLeftJoystickValue = 0.0;
	double convertedRightJoystickValue = 0.0;

	double actualUsedLeftJoystickValue = 0.0;
	double actualUsedRightJoystickValue = 0.0;

	static constexpr double POV_DRIVE_TRAIN_MOTOR_POWER = .65;

	double gyroAngle = 0.0;

	bool throttleUp = false;
	double povMotorPower = 0.0;

	static constexpr double LOW_POV_DRIVE_TRAIN_MOTOR_POWER = .3;
	static constexpr double HIGH_POV_DRIVE_TRAIN_MOTOR_POWER = .7;


	bool rightDriveEncoderDetected = false;
	bool leftDriveEncoderDetected = false;

	double desiredMaxMotorPower = 0.0;

public:
	DriveWithJoysticks();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveWithJoysticks_H
