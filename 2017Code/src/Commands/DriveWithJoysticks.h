#ifndef DriveWithJoysticks_H
#define DriveWithJoysticks_H

#include "../CommandBase.h"

class DriveWithJoysticks : public CommandBase {
private:
	double leftJoystickValue = 0.0;
	double rightJoystickValue = 0.0;

	int leftEncoderValue = 0;
	int rightEncoderValue = 0;

	double leftEncoderDistanceTraveled = 0.0;
	double rightEncoderDistanceTraveled = 0.0;

	double navXAngle = 0.0;
public:
	DriveWithJoysticks();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveWithJoysticks_H
