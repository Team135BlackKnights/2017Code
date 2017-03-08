#ifndef TurnOneSideOfRobotAngle_H
#define TurnOneSideOfRobotAngle_H

#include "../CommandBase.h"

class TurnOneSideOfRobotAngle : public CommandBase {
private:
	double desiredAngle;
	bool driveRightSide;
	double motorPower;

	bool zeroGyro = false;
	double currentGyroAngle = 0.0;
	bool turnedDesiredAngle = false;
public:
	TurnOneSideOfRobotAngle(double, bool, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TurnOneSideOfRobotAngle_H
