#ifndef AlignRobotWithGuardrail_H
#define AlignRobotWithGuardrail_H

#include "../CommandBase.h"

class AlignRobotWithGuardrail : public CommandBase {
private:
	double driveTrainMotorPower;
	bool rightHopper;

	double initialFrontUltrasonicSensorValue = 0.0;
	double initialBackUltrasonicSensorValue = 0.0;

	bool getDesiredAngleToTurn = false;
	double desiredAngleToTurn = 0.0;

	bool zeroGyro = false;
	double currentGyroAngle = 0.0;

	bool alignedRobotWithGuardRail = false;

	bool pingedFrontUltrasonicSensor = false;
	bool pingedBackUltrasonicSensor = true;
public:
	AlignRobotWithGuardrail(double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AlignRobotWithGuardrail_H
