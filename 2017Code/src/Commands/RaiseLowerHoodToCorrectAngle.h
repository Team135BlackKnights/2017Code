#ifndef RaiseLowerHoodToCorrectAngle_H
#define RaiseLowerHoodToCorrectAngle_H

#include "../CommandBase.h"

class RaiseLowerHoodToCorrectAngle : public CommandBase {
private:
	bool closeShot;
	int shooterAngledPosition;
	double desiredHoodAngleToTurn = 0.0;

	bool resetLidarVariables = false;
	double lidarValue_CM = 0.0;

	bool calculatedDesiredAngle = false;
	bool driveHoodToDesiredAngle = false;

	static constexpr double SHOOTER_HOOD_MOTOR_POWER = 1.0;
public:
	RaiseLowerHoodToCorrectAngle(bool, int);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // RaiseLowerHoodToCorrectAngle_H
