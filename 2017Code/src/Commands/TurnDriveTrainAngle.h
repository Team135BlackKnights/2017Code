#ifndef TurnDriveTrainAngle_H
#define TurnDriveTrainAngle_H

#include "../CommandBase.h"

class TurnDriveTrainAngle : public CommandBase {
private:
	double desiredAngleToTurn;
	double motorPower;
	bool turnRight;

	bool zeroedGyro = false;

	double currenGyroAngle = 0.0;

	bool turnAngleComplete = false;

	bool requiresDriveTrain = false;
public:
	TurnDriveTrainAngle(double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TurnDriveTrainAngle_H
