#ifndef TurnDriveTrainAngle_H
#define TurnDriveTrainAngle_H

#include "../CommandBase.h"

class TurnDriveTrainAngle : public CommandBase {
private:
	double desiredAngleToTurn;
	double motorPower;
	bool turnRight;

	double currentNavXAngle = 0.0;

	bool turnAngleComplete = false;
public:
	TurnDriveTrainAngle(double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TurnDriveTrainAngle_H
