#ifndef TurnDriveTrainAngle_H
#define TurnDriveTrainAngle_H

#include "../CommandBase.h"
#include <Timer.h>

class TurnDriveTrainAngle : public CommandBase {
private:
	double desiredAngleToTurn;
	double motorPower;
	bool turnRight;
	//bool setTimeout;

	bool zeroedGyro = false;

	double currenGyroAngle = 0.0;

	bool turnAngleComplete = false;

	bool requiresDriveTrain = false;

	frc::Timer* timer;
	bool initializeTimer = false;
	double timerValue = 0.0;
	static constexpr double TURN_ROBOT_INTO_HOPPER_TIMEOUT = 1.0;
public:
	TurnDriveTrainAngle(double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // TurnDriveTrainAngle_H
