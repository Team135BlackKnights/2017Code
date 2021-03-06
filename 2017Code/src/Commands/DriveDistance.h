#ifndef DriveDistance_H
#define DriveDistance_H

#include "../CommandBase.h"

class DriveDistance : public CommandBase {
private:
	double desiredDistanceToTravel;
	double motorPower;
	bool zeroDriveTrainEncoders;

	bool zeroedDriveTrainEncoders = false;

	double initialDistanceTraveled = 0.0;
	double measuredCurrentDistanceTraveled = 0.0;
	double actualCurrentDistanceTraveled = 0.0;
	bool measuredInitialDistanceTraveled = false;

	bool distanceTraveled = false;

	double gyroAngle = 0.0;
	bool zeroGyro = false;

	bool requiresDriveTrain = false;
public:
	DriveDistance(double, double, bool zeroDriveTrainEncoders = false);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveDistance_H
