#ifndef DriveStraightGivenLidarValue_H
#define DriveStraightGivenLidarValue_H

#include "../CommandBase.h"

class DriveStraightGivenLidarValue : public CommandBase {
private:
	double driveTrainMotorPower;

	bool getDesiredDistanceToTravel = false;
	double frontUltrasonicSensorValue = 0.0;
	double desiredDistanceToTravel = 0.0;

	bool initializeDistanceTraveled = false;
	double initialDistanceTraveled = 0.0;
	double currentDistanceTraveled = 0.0;
	double differenceBetweenCurrentAndInitialDistance = 0.0;

	bool zeroGyro = false;
	double currentGyroAngle = 0.0;

	bool droveToDistance = false;
public:
	DriveStraightGivenLidarValue(double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveStraightGivenLidarValue_H
