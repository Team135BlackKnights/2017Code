#ifndef DriveToGearWithLidar_H
#define DriveToGearWithLidar_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveToGearWithLidar : public CommandBase {
private:
	double desiredDistanceToTravel;
	double driveTrainMotorPower;

	bool requiresDriveTrainSubsystem = false;

	bool initialDistanceConfigured = false;
	bool configureDesiredDistanceToTravel = false;
	double initialDistanceTraveled = 0.0;
	double differenceBetweenCurrentAndInitialDistanceTraveled = 0.0;
	double currentDistanceTraveled = 0.0;
	double actualDesiredDistanceValue = 0.0;
	bool distanceTraveledWithEncoder = false;

	double gyroAngle = 0.0;
	bool zeroGyro = false;

	bool initializeLidarSendLowSignal = false;
	bool startUsingLidar = false;
	static constexpr double DISTANCE_BEFORE_STARTING_USING_LIDAR = 30.0;
	bool lidarAtGearPeg = false;

public:
	DriveToGearWithLidar(double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveToGearWithLidar_H
