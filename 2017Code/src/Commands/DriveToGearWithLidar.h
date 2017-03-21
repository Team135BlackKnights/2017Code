#ifndef DriveToGearWithLidar_H
#define DriveToGearWithLidar_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveToGearWithLidar : public CommandBase {
private:
	double desiredDistanceToTravel;
	double driveTrainMotorPower;
	bool rightGear;

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

	bool initializeI2CMultiplxerChannelToOpen = false;
	bool configureLidar = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	double currentLidarValueIN = 0.0;

	static constexpr double DISTANCE_TO_TRAVEL_TO_START_DETECTING_AIRSHIP = 0.0;
	bool startLidarDetectingGearPeg = false;
	double savedLidarTransparentAirshipValue = 0.0;
	bool waitingForGearPeg = false;
	static constexpr double DISTANCE_DROP_OF_LIDAR_FROM_ARISHIP_TO_GEAR_PEG_IN = 20.0;
	bool lidarDetectsGearPeg = false;

	static const bool RIGHT_GEAR = false;
	static const bool LEFT_GEAR = !RIGHT_GEAR;

public:
	DriveToGearWithLidar(double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveToGearWithLidar_H
