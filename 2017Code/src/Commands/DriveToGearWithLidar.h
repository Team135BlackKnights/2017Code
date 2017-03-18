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

	bool startReceivingLidarValues = false;
	double lidarValueIN = 0.0;

	static constexpr double DISTANCE_TO_START_GETTING_LIDAR_VALUES = 30.0;
	bool startGettingLidarValues = false;

	double savedFirstRoundLidarValue = 0.0;
	bool waitTimeForFirstRoundLidarValue = false;
	bool waitForLidarHighSignal = false;

	static constexpr double ADDED_DISTANCE_OF_LIDAR_THROUGH_AIRSHIP = 20.0;
	static constexpr double DISTANCE_DROP_FROM_AIRSHIP_TO_GEAR_PEG = 20.0;
	double savedSecondRoundOfLidarValue = 0.0;
	bool waitForSavingSecondRoundValue = false;
	bool waitForThirdRoundValue = false;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double TIME_TO_WAIT_TO_GET_ACCURATE_FIRST_ROUND_LIDAR_VALUE = .15;
	static constexpr double TIME_TO_WAIT_TO_GET_ACCURATE_LIDAR_VALUE_THROUGH_AIRSHIP = .2;

	int thirdRoundLidarValueCounter = 0;
	static const int THIRD_ROUND_COUNTER_MAX_VALUE = 2;
	bool lidarDetectedPeg = false;
public:
	DriveToGearWithLidar(double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveToGearWithLidar_H
