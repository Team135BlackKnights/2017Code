#ifndef DriveBackwardsWithLidar_H
#define DriveBackwardsWithLidar_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveBackwardsWithLidar : public CommandBase {
private:
	double initialDriveTrainMotorPower;
	double desiredLidarValueToDriveUntil;
	double desiredInitialDistanceToTravel;

	bool initializeI2CMultiplexerChannel = false;
	bool configureLidar = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	double currentlidarValueIN = 0.0;

	bool startUsingLidarValues = false;

	bool robotAtDesiredLidarValue = false;

	double currentGyroAngle = 0.0;
	bool zeroGyroAngle = false;

	double differenceBetweenDesiredAndCurrentLidarValue = 0.0;
	static constexpr double DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_1 = 40.0;
	static constexpr double DIFFERENCE_BETWEEN_DESIRED_AND_CURRENT_LIDAR_VALUES_TO_DRIVE_SLOWER_PART_2 = 20.0;
	static constexpr double SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_1 = -.25;
	static constexpr double SLOWER_DRIVE_TRAIN_MOTOR_POWER_PART_2 = -.15;

	static constexpr double ESTIMATED_COAST_DISTANCE_IN = 1.0;

	static constexpr double DISTANCE_BETWEEN_LIDAR_AND_EDGE_OF_BUMPER = 5.0;

	bool initializedEncoderDistanceTravel = false;
	bool startUsingEncoderValues = false;
	double initialDistanceTraveled = 0.0;
	double currentDistanceTraveled = 0.0;
	double differenceBetweenInitialAndCurrentDistanceTraveled = 0.0;
	static constexpr double DISTANCE_TO_TRAVEL_SLOW_SPEED = 20.0;
	static constexpr double ENCODER_BASED_SLOW_MOTOR_POWER = -.45;
	bool startEncoderBasedSlowerMotorPower = false;
	double initialDistanceTraveledForSlowerMotorPower = 0.0;
	double differenceBetweenCurrentAndInitialEncoderValuesSlowerMotorPower = 0.0;
	bool robotTraveledDistanceWithEncoders = false;

	frc::Timer* timer;
	bool initializeTimer = false;
	double timerValue = 0.0;
	static constexpr double TIME_TO_WAIT_UNTIL_USING_ENCODER_VALUES = .4;

public:
	DriveBackwardsWithLidar(double, double, double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveBackwardsWithLidar_H
