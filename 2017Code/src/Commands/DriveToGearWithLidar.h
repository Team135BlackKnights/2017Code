#ifndef DriveToGearWithLidar_H
#define DriveToGearWithLidar_H

#include "../CommandBase.h"
#include <Timer.h>

class DriveToGearWithLidar : public CommandBase {
private:
	double driveTrainMotorPower;
	bool rightGear;

	bool requiresDriveTrainSubsystem = false;

	double gyroAngle = 0.0;
	bool zeroGyro = false;

	bool initializeI2CMultiplxerChannelToOpen = false;
	bool configureLidar = false;
	bool receiveLidarUpperByte = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	double currentLidarValueIN = 0.0;

	static constexpr double DISTANCE_TO_TRAVEL_TO_START_DETECTING_AIRSHIP = 0.0;
	bool startLidarDetectingGearPeg = false;
	double savedLidarTransparentAirshipValue = 0.0;
	bool waitingForGearPeg = false;
	static constexpr double DISTANCE_DROP_OF_LIDAR_FROM_ARISHIP_TO_GEAR_PEG_IN = 30.0;
	bool lidarDetectsGearPeg = false;
	bool traveledToTurningRadiusOfRobot = false;

	static const bool RIGHT_GEAR = true;
	static const bool LEFT_GEAR = !RIGHT_GEAR;

	frc::Timer* timer;
	double currentTimerValue = 0.0;
	static constexpr double TIME_TO_WAIT_TO_REACT_TO_GEAR_PEG = .05;

	bool initializeGearPegLidarTimer = false;

	static constexpr double DISTANCE_BETWEEN_LIDAR_AND_TURNING_POINT_OF_ROBOT = 12.0;

	bool configureInitialDistanceBeforeTravelingExtraDistance = false;
	double initialDistanceTraveledBeforeTravelingExtraDistance = 0.0;
	double currentDistanceTraveledWhileTravelingExtraDistance = 0.0;
	double differenceBetweenCurrentAndInitialDistanceWhileTravelingExtraDistance = 0.0;
	static constexpr double DRIVE_TRAIN_MOTOR_POWER_FOR_EXTRA_DISTANCE = .25;

public:
	DriveToGearWithLidar(double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveToGearWithLidar_H
