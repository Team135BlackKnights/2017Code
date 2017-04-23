#ifndef DriveParallelWithGuardrailWithUltrasonicSensor_H
#define DriveParallelWithGuardrailWithUltrasonicSensor_H

#include "../CommandBase.h"

class DriveParallelWithGuardrailWithUltrasonicSensor : public CommandBase {
private:
	double desiredDistanceToTravel;
	double distanceAwayFromGuardrailToDrive;
	double driveTrainMotorPower;
	bool rightHopperAndShoot;

	bool initializeIsAimingBoolean = false;

	double sideUltrasonicSensorValue = 0.0;

	bool initializeEncoderDriveDistance = false;
	double initialDistanceTraveled = 0.0;
	double currentDistanceTraveled = 0.0;
	double differenceBetweenInitialAndCurrentDistance = 0.0;

	bool doneWithDrivingWithUltrasonicSensor = false;

	double acutalDistanceRobotIsFromWall = 0.0;

	bool zeroGyro = false;
	double gyroAngle = 0.0;

	static const bool CONFIGURE_INITIAL_DISTANCE = true;
	static const bool DONT_RECONFIGURE_INITIAL_DISTANCE = !CONFIGURE_INITIAL_DISTANCE;
	static const bool DRIVING_FORWARDS = true;
	static const bool DRIVING_BACKWARDS = false;

	static constexpr double THRESHOLD_FOR_ADJUSTING_DRIVE_STRAIGHT_WITH_ULTRSONIC_SENSOR = .8;

	static constexpr double THRESHOLD_GYRO_ANGLE_FOR_ADJUSTING = 1.8;

	bool initializeInitialGyroAngle = false;
	double initialGyroAngleForAdjusting = 0.0;
	bool adjustRobotWithGyro = false;

	bool getInitialUltrasonicSensorValue = 0.0;
	double initialUltrasonicSensorValue = 0.0;

	bool initializeDistanceToTravel = false;
	double desiredDistanceToTravelFromLidar = 0.0;

	double frontUltrasonicSensorValue = 0.0;
public:
	DriveParallelWithGuardrailWithUltrasonicSensor(double, double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveParallelWithGuardrailWithUltrasonicSensor_H
