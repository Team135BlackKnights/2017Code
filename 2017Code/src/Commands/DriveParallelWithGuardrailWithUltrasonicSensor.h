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
	double differenceBetweenCurrentAndDesiredDistance = 0.0;

	static constexpr double THRESHOLD_ULTRASONIC_SENSOR_VALUE = 35.0;

	bool doneWithDrivingWithUltrasonicSensor = false;

	static const bool DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE = true;
	static const bool DRIVE_WITH_DISTANCE = !DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE;
	static const bool DRIVE_ENDING = DRIVE_WITH_DISTANCE;

	double acutalDistanceRobotIsFromWall = 0.0;

	bool zeroGyro = false;
	double gyroAngle = 0.0;

	static const bool CONFIGURE_INITIAL_DISTANCE = true;
	static const bool DONT_RECONFIGURE_INITIAL_DISTANCE = !CONFIGURE_INITIAL_DISTANCE;
	static const bool DRIVING_FORWARDS = true;
	static const bool DRIVING_BACKWARDS = false;
public:
	DriveParallelWithGuardrailWithUltrasonicSensor(double, double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveParallelWithGuardrailWithUltrasonicSensor_H
