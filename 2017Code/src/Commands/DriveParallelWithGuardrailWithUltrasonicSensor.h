#ifndef DriveParallelWithGuardrailWithUltrasonicSensor_H
#define DriveParallelWithGuardrailWithUltrasonicSensor_H

#include "../CommandBase.h"

class DriveParallelWithGuardrailWithUltrasonicSensor : public CommandBase {
private:
	double desiredDistanceToTravel;
	double distanceAwayFromGuardrailToDrive;
	double driveTrainMotorPower;
	bool rightHopperAndShoot;

	double sideUltrasonicSensorValue = 0.0;

	bool initializeEncoderDriveDistance = false;
	double initialDistanceTraveled = 0.0;
	double currentDistanceTraveled = 0.0;
	double differenceBetweenCurrentAndDesiredDistance = 0.0;

	static constexpr double THRESHOLD_ULTRASONIC_SENSOR_VALUE = 35.0;

	bool doneWithDrivingWithUltrasonicSensor = false;

	static const bool DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE = true;
	static const bool DRIVE_WITH_DISTANCE = !DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE;
	static const bool DRIVE_ENDING = DRIVE_UNTIL_ULTRASONIC_SENSOR_IS_HIGH_VALUE;
public:
	DriveParallelWithGuardrailWithUltrasonicSensor(double, double, double, bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveParallelWithGuardrailWithUltrasonicSensor_H
