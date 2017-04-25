#ifndef UltrasonicSensor_H
#define UltrasonicSensor_H

#include <Commands/Subsystem.h>
#include <Ultrasonic.h>
#include <DigitalOutput.h>
#include <DigitalInput.h>
#include <math.h>

class UltrasonicSensor : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::Ultrasonic* gearUltrasonicSensor;
	frc::DigitalOutput* gearUltrasonicPingSignal;
	frc::DigitalInput* gearUltrasonicEchoSignal;

	static const int GEAR_ULTRASONIC_DIO_PING_PORT = 9;
	static const int GEAR_ULTRASONIC_DIO_ECHO_PORT = 8;

	frc::Ultrasonic* rightSideUltrasonicSensor;
	frc::DigitalOutput* rightSideUltrasonicSensorPingSignal;
	frc::DigitalInput* rightSideUltrasonicSensorEchoSignal;

	static const int RIGHT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT = 10;
	static const int RIGHT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT = 11;

	frc::Ultrasonic* leftSideUltrasonicSensor;
	frc::DigitalOutput* leftSideUltrasonicSensorPingSignal;
	frc::DigitalInput* leftSideUltrasonicSensorEchoSignal;

	static const int LEFT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT = 4;
	static const int LEFT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT = 5;

	double sideUltrasonicSensorValue = 0.0;

	double gyroAngleRadians = 0.0;
	double actualDistanceFromGuardrail = 0.0;
	static constexpr double DEGREES_TO_RADIANS_CONSTANT = (M_PI/180.0);

public:
	UltrasonicSensor();
	void InitDefaultCommand();

	void InitializeUltrasonicSensors();

	void PingSideUltrasonicSensor(bool);
	void PingGearUltrasonicSensor();

	double GetGearUltrasonicSensorValueInches();
	double GetSideUltrasonicSensorValueInches(bool);

	double GetActualDistanceFromGuardrail(double, double);

	double GetAngleToTurnToAlignWithGuardRail(double, double, bool);

	double differenceBetweenFrontAndBackUltrasonicSensorValues = 0.0;
	static constexpr double DISTANCE_BETWEEN_FRONT_AND_BACK_ULTRASONIC_SENSORS = 28.0;
	static constexpr double DISTANCE_WIDTH_WISE_FRONT_AND_BACK_ULTRASONIC_SENSORS_ARE_APART = 1.5;
	double convertedBackUltrasonicSensorValue = 0.0;
	double desiredAngleToTurnRadians = 0.0;
	static constexpr double RADIANS_TO_DERGREES_CONSTANT = (180.0/M_PI);
	double desiredAngleToTurnDegrees = 0.0;

	static const bool RIGHT_SIDE_ULTRASONIC_SENSOR = true;
	static const bool LEFT_SIDE_ULTRASONIC_SENSOR = !RIGHT_SIDE_ULTRASONIC_SENSOR;

	bool usingRightUltrasonicSensorForGearCamera = false;

	bool usingUltrasonicSensor = false;
};

#endif  // UltrasonicSensor_H
