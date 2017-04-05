#ifndef UltrasonicSensor_H
#define UltrasonicSensor_H

#include <Commands/Subsystem.h>
#include <Ultrasonic.h>
#include <DigitalOutput.h>
#include <DigitalInput.h>

class UltrasonicSensor : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::Ultrasonic* rightUltrasonicSensor;
	frc::DigitalOutput* rightUltrasonicPingSignal;
	frc::DigitalInput* rightUltrasonicEchoSignal;

	static const int RIGHT_ULTRASONIC_DIO_PING_PORT = 9;
	static const int RIGHT_ULTRASONIC_DIO_ECHO_PORT = 8;

	frc::Ultrasonic* leftUltrasonicSensor;
	frc::DigitalOutput* leftUltrasonicPingSignal;
	frc::DigitalInput* leftUltrasonicEchoSignal;

	static const int LEFT_ULTRASONIC_DIO_PING_PORT = 4;
	static const int LEFT_ULTRASONIC_DIO_ECHO_PORT = 5;

	double ultrasonicSensorValueIN = 0.0;

	double differenceBetweenUltrasonicSensorValues = 0.0;
	double desiredAngleToTurnRadians = 0.0;
	double desiredAngleToTurnDegrees = 0.0;

	static constexpr double CONVERT_RADIANS_TO_DEGREES = (180.0/M_PI);
	static constexpr double DISTANCE_BETWEEN_RIGHT_AND_LEFT_ULTRASONIC_SENSOR = 18.0;

	static const bool TURN_RIGHT = true;
	static const bool TURN_LEFT = !TURN_RIGHT;

	bool desiredDirectionToTurn = false;

public:
	UltrasonicSensor();
	void InitDefaultCommand();

	void InitializeUltrasonicSensors();

	double GetUltrasonicSensorValueInches(bool);

	double GetAngleToTurnForGear(double, double);

	static const bool RIGHT_ULTRASONIC_SENSOR = true;
	static const bool LEFT_ULTRASONIC_SENSOR = !RIGHT_ULTRASONIC_SENSOR;

	bool usingRightUltrasonicSensorForGearCamera = true;
};

#endif  // UltrasonicSensor_H
