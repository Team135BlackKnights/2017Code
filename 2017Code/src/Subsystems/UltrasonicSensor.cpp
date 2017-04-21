#include "UltrasonicSensor.h"
#include "../RobotMap.h"
#include "Commands/ReadUltrasonicSensorValue.h"
#include <math.h>

UltrasonicSensor::UltrasonicSensor() : Subsystem("UltrasonicSensor") {

}

void UltrasonicSensor::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new ReadUltrasonicSensorValue());
}

void UltrasonicSensor::InitializeUltrasonicSensors() {
	gearUltrasonicPingSignal = new frc::DigitalOutput(GEAR_ULTRASONIC_DIO_PING_PORT);
	gearUltrasonicEchoSignal = new frc::DigitalInput(GEAR_ULTRASONIC_DIO_ECHO_PORT);
	gearUltrasonicSensor = new frc::Ultrasonic(gearUltrasonicPingSignal, gearUltrasonicEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	gearUltrasonicSensor->SetAutomaticMode(true);

	rightSideUltrasonicSensorPingSignal = new frc::DigitalOutput(RIGHT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT);
	rightSideUltrasonicSensorEchoSignal = new frc::DigitalInput(RIGHT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT);
	rightSideUltrasonicSensor = new frc::Ultrasonic(rightSideUltrasonicSensorPingSignal, rightSideUltrasonicSensorEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	rightSideUltrasonicSensor->SetAutomaticMode(true);

	leftSideUltrasonicSensorPingSignal = new frc::DigitalOutput(LEFT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT);
	leftSideUltrasonicSensorEchoSignal = new frc::DigitalInput(LEFT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT);
	leftSideUltrasonicSensor = new frc::Ultrasonic(leftSideUltrasonicSensorPingSignal, leftSideUltrasonicSensorEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	leftSideUltrasonicSensor->SetAutomaticMode(true);
}

double UltrasonicSensor::GetGearUltrasonicSensorValueInches() {
	return gearUltrasonicSensor->GetRangeInches();
}

double UltrasonicSensor::GetSideUltrasonicSensorValueInches(bool sideUltrasonicSensor) {
	if (sideUltrasonicSensor == RIGHT_SIDE_ULTRASONIC_SENSOR) {
		sideUltrasonicSensorValue = rightSideUltrasonicSensor->GetRangeInches();
	}
	else if (sideUltrasonicSensor == LEFT_SIDE_ULTRASONIC_SENSOR) {
		sideUltrasonicSensorValue = leftSideUltrasonicSensor->GetRangeInches();
	}
	return sideUltrasonicSensorValue;
}

double UltrasonicSensor::GetActualDistanceFromGuardrail(double ultrasonicSensorValue, double gyroAngle) {
	gyroAngleRadians = (gyroAngle * DEGREES_TO_RADIANS_CONSTANT);
	actualDistanceFromGuardrail = (cos(gyroAngleRadians) * ultrasonicSensorValue);
	return actualDistanceFromGuardrail;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
