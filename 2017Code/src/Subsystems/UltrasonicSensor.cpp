#include "UltrasonicSensor.h"
#include "../RobotMap.h"
#include "Commands/ReadUltrasonicSensorValue.h"
#include <math.h>

UltrasonicSensor::UltrasonicSensor() : Subsystem("UltrasonicSensor") {

}

void UltrasonicSensor::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	//SetDefaultCommand(new ReadUltrasonicSensorValue());
}

void UltrasonicSensor::InitializeUltrasonicSensors() {
	rightUltrasonicPingSignal = new frc::DigitalOutput(RIGHT_ULTRASONIC_DIO_PING_PORT);
	rightUltrasonicEchoSignal = new frc::DigitalInput(RIGHT_ULTRASONIC_DIO_ECHO_PORT);
	rightUltrasonicSensor = new frc::Ultrasonic(rightUltrasonicPingSignal, rightUltrasonicEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	rightUltrasonicSensor->SetAutomaticMode(true);

	leftUltrasonicPingSignal = new frc::DigitalOutput(LEFT_ULTRASONIC_DIO_PING_PORT);
	leftUltrasonicEchoSignal = new frc::DigitalInput(LEFT_ULTRASONIC_DIO_ECHO_PORT);
	leftUltrasonicSensor = new frc::Ultrasonic(leftUltrasonicPingSignal, leftUltrasonicEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	leftUltrasonicSensor->SetAutomaticMode(true);
}

double UltrasonicSensor::GetUltrasonicSensorValueInches(bool rightUltrasonic) {
	if (rightUltrasonic) {
		ultrasonicSensorValueIN = rightUltrasonicSensor->GetRangeInches();
	}
	else if (rightUltrasonic == false) {
		ultrasonicSensorValueIN = leftUltrasonicSensor->GetRangeInches();
	}
	return ultrasonicSensorValueIN;
}

double UltrasonicSensor::GetAngleToTurnForGear(double rightUltrasonicSensorValue, double leftUltrasonicSensorValue) {
	differenceBetweenUltrasonicSensorValues = (leftUltrasonicSensorValue - rightUltrasonicSensorValue);
	desiredAngleToTurnRadians = atan2(differenceBetweenUltrasonicSensorValues, DISTANCE_BETWEEN_RIGHT_AND_LEFT_ULTRASONIC_SENSOR);
	desiredAngleToTurnDegrees = (desiredAngleToTurnRadians * CONVERT_RADIANS_TO_DEGREES);
	return desiredAngleToTurnDegrees;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
