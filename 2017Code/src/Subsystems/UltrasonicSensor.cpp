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
	gearUltrasonicSensor->SetAutomaticMode(false);

	rightSideUltrasonicSensorPingSignal = new frc::DigitalOutput(RIGHT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT);
	rightSideUltrasonicSensorEchoSignal = new frc::DigitalInput(RIGHT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT);
	rightSideUltrasonicSensor = new frc::Ultrasonic(rightSideUltrasonicSensorPingSignal, rightSideUltrasonicSensorEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	rightSideUltrasonicSensor->SetAutomaticMode(false);

	leftSideUltrasonicSensorPingSignal = new frc::DigitalOutput(LEFT_SIDE_ULTRASONIC_SENSOR_PING_SIGNAL_PORT);
	leftSideUltrasonicSensorEchoSignal = new frc::DigitalInput(LEFT_SIDE_ULTRASONIC_SENSOR_ECHO_SIGNAL_PORT);
	leftSideUltrasonicSensor = new frc::Ultrasonic(leftSideUltrasonicSensorPingSignal, leftSideUltrasonicSensorEchoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	leftSideUltrasonicSensor->SetAutomaticMode(false);
}

void UltrasonicSensor::PingSideUltrasonicSensor(bool rightUltrasonic) {
	if (rightUltrasonic) {
		rightSideUltrasonicSensor->Ping();
	}
	else if (rightUltrasonic == false) {
		leftSideUltrasonicSensor->Ping();
	}
}

void UltrasonicSensor::PingGearUltrasonicSensor() {
	gearUltrasonicSensor->Ping();
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

double UltrasonicSensor::GetAngleToTurnToAlignWithGuardRail(double frontUltrasonicSensorValue, double backUltrasonicSensorValue, bool rightHopper) {
	//std::cout << "Front Ultrasonic Sensor Value: " << frontUltrasonicSensorValue << std::endl;
	//std::cout << "Back Ultrasonic Sensor Value: " << backUltrasonicSensorValue << std::endl;
	convertedBackUltrasonicSensorValue = (backUltrasonicSensorValue + DISTANCE_WIDTH_WISE_FRONT_AND_BACK_ULTRASONIC_SENSORS_ARE_APART);
	if (rightHopper) {
		differenceBetweenFrontAndBackUltrasonicSensorValues = (backUltrasonicSensorValue - frontUltrasonicSensorValue);
	}
	else if (rightHopper == false) {
		differenceBetweenFrontAndBackUltrasonicSensorValues = (frontUltrasonicSensorValue - backUltrasonicSensorValue);
	}
	//std::cout << "Difference Between Two Ultrasonic Sensor Values: " << differenceBetweenFrontAndBackUltrasonicSensorValues << std::endl;
	desiredAngleToTurnRadians = atan2(differenceBetweenFrontAndBackUltrasonicSensorValues, DISTANCE_BETWEEN_FRONT_AND_BACK_ULTRASONIC_SENSORS);
	//std::cout << "Angle To Turn Radians: " << desiredAngleToTurnRadians << std::endl;
	desiredAngleToTurnDegrees = (desiredAngleToTurnRadians * RADIANS_TO_DERGREES_CONSTANT);
	//std::cout << "Desired Angle To Turn Degrees: " << desiredAngleToTurnDegrees << std::endl;
	return desiredAngleToTurnDegrees;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
