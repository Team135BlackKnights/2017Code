#include "UltrasonicSensor.h"
#include "../RobotMap.h"
#include "Commands/ReadUltrasonicSensorValue.h"

UltrasonicSensor::UltrasonicSensor() : Subsystem("UltrasonicSensor") {

}

void UltrasonicSensor::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	//SetDefaultCommand(new ReadUltrasonicSensorValue());
}

void UltrasonicSensor::InitializeUltrasonicSensor() {
	pingSignal = new frc::DigitalOutput(DIO_PING_PORT);
	echoSignal = new frc::DigitalInput(DIO_ECHO_PORT);
	ultrasonicSensor = new frc::Ultrasonic(pingSignal, echoSignal, frc::Ultrasonic::DistanceUnit::kInches);
	ultrasonicSensor->SetAutomaticMode(true);
}

double UltrasonicSensor::GetUltrasonicSensorValueInches() {
	return ultrasonicSensor->GetRangeInches();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
