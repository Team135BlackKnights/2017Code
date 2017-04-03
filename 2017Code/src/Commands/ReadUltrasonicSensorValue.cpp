#include "ReadUltrasonicSensorValue.h"

ReadUltrasonicSensorValue::ReadUltrasonicSensorValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::ultrasonicSensor.get());
}

// Called just before this Command runs the first time
void ReadUltrasonicSensorValue::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ReadUltrasonicSensorValue::Execute() {
	leftUltrasonicSensorValueInches = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::LEFT_ULTRASONIC_SENSOR);
	rightUltrasonicSensorValueInches = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::RIGHT_ULTRASONIC_SENSOR);
	frc::SmartDashboard::PutNumber("Left Ultrasonic Sensor Value", leftUltrasonicSensorValueInches);
	frc::SmartDashboard::PutNumber("Right Ultrasonic Sensor Value", rightUltrasonicSensorValueInches);
}

// Make this return true when this Command no longer needs to run execute()
bool ReadUltrasonicSensorValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadUltrasonicSensorValue::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadUltrasonicSensorValue::Interrupted() {

}
