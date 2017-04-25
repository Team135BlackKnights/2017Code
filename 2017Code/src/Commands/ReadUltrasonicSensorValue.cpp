#include "ReadUltrasonicSensorValue.h"

ReadUltrasonicSensorValue::ReadUltrasonicSensorValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::ultrasonicSensor.get());
}

// Called just before this Command runs the first time
void ReadUltrasonicSensorValue::Initialize() {
	readingUltrasonicSensorsCounter = 1;
}

// Called repeatedly when this Command is scheduled to run
void ReadUltrasonicSensorValue::Execute() {
	if (CommandBase::ultrasonicSensor->usingUltrasonicSensor) return;

	if ((readingUltrasonicSensorsCounter % 2) == 1) {
		CommandBase::ultrasonicSensor->PingSideUltrasonicSensor(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
		if (readingUltrasonicSensorsCounter > 1) {
			rightUltrasonicSensorValueInches = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();
		}
	}
	else if ((readingUltrasonicSensorsCounter % 2) == 0) {
		leftUltrasonicSensorValueInches = CommandBase::ultrasonicSensor->GetSideUltrasonicSensorValueInches(UltrasonicSensor::LEFT_SIDE_ULTRASONIC_SENSOR);
		CommandBase::ultrasonicSensor->PingGearUltrasonicSensor();
	}
	readingUltrasonicSensorsCounter++;

	frc::SmartDashboard::PutNumber("Back Left Side Ultrasonic Sensor Value", leftUltrasonicSensorValueInches);
	std::cout << "Back Left Ultrasonic Sensor Value: " << leftUltrasonicSensorValueInches << std::endl;

	frc::SmartDashboard::PutNumber("Front Left Ultrasonic Sensor Value", rightUltrasonicSensorValueInches);
	std::cout << "Front Left Ultrasonic Sensor Value: " << rightUltrasonicSensorValueInches << std::endl;
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
