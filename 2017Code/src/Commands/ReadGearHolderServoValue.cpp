#include "ReadGearHolderServoValue.h"

ReadGearHolderServoValue::ReadGearHolderServoValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::gearHolder.get());
}

// Called just before this Command runs the first time
void ReadGearHolderServoValue::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void ReadGearHolderServoValue::Execute() {
	servoPosition = CommandBase::gearHolder->GetGearHolderServoValue();
	std::cout << "Gear Holder Servo Position: " << servoPosition << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool ReadGearHolderServoValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadGearHolderServoValue::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadGearHolderServoValue::Interrupted() {

}
