#include "DriveGearHolder.h"

DriveGearHolder::DriveGearHolder(bool driveUpwards) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::gearHolder.get());
	this->driveUpwards = driveUpwards;
}

// Called just before this Command runs the first time
void DriveGearHolder::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveGearHolder::Execute() {
	if (driveUpwards) {
		CommandBase::gearHolder->DriveGearHolder(GEAR_HOLDER_MOTOR_POWER);
	}
	else if (driveUpwards == false) {
		CommandBase::gearHolder->DriveGearHolder(-GEAR_HOLDER_MOTOR_POWER);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveGearHolder::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveGearHolder::End() {
	CommandBase::gearHolder->DriveGearHolder(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveGearHolder::Interrupted() {
	End();
}
