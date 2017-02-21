#include "DriveAgitator.h"

DriveAgitator::DriveAgitator(bool driveForwards) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::agitator.get());
	this->driveForwards = driveForwards;
}

// Called just before this Command runs the first time
void DriveAgitator::Initialize() {
	agitatorMotorSpeed = Preferences::GetInstance()->GetDouble("Agitator Motor Power", .35);
}

// Called repeatedly when this Command is scheduled to run
void DriveAgitator::Execute() {
	if (this->driveForwards) {
		CommandBase::agitator->DriveAgitator(agitatorMotorSpeed);
	}
	else if (this->driveForwards == false) {
		CommandBase::agitator->DriveAgitator(-agitatorMotorSpeed);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveAgitator::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveAgitator::End() {
	CommandBase::agitator->DriveAgitator(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveAgitator::Interrupted() {
	End();
}