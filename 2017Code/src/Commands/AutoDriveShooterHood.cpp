#include "AutoDriveShooterHood.h"

AutoDriveShooterHood::AutoDriveShooterHood(int desiredShooterHoodEncoderValue) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooterHood.get());

	this->desiredShooterHoodEncoderValue = desiredShooterHoodEncoderValue;
}

// Called just before this Command runs the first time
void AutoDriveShooterHood::Initialize() {
	shooterHoodAtDesiredEncoderPosition = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveShooterHood::Execute() {
	shooterHoodAtDesiredEncoderPosition = CommandBase::shooterHood->DriveShooterHoodToDesiredEncoderValue(this->desiredShooterHoodEncoderValue);
}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveShooterHood::IsFinished() {
	return shooterHoodAtDesiredEncoderPosition;
}

// Called once after isFinished returns true
void AutoDriveShooterHood::End() {
	shooterHoodAtDesiredEncoderPosition = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDriveShooterHood::Interrupted() {
	End();
}
