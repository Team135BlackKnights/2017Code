#include "DriveShooterHood.h"

DriveShooterHood::DriveShooterHood(bool driveUpwards) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooterHood.get());
	this->driveUpwards = driveUpwards;
}

// Called just before this Command runs the first time
void DriveShooterHood::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveShooterHood::Execute() {
	if (this->driveUpwards) {
		CommandBase::shooterHood->DriveShooterHoodMotor(SHOOTER_HOOD_MOTOR_POWER);
	}
	else if (this->driveUpwards == false) {
		CommandBase::shooterHood->DriveShooterHoodMotor(-SHOOTER_HOOD_MOTOR_POWER);
	}
	else {
		CommandBase::shooterHood->DriveShooterHoodMotor(0.0);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveShooterHood::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveShooterHood::End() {
	CommandBase::shooterHood->DriveShooterHoodMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveShooterHood::Interrupted() {
	End();
}
