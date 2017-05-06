#include "TurnOneSideOfRobotAngle.h"

TurnOneSideOfRobotAngle::TurnOneSideOfRobotAngle(double desiredAngle, bool driveRightSide, double motorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());

	this->desiredAngle = desiredAngle;
	this->driveRightSide = driveRightSide;
	this->motorPower = motorPower;
}

// Called just before this Command runs the first time
void TurnOneSideOfRobotAngle::Initialize() {
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;
	turnedDesiredAngle = false;

	CommandBase::driveTrain->is_aiming = true;
}

// Called repeatedly when this Command is scheduled to run
void TurnOneSideOfRobotAngle::Execute() {
	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	currentGyroAngle = (fabs(CommandBase::driveTrain->GetGyroAngle()));
	if (this->driveRightSide) {
		if (currentGyroAngle >= this->desiredAngle) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			turnedDesiredAngle = true;
		}
		else {
			CommandBase::driveTrain->DriveTank(0.0, this->motorPower);
		}
	}
	else if (this->driveRightSide == false) {
		if (currentGyroAngle >= this->desiredAngle) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			turnedDesiredAngle = true;
		}
		else {
			CommandBase::driveTrain->DriveTank(this->motorPower, 0.0);
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool TurnOneSideOfRobotAngle::IsFinished() {
	return turnedDesiredAngle;
}

// Called once after isFinished returns true
void TurnOneSideOfRobotAngle::End() {
	zeroGyro = false;
	turnedDesiredAngle = false;

	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnOneSideOfRobotAngle::Interrupted() {
	End();
}
