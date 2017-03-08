#include "DriveUntilLidarIsCertainValue.h"

DriveUntilLidarIsCertainValue::DriveUntilLidarIsCertainValue(double desiredLidarValueToDriveUntil, double motorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidars.get());

	this->desiredLidarValueToDriveUntil = desiredLidarValueToDriveUntil;
	this->motorPower = motorPower;
}

// Called just before this Command runs the first time
void DriveUntilLidarIsCertainValue::Initialize() {
	SetTimeout(3.5);
	receivedFirstLidarValue = false;
	drivenUntilDesiredLidarValue = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilLidarIsCertainValue::Execute() {
	CommandBase::driveTrain->DriveTank(this->motorPower, this->motorPower);

	currentLidarValueInches = CommandBase::lidars->GetLidarValueWholeProcess(Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
	if (receivedFirstLidarValue == false) {
		if (currentLidarValueInches > 0.0) {
			receivedFirstLidarValue = true;
		}
	}

	if (receivedFirstLidarValue) {
		if (currentLidarValueInches <= this->desiredLidarValueToDriveUntil) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			drivenUntilDesiredLidarValue = true;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveUntilLidarIsCertainValue::IsFinished() {
	return (drivenUntilDesiredLidarValue || IsTimedOut());
}

// Called once after isFinished returns true
void DriveUntilLidarIsCertainValue::End() {
	receivedFirstLidarValue = false;
	drivenUntilDesiredLidarValue = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveUntilLidarIsCertainValue::Interrupted() {
	End();
}
