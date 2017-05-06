#include "DriveDriveTrainCertainTime.h"

DriveDriveTrainCertainTime::DriveDriveTrainCertainTime(double desiredTimerValue, double motorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());

	this->desiredTimerValue = desiredTimerValue;
	this->motorPower = motorPower;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void DriveDriveTrainCertainTime::Initialize() {
	timer->Reset();
	timer->Start();
	restartTimer = true;
	doneDrivingWithTimer = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveDriveTrainCertainTime::Execute() {
	if (restartTimer == false) {
		timer->Reset();
		timer->Start();
		restartTimer = true;
	}

	currentTime = timer->Get();

	if (doneDrivingWithTimer == false) {
		if (currentTime >= this->desiredTimerValue) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			timer->Stop();
			timer->Reset();
			doneDrivingWithTimer = true;
		}
		else {
			CommandBase::driveTrain->DriveTank(this->motorPower, this->motorPower);
			doneDrivingWithTimer = false;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveDriveTrainCertainTime::IsFinished() {
	return doneDrivingWithTimer;
}

// Called once after isFinished returns true
void DriveDriveTrainCertainTime::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	timer->Stop();
	timer->Reset();
	restartTimer = false;
	doneDrivingWithTimer = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDriveTrainCertainTime::Interrupted() {
	End();
}
