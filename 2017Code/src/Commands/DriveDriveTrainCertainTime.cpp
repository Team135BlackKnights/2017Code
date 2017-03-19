#include "DriveDriveTrainCertainTime.h"

DriveDriveTrainCertainTime::DriveDriveTrainCertainTime(double desiredTimerValue, double motorPower, bool rammingIntoHopper) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());

	this->desiredTimerValue = desiredTimerValue;
	this->motorPower = motorPower;
	this->rammingIntoHopper = rammingIntoHopper;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void DriveDriveTrainCertainTime::Initialize() {
	timer->Reset();
	timer->Start();
	restartTimer = true;
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;
	doneDrivingWithTimer = false;
	initializeTimerForWaitingBeforeAdjustingAngle = false;
	waitBeforeAdjustingAngle = false;
	initializeTurnToAdjustAngle = false;
	adjustedAngle = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveDriveTrainCertainTime::Execute() {
	if (restartTimer == false) {
		timer->Reset();
		timer->Start();
		restartTimer = true;
	}

	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	currentGyroAngle = CommandBase::driveTrain->GetGyroAngle();

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
	else if (doneDrivingWithTimer && this->rammingIntoHopper) {
		if (waitBeforeAdjustingAngle == false) {
			if (initializeTimerForWaitingBeforeAdjustingAngle == false) {
				timer->Reset();
				timer->Start();
				initializeTimerForWaitingBeforeAdjustingAngle = true;
			}

			currentTime = timer->Get();

			if (currentTime >= WAIT_TIME_BEFORE_ADJUSTING_ANGLE) {
				timer->Stop();
				timer->Reset();
				waitBeforeAdjustingAngle = true;
			}

			CommandBase::driveTrain->DriveTank(0.0, 0.0);
		}
		else if (waitBeforeAdjustingAngle) {
			if (initializeTurnToAdjustAngle == false) {
				if (currentGyroAngle < 0.0) {
					turnRightToAdjustAngle = true;
				}
				else if (currentGyroAngle > 0.0) {
					turnRightToAdjustAngle = false;
				}
				else {
					adjustedAngle = true;
				}
				initializeTurnToAdjustAngle = true;
			}

			if (turnRightToAdjustAngle) {
				if (currentGyroAngle >= 0.0) {
					CommandBase::driveTrain->DriveTank(0.0, 0.0);
					adjustedAngle = true;
				}
				else {
					CommandBase::driveTrain->DriveTank(0.0, -ADJUST_ANGLE_MOTOR_POWER);
					adjustedAngle = false;
				}
			}
			else if (turnRightToAdjustAngle == false) {
				if (currentGyroAngle <= 0.0) {
					CommandBase::driveTrain->DriveTank(0.0, 0.0);
					adjustedAngle = true;
				}
				else {
					CommandBase::driveTrain->DriveTank(-ADJUST_ANGLE_MOTOR_POWER, 0.0);
					adjustedAngle = false;
				}
			}
		}
	}
	else if (doneDrivingWithTimer && this->rammingIntoHopper == false) {
		adjustedAngle = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveDriveTrainCertainTime::IsFinished() {
	return adjustedAngle;
}

// Called once after isFinished returns true
void DriveDriveTrainCertainTime::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	restartTimer = false;
	zeroGyro = false;
	doneDrivingWithTimer = false;
	initializeTimerForWaitingBeforeAdjustingAngle = false;
	waitBeforeAdjustingAngle = false;
	initializeTurnToAdjustAngle = false;
	adjustedAngle = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDriveTrainCertainTime::Interrupted() {
	End();
}
