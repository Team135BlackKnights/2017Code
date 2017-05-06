#include "AutoDriveAgitator.h"

AutoDriveAgitator::AutoDriveAgitator(double timeToDriveAgitator) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::agitator.get());
	Requires(CommandBase::collection.get());
	Requires(CommandBase::driveTrain.get());

	this->timeToDriveAgitator = timeToDriveAgitator;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoDriveAgitator::Initialize() {
	SetTimeout((this->timeToDriveAgitator + .5));
	shooterUpToSpeed = false;
	doneDrivingAgitator = false;

	timer->Reset();
	timer->Start();
	startTimer = false;

	turnOffCollection = false;

	CommandBase::shooter->stopShooterFromDriving = false;
	initializeContinueDrivingShooter = true;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveAgitator::Execute() {
	if (initializeContinueDrivingShooter == false) {
		CommandBase::shooter->stopShooterFromDriving = false;
		initializeContinueDrivingShooter = true;
	}

	if (turnOffCollection == false) {
		CommandBase::collection->SetAutoDriveCollection(false);
		turnOffCollection = true;
	}

	timerValue = timer->Get();

	shooterUpToSpeed = CommandBase::shooter->GetShooterUpToSpeed();

	if (shooterUpToSpeed == false) {
		CommandBase::agitator->DriveAgitator(0.0, 0.0);
	}
	else if (shooterUpToSpeed) {
		if (startTimer == false) {
			timer->Stop();
			timer->Reset();
			timer->Start();
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
			startTimer = true;
		}

		timerValue = timer->Get();

		if (startTimer && timerValue < this->timeToDriveAgitator) {
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER, AGITATOR_2_MOTOR_POWER);
		}
		else if (startTimer && timerValue >= this->timeToDriveAgitator) {
			CommandBase::agitator->DriveAgitator(0.0, 0.0);
			CommandBase::shooter->stopShooterFromDriving = true;
			shooterUpToSpeed = false;
			startTimer = false;
			doneDrivingAgitator = true;
		}
	}

}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveAgitator::IsFinished() {
	return (doneDrivingAgitator || IsTimedOut());
}

// Called once after isFinished returns true
void AutoDriveAgitator::End() {
	CommandBase::agitator->DriveAgitator(0.0, 0.0);
	CommandBase::collection->DriveCollection(0.0);
	initializeContinueDrivingShooter = false;
	CommandBase::shooter->stopShooterFromDriving = true;
	shooterUpToSpeed = false;
	turnOffCollection = false;
	startTimer = false;
	doneDrivingAgitator = false;
	timer->Stop();
	timer->Reset();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDriveAgitator::Interrupted() {
	End();
}
