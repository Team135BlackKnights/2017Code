#include "AutoDriveAgitator.h"

AutoDriveAgitator::AutoDriveAgitator(bool stopCollection) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::agitator.get());
	Requires(CommandBase::collection.get());

	this->stopCollection = stopCollection;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoDriveAgitator::Initialize() {
	SetTimeout(AUTO_DRIVE_AGITATOR_TIMEOUT);
	shooterUpToSpeed = false;
	startTimer = false;
	doneDrivingAgitator = false;
	timer->Reset();
	timer->Start();

	turnOffCollection = false;

	startDrivingCollection = false;
	initializeTimerForCollection = false;
	doneDrivingCollection = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveAgitator::Execute() {
	if (this->stopCollection) {
		if (turnOffCollection == false) {
			CommandBase::collection->SetAutoDriveCollection(false);
			turnOffCollection = true;
		}
	}

	timerValue = timer->Get();

	shooterUpToSpeed = CommandBase::shooter->GetShooterUpToSpeed();

	if (shooterUpToSpeed == false) {
		CommandBase::agitator->DriveAgitator(0.0);
	}
	else if (shooterUpToSpeed) {
		if (startTimer == false) {
			timer->Stop();
			timer->Reset();
			timer->Start();
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
			startTimer = true;
		}

		if (startTimer && timerValue < TIME_TO_WAIT_FOR_FUEL_TO_SHOOT) {
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
		}
		else if (startTimer && timerValue >= TIME_TO_WAIT_FOR_FUEL_TO_SHOOT) {
			CommandBase::agitator->DriveAgitator(0.0);
			shooterUpToSpeed = false;
			startTimer = false;
			doneDrivingAgitator = true;
		}

		if (timerValue >= TIME_TO_WAIT_BEFORE_RUNNING_COLLECTION && startDrivingCollection == false && doneDrivingCollection == false) {
			startDrivingCollection = true;
		}

		if (startDrivingCollection && doneDrivingCollection == false) {
			timerValue = timer->Get();
			if (initializeTimerForCollection == false) {
				initialTimeForRunningCollection = timerValue;
				initializeTimerForCollection = true;
			}

			differenceBetweenCurrentAndInitialTimeForRunningCollection = (timerValue - initialTimeForRunningCollection);

			if (differenceBetweenCurrentAndInitialTimeForRunningCollection < TIME_TO_RUN_COLLECTION) {
				CommandBase::collection->DriveCollection(COLLECTION_MOTOR_POWER);
			}
			else if (differenceBetweenCurrentAndInitialTimeForRunningCollection >= TIME_TO_RUN_COLLECTION) {
				CommandBase::collection->DriveCollection(0.0);
				doneDrivingCollection = true;
			}
		}
	}

}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveAgitator::IsFinished() {
	return (doneDrivingAgitator || IsTimedOut());
}

// Called once after isFinished returns true
void AutoDriveAgitator::End() {
	CommandBase::agitator->DriveAgitator(0.0);
	CommandBase::collection->DriveCollection(0.0);
	shooterUpToSpeed = false;
	turnOffCollection = false;
	startTimer = false;
	doneDrivingAgitator = false;
	timer->Stop();
	timer->Reset();

	startDrivingCollection = false;
	initializeTimerForCollection = false;
	doneDrivingCollection = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDriveAgitator::Interrupted() {
	End();
}
