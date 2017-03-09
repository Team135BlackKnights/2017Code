#include "AutoDriveAgitator.h"

AutoDriveAgitator::AutoDriveAgitator() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::agitator.get());

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoDriveAgitator::Initialize() {
	shooterUpToSpeed = false;
	startTimer = false;
	doneDrivingAgitator = false;
	timer->Reset();
	timer->Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveAgitator::Execute() {
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
		else if (startTimer && timerValue < TIME_TO_WAIT_FOR_FUEL_TO_SHOOT) {
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
		}
		else if (startTimer && timerValue >= TIME_TO_WAIT_FOR_FUEL_TO_SHOOT) {
			CommandBase::agitator->DriveAgitator(0.0);
			shooterUpToSpeed = false;
			startTimer = false;
			doneDrivingAgitator = true;
		}
	}

}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveAgitator::IsFinished() {
	return doneDrivingAgitator;
}

// Called once after isFinished returns true
void AutoDriveAgitator::End() {
	CommandBase::agitator->DriveAgitator(0.0);
	shooterUpToSpeed = false;
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
