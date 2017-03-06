#include "AutoDriveShooter.h"

AutoDriveShooter::AutoDriveShooter(int setpointRPM) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());
	Requires(CommandBase::agitator.get());
	Requires(CommandBase::pdp.get());
	this->setpointRPM = setpointRPM;
	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoDriveShooter::Initialize() {
	CommandBase::shooter->ConfigureShooterPID();
	beginTimerToLevelOutSetpoint = false;
	moveAgitatorToShootFuel = false;
	timer->Reset();
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveShooter::Execute() {
	timerValue = timer->Get();

	currentShooterWheelRPM = CommandBase::shooter->GetShooterWheelRPM();
	CommandBase::shooter->DriveShooterMotor(this->setpointRPM);

	agitatorCurrent = CommandBase::pdp->GetCurrentOfPDPPort(PDP::AGITATOR_TALON_PDP_PORT);

	if (currentShooterWheelRPM >= this->setpointRPM && beginTimerToLevelOutSetpoint == false) {
		beginTimerToLevelOutSetpoint = true;
		timer->Reset();
		timer->Start();
	}
	else if (beginTimerToLevelOutSetpoint && timerValue >= WAIT_TIME_FOR_SHOOTER && moveAgitatorToShootFuel == false) {
		moveAgitatorToShootFuel = true;
		timer->Stop();
		timer->Reset();
	}
	else if (moveAgitatorToShootFuel) {
		if (agitatorCurrent >= THRESHOLD_AGITATOR_CURRENT_DRAW || startDrivingAgitatorBackwards) {
			if (startDrivingAgitatorBackwards == false) {
				timer->Reset();
				timer->Start();
				CommandBase::agitator->DriveAgitator(-AGITATOR_MOTOR_POWER);
				startDrivingAgitatorBackwards = true;
			}
			else if (startDrivingAgitatorBackwards) {
				if (timerValue >= TIME_TO_MOVE_AGITATOR_BACKWARDS) {
					CommandBase::agitator->DriveAgitator(0.0);
					startDrivingAgitatorBackwards = false;
				}
				else {
					CommandBase::agitator->DriveAgitator(-AGITATOR_MOTOR_POWER);
				}
			}
		}
		else if (agitatorCurrent < THRESHOLD_AGITATOR_CURRENT_DRAW) {
			CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveShooter::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutoDriveShooter::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDriveShooter::Interrupted() {

}
