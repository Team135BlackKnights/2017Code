#include "AutoDriveShooter.h"

AutoDriveShooter::AutoDriveShooter(int setpointRPM) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
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
	shooterWheelRPM = CommandBase::shooter->GetShooterWheelRPM();
	CommandBase::shooter->DriveShooterMotor(this->setpointRPM);
	if (shooterWheelRPM >= this->setpointRPM && beginTimerToLevelOutSetpoint == false) {
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
		CommandBase::agitator->DriveAgitator(AGITATOR_MOTOR_POWER);
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
