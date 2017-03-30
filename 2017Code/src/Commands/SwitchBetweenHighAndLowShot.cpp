#include "SwitchBetweenHighAndLowShot.h"

SwitchBetweenHighAndLowShot::SwitchBetweenHighAndLowShot() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());
}

// Called just before this Command runs the first time
void SwitchBetweenHighAndLowShot::Initialize() {
	switchedFarAndCloseShotRPM = false;
}

// Called repeatedly when this Command is scheduled to run
void SwitchBetweenHighAndLowShot::Execute() {
	if (switchedFarAndCloseShotRPM == false) {
		closeShot = !closeShot;
		CommandBase::shooter->SetSwitchFarAndCloseShotShooterRPM(closeShot);
		switchedFarAndCloseShotRPM = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool SwitchBetweenHighAndLowShot::IsFinished() {
	return switchedFarAndCloseShotRPM;
}

// Called once after isFinished returns true
void SwitchBetweenHighAndLowShot::End() {
	switchedFarAndCloseShotRPM = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwitchBetweenHighAndLowShot::Interrupted() {
	End();
}
