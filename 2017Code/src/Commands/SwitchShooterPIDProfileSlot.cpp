#include "SwitchShooterPIDProfileSlot.h"

SwitchShooterPIDProfileSlot::SwitchShooterPIDProfileSlot() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooter.get());
}

// Called just before this Command runs the first time
void SwitchShooterPIDProfileSlot::Initialize() {
	doneSwitchPIDProfile = false;
}

// Called repeatedly when this Command is scheduled to run
void SwitchShooterPIDProfileSlot::Execute() {
	pidProfileSlotCounter++;
	if (pidProfileSlotCounter % 2 == 1) {
		CommandBase::shooter->SelectPIDProfileSlot(Shooter::FAR_SHOT_PID_VALUES);
	}
	else if (pidProfileSlotCounter % 2 == 0) {
		CommandBase::shooter->SelectPIDProfileSlot(Shooter::CLOSE_SHOT_PID_VALUES);
	}
	doneSwitchPIDProfile = true;
}

// Make this return true when this Command no longer needs to run execute()
bool SwitchShooterPIDProfileSlot::IsFinished() {
	return doneSwitchPIDProfile;
}

// Called once after isFinished returns true
void SwitchShooterPIDProfileSlot::End() {
	doneSwitchPIDProfile = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwitchShooterPIDProfileSlot::Interrupted() {
	End();
}
