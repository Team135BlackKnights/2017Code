#include "AutoMoveGearHolder.h"

AutoMoveGearHolder::AutoMoveGearHolder(bool moveGearHolderUpwards) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::gearHolder.get());

	this->moveGearHolderUpwards = moveGearHolderUpwards;
}

// Called just before this Command runs the first time
void AutoMoveGearHolder::Initialize() {
	SetTimeout(AUTO_MOVE_GEAR_HOLDER_TIMEOUT);
	movedGearHolderToDesiredLimitSwitch = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoMoveGearHolder::Execute() {
	if (this->moveGearHolderUpwards) {
		maxLimitSwitchPressed = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::UPPER_LIMIT_SWITCH_PORT);
		if (maxLimitSwitchPressed) {
			CommandBase::gearHolder->DriveGearHolderMotor(0.0);
			movedGearHolderToDesiredLimitSwitch = true;
		}
		else {
			CommandBase::gearHolder->DriveGearHolderMotor(GEAR_HOLDER_MOTOR_POWER);
			movedGearHolderToDesiredLimitSwitch = false;
		}
	}
	else if (this->moveGearHolderUpwards == false) {
		minLimitSwitchPressed = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
		if (minLimitSwitchPressed) {
			CommandBase::gearHolder->DriveGearHolderMotor(0.0);
			movedGearHolderToDesiredLimitSwitch = true;
		}
		else {
			CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
			movedGearHolderToDesiredLimitSwitch = false;
		}
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoMoveGearHolder::IsFinished() {
	return (movedGearHolderToDesiredLimitSwitch || IsTimedOut());
}

// Called once after isFinished returns true
void AutoMoveGearHolder::End() {
	movedGearHolderToDesiredLimitSwitch = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoMoveGearHolder::Interrupted() {
	End();
}
