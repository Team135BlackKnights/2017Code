#include "GearUpAndDown.h"

GearUpAndDown::GearUpAndDown() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::gearHolder.get());
}

// Called just before this Command runs the first time
void GearUpAndDown::Initialize() {
	gearDirectionCounter = 1;
	CommandBase::gearHolder->gearUsedNotDefaultly = true;
}

// Called repeatedly when this Command is scheduled to run
void GearUpAndDown::Execute() {
	minLimitSwitchPressed = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
	maxLimitSwitchPressed = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::UPPER_LIMIT_SWITCH_PORT);

	if ((gearDirectionCounter % 2) == 1) {
		if (minLimitSwitchPressed) {
			gearDirectionCounter++;
		}
		else {
			CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
			std::cout << "Gear Holder Driving Down" << std::endl;
		}
	}
	else if ((gearDirectionCounter % 2) == 0) {
		if (maxLimitSwitchPressed) {
			gearDirectionCounter++;
		}
		else {
			CommandBase::gearHolder->DriveGearHolderMotor(GEAR_HOLDER_MOTOR_POWER);
			std::cout << "Gear Holder Driving Up" << std::endl;
		}
	}

	std::cout << "Gear Direction Counter: " << gearDirectionCounter << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool GearUpAndDown::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void GearUpAndDown::End() {
	gearDirectionCounter = 1;
	CommandBase::gearHolder->gearUsedNotDefaultly = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GearUpAndDown::Interrupted() {
	End();
}
