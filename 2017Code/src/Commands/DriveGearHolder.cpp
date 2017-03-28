#include "DriveGearHolder.h"

DriveGearHolder::DriveGearHolder() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::gearHolder.get());
}

// Called just before this Command runs the first time
void DriveGearHolder::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveGearHolder::Execute() {
	gearHolderUpwardsButtonPressed = CommandBase::oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::GEAR_HOLDER_UPWARDS_BUTTON);
	gearHolderDownwardsButtonPressed = CommandBase::oi->GetButtonPressed(OI::MANIPULATOR_JOYSTICK, OI::GEAR_HOLDER_DOWNWARDS_BUTTON);

	if (gearHolderUpwardsButtonPressed) {
		CommandBase::gearHolder->DriveGearHolderMotor(GEAR_HOLDER_MOTOR_POWER);
	}
	else if (gearHolderDownwardsButtonPressed) {
		CommandBase::gearHolder->DriveGearHolderMotor(-GEAR_HOLDER_MOTOR_POWER);
	}
	else {
		CommandBase::gearHolder->DriveGearHolderMotor(0.0);
	}

	if (CommandBase::oi->POVDirectionPressed(OI::MANIPULATOR_JOYSTICK, OI::TOP_POV)) {
		CommandBase::gearHolder->SetGearHolderServoValue(GearHolder::SERVO_IN_POSITION);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::MANIPULATOR_JOYSTICK, OI::BOTTOM_POV)) {
		CommandBase::gearHolder->SetGearHolderServoValue(GearHolder::SERVO_OUT_POSITION);
	}

	photoElectricSensorValue = CommandBase::gearHolder->GetPhotoElectricSensorValue();
	frc::SmartDashboard::PutBoolean("Gear In Gear Holder", photoElectricSensorValue);

	upperLimitSwitchPressed = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::UPPER_LIMIT_SWITCH_PORT);
	lowerLimitSwitchPressed = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
	frc::SmartDashboard::PutBoolean("Gear Upper Limit Switch Pressed", upperLimitSwitchPressed);
	frc::SmartDashboard::PutBoolean("Gear Lower Limit Switch Pressed", lowerLimitSwitchPressed);
}

// Make this return true when this Command no longer needs to run execute()
bool DriveGearHolder::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveGearHolder::End() {
	CommandBase::gearHolder->DriveGearHolderMotor(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveGearHolder::Interrupted() {
	End();
}
