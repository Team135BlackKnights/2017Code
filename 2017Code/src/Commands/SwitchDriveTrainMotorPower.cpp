#include "SwitchDriveTrainMotorPower.h"

SwitchDriveTrainMotorPower::SwitchDriveTrainMotorPower() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
}

// Called just before this Command runs the first time
void SwitchDriveTrainMotorPower::Initialize() {
	setDriveTrainMotorPower = false;
}

// Called repeatedly when this Command is scheduled to run
void SwitchDriveTrainMotorPower::Execute() {
	if (setDriveTrainMotorPower == false) {
		fastDriveTrainMotorPower = !fastDriveTrainMotorPower;
		CommandBase::driveTrain->SwitchBetweenFastAndSlowDriveTrainMotorPower(fastDriveTrainMotorPower);
		setDriveTrainMotorPower = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool SwitchDriveTrainMotorPower::IsFinished() {
	return setDriveTrainMotorPower;
}

// Called once after isFinished returns true
void SwitchDriveTrainMotorPower::End() {
	setDriveTrainMotorPower = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void SwitchDriveTrainMotorPower::Interrupted() {
	End();
}
