#include "DriveWithJoysticks.h"

DriveWithJoysticks::DriveWithJoysticks() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get())
;}

// Called just before this Command runs the first time
void DriveWithJoysticks::Initialize() {
	CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::LEFT_SIDE_ENCODER);
	CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::RIGHT_SIDE_ENCODER);
	CommandBase::driveTrain->ZeroNavXAngle();
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoysticks::Execute() {
	leftJoystickValue = oi->GetYAxis(OI::LEFT_DRIVE_JOYSTICK);
	rightJoystickValue = oi->GetYAxis(OI::RIGHT_DRIVE_JOYSTICK);
	CommandBase::driveTrain->DriveTank(leftJoystickValue, rightJoystickValue);

	if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::TOP_POV)) {
		CommandBase::driveTrain->DriveTank(POV_DRIVE_TRAIN_MOTOR_POWER, POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::RIGHT_POV)) {
		CommandBase::driveTrain->DriveTank(POV_DRIVE_TRAIN_MOTOR_POWER, -POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::BOTTOM_POV)) {
		CommandBase::driveTrain->DriveTank(-POV_DRIVE_TRAIN_MOTOR_POWER, -POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::LEFT_POV)) {
		CommandBase::driveTrain->DriveTank(-POV_DRIVE_TRAIN_MOTOR_POWER, POV_DRIVE_TRAIN_MOTOR_POWER);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoysticks::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveWithJoysticks::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoysticks::Interrupted() {
	End();
}
