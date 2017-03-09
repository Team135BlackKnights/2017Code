#include "DriveWithJoysticks.h"
#include <iostream>

DriveWithJoysticks::DriveWithJoysticks() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get())
;}

// Called just before this Command runs the first time
void DriveWithJoysticks::Initialize() {
	CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::LEFT_SIDE_ENCODER);
	CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::RIGHT_SIDE_ENCODER);
	CommandBase::driveTrain->ZeroGyroAngle();
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoysticks::Execute() {
	if(CommandBase::driveTrain->is_aiming) return;
	leftJoystickValue = oi->GetYAxis(OI::LEFT_DRIVE_JOYSTICK);
	rightJoystickValue = oi->GetYAxis(OI::RIGHT_DRIVE_JOYSTICK);
	CommandBase::driveTrain->DriveTank(leftJoystickValue, rightJoystickValue);

	throttleValue = CommandBase::oi->GetThrottleValue(OI::RIGHT_DRIVE_JOYSTICK);
	povMotorPower = (.5 * (throttleValue + 1.0));
	gyroAngle = CommandBase::driveTrain->GetGyroAngle();

	if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::TOP_POV)) {
		CommandBase::driveTrain->DriveStraightWithGyro(povMotorPower, gyroAngle);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::RIGHT_POV)) {
		CommandBase::driveTrain->DriveTank(POV_DRIVE_TRAIN_MOTOR_POWER, -POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::BOTTOM_POV)) {
		CommandBase::driveTrain->DriveStraightWithGyro(-povMotorPower, gyroAngle);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::LEFT_POV)) {
		CommandBase::driveTrain->DriveTank(-POV_DRIVE_TRAIN_MOTOR_POWER, POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else {
		CommandBase::driveTrain->ZeroGyroAngle();
	}

	//frc::SmartDashboard::PutNumber("Angle: ", gyroAngle);

	/*leftDriveTrainEncoderRPM = CommandBase::driveTrain->GetEncoderRPM(DriveTrain::LEFT_SIDE_ENCODER);
	rightDriveTrainEncoderRPM = CommandBase::driveTrain->GetEncoderRPM(DriveTrain::RIGHT_SIDE_ENCODER);

	frc::SmartDashboard::PutNumber("Left Drive Train Encoder RPM", leftDriveTrainEncoderRPM);
	frc::SmartDashboard::PutNumber("Right Drive Train Encoder RPM", rightDriveTrainEncoderRPM); */
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
