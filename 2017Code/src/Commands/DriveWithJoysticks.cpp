#include "DriveWithJoysticks.h"
#include <iostream>

DriveWithJoysticks::DriveWithJoysticks() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
}

// Called just before this Command runs the first time
void DriveWithJoysticks::Initialize() {
	CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::LEFT_SIDE_ENCODER);
	CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::RIGHT_SIDE_ENCODER);
	CommandBase::driveTrain->ZeroGyroAngle();

	stopLeftMotorCounter = 1;
	resetLeftStopCounter = true;
	stopRightMotorCounter = 1;
	resetRightStopCounter = true;

	leftBrakeDrive = false;
	rightBrakeDrive = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoysticks::Execute() {
	if(CommandBase::driveTrain->is_aiming) return;

	if (resetLeftStopCounter == false) {
		stopLeftMotorCounter = 1;
		resetLeftStopCounter = true;
	}

	if (resetRightStopCounter == false) {
		stopRightMotorCounter = 1;
		resetRightStopCounter = true;
	}

	leftJoystickValue = oi->GetYAxis(OI::LEFT_DRIVE_JOYSTICK);
	rightJoystickValue = oi->GetYAxis(OI::RIGHT_DRIVE_JOYSTICK);


	if (leftJoystickValue == 0.0) {
		leftBrakeDrive = true;
		if ((stopLeftMotorCounter % BRAKING_COUNTER_DIVIDER) != 0) {
			actualUsedLeftJoystickValue = ZERO_DRIVE_TRAIN_MOTOR_POWER;
		}
		else if ((stopLeftMotorCounter % BRAKING_COUNTER_DIVIDER) == 0) {
			actualUsedLeftJoystickValue = STOP_DRIVE_TRAIN_MOTOR_POWER;
		}
		stopLeftMotorCounter++;
	}
	else {
		stopLeftMotorCounter = 1;
		leftBrakeDrive = false;
	}

	if (rightJoystickValue == 0.0) {
		rightBrakeDrive = true;
		if ((stopRightMotorCounter % BRAKING_COUNTER_DIVIDER) != 0) {
			actualUsedRightJoystickValue = ZERO_DRIVE_TRAIN_MOTOR_POWER;
		}
		else if ((stopRightMotorCounter % BRAKING_COUNTER_DIVIDER) == 0) {
			actualUsedRightJoystickValue = STOP_DRIVE_TRAIN_MOTOR_POWER;
		}
		stopRightMotorCounter++;
	}
	else {
		stopRightMotorCounter = 1;
		rightBrakeDrive = false;
	}

	if (leftBrakeDrive && rightBrakeDrive) {
		CommandBase::driveTrain->DriveTank(actualUsedLeftJoystickValue, actualUsedRightJoystickValue);
	}
	else if (leftBrakeDrive && rightBrakeDrive == false) {
		CommandBase::driveTrain->DriveTank(actualUsedLeftJoystickValue, rightJoystickValue);
	}
	else if (leftBrakeDrive == false && rightBrakeDrive) {
		CommandBase::driveTrain->DriveTank(leftJoystickValue, actualUsedRightJoystickValue);
	}
	else if (leftBrakeDrive == false && rightBrakeDrive == false) {
		CommandBase::driveTrain->DriveTank(leftJoystickValue, rightJoystickValue);
	}

	throttleValue = CommandBase::oi->GetThrottleValue(OI::RIGHT_DRIVE_JOYSTICK);
	povMotorPower = (.5 * (throttleValue + 1.0));
	gyroAngle = CommandBase::driveTrain->GetGyroAngle();

	if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::TOP_POV)) {
		CommandBase::driveTrain->DriveStraightWithGyro(povMotorPower, gyroAngle);
		//CommandBase::driveTrain->DriveTank(.22, .22);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::RIGHT_POV)) {
		CommandBase::driveTrain->DriveTank(POV_DRIVE_TRAIN_MOTOR_POWER, -POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::BOTTOM_POV)) {
		CommandBase::driveTrain->DriveStraightWithGyro(-povMotorPower, gyroAngle);
		//CommandBase::driveTrain->DriveTank(-.22, -.22);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::RIGHT_DRIVE_JOYSTICK, OI::LEFT_POV)) {
		CommandBase::driveTrain->DriveTank(-POV_DRIVE_TRAIN_MOTOR_POWER, POV_DRIVE_TRAIN_MOTOR_POWER);
	}
	else {
		CommandBase::driveTrain->ZeroGyroAngle();
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveWithJoysticks::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveWithJoysticks::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	resetLeftStopCounter = false;
	resetRightStopCounter = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveWithJoysticks::Interrupted() {
	End();
}
