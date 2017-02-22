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

	leftEncoderValue = CommandBase::driveTrain->GetEncoderPosition(DriveTrain::LEFT_SIDE_ENCODER);
	rightEncoderValue = CommandBase::driveTrain->GetEncoderPosition(DriveTrain::RIGHT_SIDE_ENCODER);

	leftEncoderDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::LEFT_SIDE_ENCODER);
	rightEncoderDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);

	std::cout << "Left Encoder Value: " << leftEncoderValue << std::endl;
	std::cout << "Right Encoder Value: " << rightEncoderValue << std::endl;
	std::cout << "Left Encoder Distance Traveled: " << leftEncoderDistanceTraveled << std::endl;
	std::cout << "Right Encoder Distance Traveled: " << rightEncoderDistanceTraveled << std::endl;

	navXAngle = CommandBase::driveTrain->GetNavXAngle();
	std::cout << "NavX Angle: " << navXAngle << std::endl;
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
