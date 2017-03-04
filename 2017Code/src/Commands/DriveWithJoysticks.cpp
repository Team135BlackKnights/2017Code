#include "DriveWithJoysticks.h"
#include <iostream>

DriveWithJoysticks::DriveWithJoysticks() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get())
;}

// Called just before this Command runs the first time
void DriveWithJoysticks::Initialize() {
	std::cout << "running\n";
}

// Called repeatedly when this Command is scheduled to run
void DriveWithJoysticks::Execute() {
	if(CommandBase::driveTrain->is_aiming) return;
	leftJoystickValue = oi->GetYAxis(OI::LEFT_DRIVE_JOYSTICK);
	rightJoystickValue = oi->GetYAxis(OI::RIGHT_DRIVE_JOYSTICK);
	CommandBase::driveTrain->DriveTank(leftJoystickValue, rightJoystickValue);
	frc::SmartDashboard::PutNumber("Angle: ", CommandBase::driveTrain->GetGyroAngle());
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
