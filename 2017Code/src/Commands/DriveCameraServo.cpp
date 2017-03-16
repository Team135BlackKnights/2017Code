#include "DriveCameraServo.h"

DriveCameraServo::DriveCameraServo() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::cameraServo.get());

}

// Called just before this Command runs the first time
void DriveCameraServo::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveCameraServo::Execute() {
	if (CommandBase::oi->POVDirectionPressed(OI::LEFT_DRIVE_JOYSTICK, OI::TOP_POV)) {
		CommandBase::cameraServo->SetCameraServoValue(CameraServo::SERVO_IN_POSITION);
	}
	else if (CommandBase::oi->POVDirectionPressed(OI::LEFT_DRIVE_JOYSTICK, OI::BOTTOM_POV)) {
		CommandBase::cameraServo->SetCameraServoValue(CameraServo::SERVO_OUT_POSITION);
	}

}

// Make this return true when this Command no longer needs to run execute()
bool DriveCameraServo::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveCameraServo::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveCameraServo::Interrupted() {

}
