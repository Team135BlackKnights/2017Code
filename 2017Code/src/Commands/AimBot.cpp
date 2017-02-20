#include "AimBot.h"


AimBot::AimBot(int camNumber) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(driveTrain.get());
	this->cameraNumber = camNumber;
}

// Called just before this Command runs the first time
void AimBot::Initialize() {
	driveTrain->TurnPIDEnable(server->get_angle(cameraNumber));
}

// Called repeatedly when this Command is scheduled to run
void AimBot::Execute() {
	frc::SmartDashboard::PutNumber("Angle: ", driveTrain->GetNavXAngle());
}

// Make this return true when this Command no longer needs to run execute()
bool AimBot::IsFinished() {
	return !driveTrain->turnController->IsEnabled() || oi->GetAction(oi->LEFT_DRIVE_JOYSTICK,10);
}

// Called once after isFinished returns true
void AimBot::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimBot::Interrupted() {

}
