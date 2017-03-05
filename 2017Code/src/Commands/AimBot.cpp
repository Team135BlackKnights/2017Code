#include "AimBot.h"


AimBot::AimBot(int camNumber) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	this->cameraNumber = camNumber;
}

// Called just before this Command runs the first time
void AimBot::Initialize() {
	std::cout << "running \n\n\n:";
	double angleToTurn = -CommandBase::server->get_angle(cameraNumber);
	std::cout << "angle: " << angleToTurn;
	CommandBase::driveTrain->TurnPIDEnable(angleToTurn - 10);
	time.Start();
	time.Reset();
	CommandBase::driveTrain->ZeroGyroAngle();
	CommandBase::driveTrain->is_aiming = true;
	std::cout <<"initialized \n\n";
}

// Called repeatedly when this Command is scheduled to run
void AimBot::Execute() {
	frc::SmartDashboard::PutNumber("Angle: ", CommandBase::driveTrain->GetGyroAngle());
	CommandBase::driveTrain->PIDTurning();
}

// Make this return true when this Command no longer needs to run execute()
bool AimBot::IsFinished() {
	return time.Get() > 1 || !CommandBase::driveTrain->turnController->IsEnabled() || CommandBase::oi->GetAction(CommandBase::oi->LEFT_DRIVE_JOYSTICK, 10);
}

// Called once after isFinished returns true
void AimBot::End() {
	time.Stop();
	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimBot::Interrupted() {

}
