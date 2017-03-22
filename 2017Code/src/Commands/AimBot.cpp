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
	if(angleToTurn == 0) isbad = true;
	else isbad = false;
	if(cameraNumber == 1)
	{
		double sonar_value = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches();
		double dangleToTurn = 90 - atan( (sonar_value - SPRING_IN) / (CAMERA_TO_GEAR_IN - sonar_value / tan((90 -angleToTurn) * M_PI / 180))) * 180 / M_PI;
		std::cout << "\n\n\n\n\nangle: " << dangleToTurn << "\nSonar: " << sonar_value << "\n\n\n\n";
		CommandBase::driveTrain->TurnPIDEnable(angleToTurn - 18);
	}
	else
		CommandBase::driveTrain->TurnPIDEnable(angleToTurn - 4);//- 18);

	time.Start();
	time.Reset();
	CommandBase::driveTrain->ZeroGyroAngle();
	CommandBase::driveTrain->is_aiming = true;
	std::cout <<"initialized \n\n";
}

// Called repeatedly when this Command is scheduled to run
void AimBot::Execute() {
	frc::SmartDashboard::PutNumber("Angle to Turn: ", CommandBase::driveTrain->GetGyroAngle());
	CommandBase::driveTrain->PIDTurning();
	std::cout << "testing\n";
}

// Make this return true when this Command no longer needs to run execute()
bool AimBot::IsFinished() {
	return time.Get() > 1 || !CommandBase::driveTrain->turnController->IsEnabled() || (CommandBase::oi->GetAction(CommandBase::oi->LEFT_DRIVE_JOYSTICK, 10) && CommandBase::oi->GetAction(CommandBase::oi->LEFT_DRIVE_JOYSTICK, 8)) ;
}

// Called once after isFinished returns true
void AimBot::End() {
	time.Stop();
	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimBot::Interrupted() {
	time.Stop();
	CommandBase::driveTrain->is_aiming = false;
}
