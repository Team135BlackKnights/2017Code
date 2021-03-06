#include "AimBot.h"


AimBot::AimBot(int camNumber) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
	this->cameraNumber = camNumber;
}

// Called just before this Command runs the first time
void AimBot::Initialize() {
	std::cout << "running \n\n\n:";
	double angleToTurn = -CommandBase::server->get_angle(cameraNumber);
	/*if(angleToTurn == 0) isbad = true;
	else isbad = false;*/
	if(cameraNumber == 1)
	{
		std::cout << "\n\n\n\n\n\n\n\n\n\nCAMERA NUMBER IS ONE \n\n\n\n\n\n\n\n";
		if (CommandBase::ultrasonicSensor->usingRightUltrasonicSensorForGearCamera) {
			sonar_value = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();
		}
		else {
			//sonar_value = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches(UltrasonicSensor::LEFT_ULTRASONIC_SENSOR);
		}
		double dangleToTurn = 90 - atan( (sonar_value - SPRING_IN) / (CAMERA_TO_GEAR_IN - sonar_value * tan((angleToTurn) * M_PI / 180))) * 180 / M_PI;
		std::cout << "\n\n\n\n\nangle: " << dangleToTurn << "\nSonar: " << sonar_value << "\n\n\n\n" << "\ncam angle " << angleToTurn;
		frc::SmartDashboard::PutNumber("Trig Angle: ", dangleToTurn);
		CommandBase::driveTrain->TurnPIDEnable(-dangleToTurn);
	}
	else
	{
		std::cout << "\n\n\n\nCAMERA !\n\n\n\n";
		CommandBase::driveTrain->TurnPIDEnable(2.5);//- 18);
	}
	frc::SmartDashboard::PutNumber("Angle to Turn Starting: ", angleToTurn);
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
	std::cout << "Time: " << time.Get() << std::endl;
	//std::cout << "testing\n";
}

// Make this return true when this Command no longer needs to run execute()
bool AimBot::IsFinished() {
	//return false;
	return time.Get() > 3  || isbad == true;
}

// Called once after isFinished returns true
void AimBot::End() {
	time.Stop();
	CommandBase::driveTrain->DriveTank(0,0);
	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AimBot::Interrupted() {
	time.Stop();
	CommandBase::driveTrain->is_aiming = false;
}
