#include "DriveDistance.h"

DriveDistance::DriveDistance(double desiredDistanceToTravel, double motorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->motorPower = motorPower;
}

// Called just before this Command runs the first time
void DriveDistance::Initialize() {
	initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	measuredInitialDistanceTraveled = true;
	distanceTraveled = false;
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;
	CommandBase::driveTrain->is_aiming = true;
	requiresDriveTrain = true;
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
	if (requiresDriveTrain == false) {
		CommandBase::driveTrain->is_aiming = true;
		requiresDriveTrain = true;
	}

	if (measuredInitialDistanceTraveled == false) {
		initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		measuredInitialDistanceTraveled = true;
	}

	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	gyroAngle = CommandBase::driveTrain->GetGyroAngle();

	measuredCurrentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	actualCurrentDistanceTraveled = (fabs(measuredCurrentDistanceTraveled - initialDistanceTraveled));
	if (actualCurrentDistanceTraveled >= this->desiredDistanceToTravel) {
		CommandBase::driveTrain->DriveStraightWithGyro(0.0, 0.0);
		distanceTraveled = true;
	}
	else {
		CommandBase::driveTrain->DriveStraightWithGyro(this->motorPower, gyroAngle);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished() {
	return distanceTraveled;
}

// Called once after isFinished returns true
void DriveDistance::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	distanceTraveled = false;
	measuredInitialDistanceTraveled = false;
	zeroGyro = false;
	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDistance::Interrupted() {
	End();
}
