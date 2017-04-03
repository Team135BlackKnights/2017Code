#include "DriveDistance.h"

DriveDistance::DriveDistance(double desiredDistanceToTravel, double motorPower, bool zeroDriveTrainEncoders) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->motorPower = motorPower;
	this->zeroDriveTrainEncoders = zeroDriveTrainEncoders;
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
	if (this->zeroDriveTrainEncoders) {
		CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::RIGHT_SIDE_ENCODER);
		CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::LEFT_SIDE_ENCODER);
		zeroedDriveTrainEncoders = true;
	}
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
	if (requiresDriveTrain == false) {
		CommandBase::driveTrain->is_aiming = true;
		requiresDriveTrain = true;
	}

	if (this->zeroDriveTrainEncoders && zeroedDriveTrainEncoders == false) {
		CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::RIGHT_SIDE_ENCODER);
		CommandBase::driveTrain->ZeroDriveTrainEncoder(DriveTrain::LEFT_SIDE_ENCODER);
		zeroedDriveTrainEncoders = true;
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
	zeroedDriveTrainEncoders = false;
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
