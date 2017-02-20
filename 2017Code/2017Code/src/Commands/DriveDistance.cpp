#include "DriveDistance.h"

DriveDistance::DriveDistance(double desiredDistanceToTravel, double motorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	this->desiredDistanceToTravel = desiredDistanceToTravel;
	this->motorPower = motorPower;
}

// Called just before this Command runs the first time
void DriveDistance::Initialize() {
	initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::LEFT_SIDE_ENCODER);
	distanceTraveled = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveDistance::Execute() {
	measuredCurrentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::LEFT_SIDE_ENCODER);
	actualCurrentDistanceTraveled = (fabs(measuredCurrentDistanceTraveled - initialDistanceTraveled));
	if (actualCurrentDistanceTraveled >= this->desiredDistanceToTravel) {
		CommandBase::driveTrain->DriveTank(0.0, 0.0);
		distanceTraveled = true;
	}
	else {
		CommandBase::driveTrain->DriveTank(motorPower, motorPower);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveDistance::IsFinished() {
	return distanceTraveled;
}

// Called once after isFinished returns true
void DriveDistance::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveDistance::Interrupted() {
	End();
}
