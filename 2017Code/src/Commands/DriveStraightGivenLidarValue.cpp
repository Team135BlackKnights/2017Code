#include "DriveStraightGivenLidarValue.h"

DriveStraightGivenLidarValue::DriveStraightGivenLidarValue(double driveTrainMotorPower) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::driveTrain.get());

	this->driveTrainMotorPower = driveTrainMotorPower;
}

// Called just before this Command runs the first time
void DriveStraightGivenLidarValue::Initialize() {
	frontUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();
	desiredDistanceToTravel = CommandBase::lidars->GetDistanceToTravelToHopper(frontUltrasonicSensorValue);
	getDesiredDistanceToTravel = true;

	initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	initializeDistanceTraveled = true;

	CommandBase::driveTrain->ZeroGyroAngle();
	zeroGyro = true;

	droveToDistance = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveStraightGivenLidarValue::Execute() {
	if (getDesiredDistanceToTravel == false) {
		frontUltrasonicSensorValue = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();
		desiredDistanceToTravel = CommandBase::lidars->GetDistanceToTravelToHopper(frontUltrasonicSensorValue);
		getDesiredDistanceToTravel = true;
	}

	if (initializeDistanceTraveled == false) {
		initialDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
		initializeDistanceTraveled = true;
	}

	if (zeroGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroGyro = true;
	}

	currentGyroAngle = CommandBase::driveTrain->GetGyroAngle();

	currentDistanceTraveled = CommandBase::driveTrain->GetDistance(DriveTrain::RIGHT_SIDE_ENCODER);
	differenceBetweenCurrentAndInitialDistance = (fabs(currentDistanceTraveled - initialDistanceTraveled));
	if (differenceBetweenCurrentAndInitialDistance >= desiredDistanceToTravel) {
		CommandBase::driveTrain->DriveTank(0.0, 0.0);
		droveToDistance = true;
	}
	else if (differenceBetweenCurrentAndInitialDistance < desiredDistanceToTravel) {
		CommandBase::driveTrain->DriveStraightWithGyro(this->driveTrainMotorPower, currentGyroAngle);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveStraightGivenLidarValue::IsFinished() {
	return droveToDistance;
}

// Called once after isFinished returns true
void DriveStraightGivenLidarValue::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	getDesiredDistanceToTravel = false;
	initializeDistanceTraveled = false;
	zeroGyro = false;
	droveToDistance = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveStraightGivenLidarValue::Interrupted() {
	End();
}
