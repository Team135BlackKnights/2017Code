#include "TurnDriveTrainAngle.h"

TurnDriveTrainAngle::TurnDriveTrainAngle(double desiredAngleToTurn, double motorPower, bool turnRight, bool setTimeout) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	this->desiredAngleToTurn = desiredAngleToTurn;
	this->motorPower = motorPower;
	this->turnRight = turnRight;
	this->setTimeout = setTimeout;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void TurnDriveTrainAngle::Initialize() {
	CommandBase::driveTrain->ZeroGyroAngle();
	zeroedGyro = true;
	turnAngleComplete = false;
	CommandBase::driveTrain->is_aiming = true;
	requiresDriveTrain = true;
	timer->Reset();
	timer->Start();
	initializeTimer = true;
}

// Called repeatedly when this Command is scheduled to run
void TurnDriveTrainAngle::Execute() {
	if (requiresDriveTrain == false) {
		CommandBase::driveTrain->is_aiming = true;
		requiresDriveTrain = true;
	}

	if (zeroedGyro == false) {
		CommandBase::driveTrain->ZeroGyroAngle();
		zeroedGyro = true;
	}

	if (initializeTimer == false) {
		timer->Reset();
		timer->Start();
		initializeTimer = true;
	}

	timerValue = timer->Get();

	currenGyroAngle = fabs(CommandBase::driveTrain->GetGyroAngle());

	if (currenGyroAngle >= this->desiredAngleToTurn) {
		CommandBase::driveTrain->RotateTank(0.0, this->turnRight);
		turnAngleComplete = true;
	}
	else {
		CommandBase::driveTrain->RotateTank(this->motorPower, this->turnRight);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool TurnDriveTrainAngle::IsFinished() {
	if (setTimeout) {
		return (turnAngleComplete || (timerValue >= TURN_ROBOT_INTO_HOPPER_TIMEOUT));
	}
	else if (setTimeout == false) {
		return turnAngleComplete;
	}
}

// Called once after isFinished returns true
void TurnDriveTrainAngle::End() {
	CommandBase::driveTrain->RotateTank(0.0, this->turnRight);
	zeroedGyro = false;
	turnAngleComplete = false;
	CommandBase::driveTrain->is_aiming = false;
	requiresDriveTrain = false;
	initializeTimer = false;
	timer->Stop();
	timer->Reset();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void TurnDriveTrainAngle::Interrupted() {
	End();
}
