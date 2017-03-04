#include "RaiseLowerHoodToCorrectAngle.h"

RaiseLowerHoodToCorrectAngle::RaiseLowerHoodToCorrectAngle(bool closeShot, int shooterAngledPosition) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::shooterHood.get());
	Requires(CommandBase::lidars.get());

	this->shooterAngledPosition = shooterAngledPosition;
}

// Called just before this Command runs the first time
void RaiseLowerHoodToCorrectAngle::Initialize() {
	CommandBase::lidars->ResetLidarWholeProcessVariables();
	resetLidarVariables = true;
	calculatedDesiredAngle = false;
	driveHoodToDesiredAngle = false;
}

// Called repeatedly when this Command is scheduled to run
void RaiseLowerHoodToCorrectAngle::Execute() {
	if (resetLidarVariables == false) {
		CommandBase::lidars->ResetLidarWholeProcessVariables();
		resetLidarVariables = true;
	}
	if (lidarValue_CM == 0.0) {
		lidarValue_CM = CommandBase::lidars->GetLidarValueWholeProcess(Lidars::DISTANCE_UNIT_ARRAY[Lidars::CENTIMETERS]);
	}
	else if (lidarValue_CM > 0.0 && calculatedDesiredAngle == false) {
		desiredHoodAngleToTurn = CommandBase::shooterHood->GetDesiredAngleOfShooterHood(lidarValue_CM, this->closeShot, this->shooterAngledPosition);
		if (desiredHoodAngleToTurn > 0.0) {
			calculatedDesiredAngle = true;
		}
	}
	else if (calculatedDesiredAngle && driveHoodToDesiredAngle == false) {
		driveHoodToDesiredAngle = CommandBase::shooterHood->DriveShooterHoodMotorToDesiredAngle(calculatedDesiredAngle, SHOOTER_HOOD_MOTOR_POWER);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool RaiseLowerHoodToCorrectAngle::IsFinished() {
	return driveHoodToDesiredAngle;
}

// Called once after isFinished returns true
void RaiseLowerHoodToCorrectAngle::End() {
	resetLidarVariables = false;
	calculatedDesiredAngle = false;
	driveHoodToDesiredAngle = false;
	lidarValue_CM = 0.0;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void RaiseLowerHoodToCorrectAngle::Interrupted() {
	End();
}
