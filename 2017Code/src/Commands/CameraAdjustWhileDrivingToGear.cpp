#include "CameraAdjustWhileDrivingToGear.h"

CameraAdjustWhileDrivingToGear::CameraAdjustWhileDrivingToGear() {
	// Use Requires() here to declare subsystem dependencies
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
}

// Called just before this Command runs the first time
void CameraAdjustWhileDrivingToGear::Initialize() {
	done = false;
}

// Called repeatedly when this Command is scheduled to run
void CameraAdjustWhileDrivingToGear::Execute() {
	ultrasonicValue = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();

	if(ultrasonicValue < DISTANCE_TO_STOP)
	{
		tableOffset = GetTableOffset(ultrasonicValue);
		actualOffset = CommandBase::server->get_angle(1); //actually returns an offset
		//Curve < 0 will turn left
		curveValue = ((actualOffset - tableOffset) * proportionalConstant);
		driveTrain->chassis->Drive(driveSpeed, curveValue);
	}
	else
	{
		done = true;
	}
}

double CameraAdjustWhileDrivingToGear::GetTableOffset(double ultrasonicInput)
{
	return 10;
}

// Make this return true when this Command no longer needs to run execute()
bool CameraAdjustWhileDrivingToGear::IsFinished() {
	return done;
}

// Called once after isFinished returns true
void CameraAdjustWhileDrivingToGear::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CameraAdjustWhileDrivingToGear::Interrupted() {

}
