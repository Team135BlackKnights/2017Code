#include "CameraAdjustWhileDrivingToGear.h"

CameraAdjustWhileDrivingToGear::CameraAdjustWhileDrivingToGear() {
	// Use Requires() here to declare subsystem dependencies
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
}

// Called just before this Command runs the first time
void CameraAdjustWhileDrivingToGear::Initialize() {
	done = false;
	CommandBase::driveTrain->is_aiming = true;
}

// Called repeatedly when this Command is scheduled to run
void CameraAdjustWhileDrivingToGear::Execute() {
	ultrasonicValue = CommandBase::ultrasonicSensor->GetGearUltrasonicSensorValueInches();

	if(ultrasonicValue > DISTANCE_TO_STOP)
	{
		tableOffset = GetTableOffset(ultrasonicValue);
		actualOffset = CommandBase::server->get_angle(1); //actually returns an offset
		if(tableOffset == 0) done = true;
		//Curve < 0 will turn left
		std::cout << "offset value: " << actualOffset << " table offset: " << tableOffset << std::endl;
		curveValue = ((actualOffset - tableOffset) * proportionalConstant);
		SmartDashboard::PutNumber("curve Value", curveValue);
		if(curveValue < -1) curveValue = -1;
		else if(curveValue > 1) curveValue = 1;
		driveTrain->chassis->Drive(driveSpeed, curveValue);
	}
	else
	{
		done = true;
	}
}

double CameraAdjustWhileDrivingToGear::GetTableOffset(double ultrasonicInput)
{
	if(ultrasonicInput >= 43.5)
		return -104.5;
	else if(ultrasonicInput >= 36)
		return -124.5;
	else if(ultrasonicInput >= 30.5)
		return -145.5;
	else if(ultrasonicInput >= 24.5)
		return -187.5;
	else
		return 0;
}

// Make this return true when this Command no longer needs to run execute()
bool CameraAdjustWhileDrivingToGear::IsFinished() {
	return done || (CommandBase::oi->GetAction(CommandBase::oi->LEFT_DRIVE_JOYSTICK, 10));
}

// Called once after isFinished returns true
void CameraAdjustWhileDrivingToGear::End() {
	CommandBase::driveTrain->is_aiming = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void CameraAdjustWhileDrivingToGear::Interrupted() {

}
