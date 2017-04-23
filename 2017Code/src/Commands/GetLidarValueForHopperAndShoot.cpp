#include "GetLidarValueForHopperAndShoot.h"

GetLidarValueForHopperAndShoot::GetLidarValueForHopperAndShoot() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void GetLidarValueForHopperAndShoot::Initialize() {
	SetTimeout(GET_LIDAR_VALUE_FOR_HOPPER_AND_SHOOT_TIMEOUT);
}

// Called repeatedly when this Command is scheduled to run
void GetLidarValueForHopperAndShoot::Execute() {
	if (configureLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configureLidar = true;
	}
	else if (configureLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		frc::Wait(.002);
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		lidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configureLidar = false;
	}

	if (lidarValueIN > 0.0) {
		CommandBase::lidars->StoreLidarValueForHopperAndShoot(lidarValueIN);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool GetLidarValueForHopperAndShoot::IsFinished() {
	return IsTimedOut();
}

// Called once after isFinished returns true
void GetLidarValueForHopperAndShoot::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void GetLidarValueForHopperAndShoot::Interrupted() {

}
