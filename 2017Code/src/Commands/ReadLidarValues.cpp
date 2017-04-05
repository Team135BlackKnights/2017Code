#include "ReadLidarValues.h"
#include <Timer.h>

ReadLidarValues::ReadLidarValues() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void ReadLidarValues::Initialize() {
	CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
	openFrontLidarChannel = true;
	readLidarValueForFirstTime = false;
	configuredLidar = false;
}

// Called repeatedly when this Command is scheduled to run
void ReadLidarValues::Execute() {
	if (openFrontLidarChannel == false) {
		CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7);
		openFrontLidarChannel = true;
	}

	if (configuredLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		readLidarValueForFirstTime = true;
		configuredLidar = true;
	}
	else if (configuredLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		frc::Wait(.002);
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		lidarValueIN = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		configuredLidar = false;
	}

	/*if (configuredLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configuredLidar = true;
	} */

	frc::SmartDashboard::PutNumber("Front Lidar Valuee:", lidarValueIN);
	//std::cout << "Front Lidar Value IN: " << lidarValueIN << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool ReadLidarValues::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadLidarValues::End() {
	openFrontLidarChannel = false;
	readLidarValueForFirstTime = false;
	configuredLidar = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadLidarValues::Interrupted() {
	End();
}
