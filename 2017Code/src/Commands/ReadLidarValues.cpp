#include "ReadLidarValues.h"

ReadLidarValues::ReadLidarValues() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void ReadLidarValues::Initialize() {
	configuredLidar = false;
}

// Called repeatedly when this Command is scheduled to run
void ReadLidarValues::Execute() {

	if (configuredLidar == false) {
		if (GET_LEFT_LIDAR_VALUE_CHANNEL_6) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_6_LEFT_LIDAR);
			CommandBase::lidars->ConfigureLidar();
		}

		if (GET_RIGHT_LIDAR_VALUE_CHANNEL_7) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_7_RIGHT_LIDAR);
			CommandBase::lidars->ConfigureLidar();
		}
		configuredLidar = true;
	}
	else if (configuredLidar) {
		if (GET_LEFT_LIDAR_VALUE_CHANNEL_6) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_6_LEFT_LIDAR);
			leftLidarUpperByte = CommandBase::lidars->GetUpperByte();
			leftLidarLowerByte = CommandBase::lidars->GetLowerByte();
			leftLidarValueIN = CommandBase::lidars->GetLidarValue(leftLidarLowerByte, leftLidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		}

		if (GET_RIGHT_LIDAR_VALUE_CHANNEL_7) {
			CommandBase::lidars->OpenLidarChannelOnMultiplexer(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_7_RIGHT_LIDAR);
			rightLidarUpperByte = CommandBase::lidars->GetUpperByte();
			rightLidarLowerByte = CommandBase::lidars->GetLowerByte();
			rightLidarValueIN = CommandBase::lidars->GetLidarValue(rightLidarLowerByte, rightLidarUpperByte, Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
		}
		configuredLidar = false;
	}

	frc::SmartDashboard::PutNumber("Left Lidar Value IN:", leftLidarValueIN);
	frc::SmartDashboard::PutNumber("Right Lidar Value IN:", rightLidarValueIN);
}

// Make this return true when this Command no longer needs to run execute()
bool ReadLidarValues::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadLidarValues::End() {
	configuredLidar = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadLidarValues::Interrupted() {
	End();
}
