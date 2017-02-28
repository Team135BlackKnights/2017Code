#include "ReadLidarValue.h"

ReadLidarValue::ReadLidarValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void ReadLidarValue::Initialize() {
	configuredLidar = false;
}

// Called repeatedly when this Command is scheduled to run
void ReadLidarValue::Execute() {
	if (configuredLidar == false) {
		CommandBase::lidars->ConfigureLidar();
		configuredLidar = true;
	}
	else if (configuredLidar) {
		lidarUpperByte = CommandBase::lidars->GetUpperByte();
		lidarLowerByte = CommandBase::lidars->GetLowerByte();
		lidarValue_CM = CommandBase::lidars->GetLidarValue(lidarLowerByte, lidarUpperByte);
		lidarValue_IN = CommandBase::lidars->ConvertCentimetersToInches(lidarValue_CM);
		configuredLidar = false;
	}

	std::cout << "Lidar Value CM: " << lidarValue_CM << std::endl;
	std::cout << "Lidar Value IN: " << lidarValue_IN << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool ReadLidarValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadLidarValue::End() {
	configuredLidar = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadLidarValue::Interrupted() {
	End();
}
