#include "ReadPWMLidarValue.h"

ReadPWMLidarValue::ReadPWMLidarValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::pwmLidar.get());
}

// Called just before this Command runs the first time
void ReadPWMLidarValue::Initialize() {
	CommandBase::pwmLidar->StartReceivingLidarValues();
	startReceivingLidarValue = true;
}

// Called repeatedly when this Command is scheduled to run
void ReadPWMLidarValue::Execute() {
	if (startReceivingLidarValue == false) {
		CommandBase::pwmLidar->StartReceivingLidarValues();
		startReceivingLidarValue = true;
	}

	lidarValueCM = CommandBase::pwmLidar->GetLidarPWMValue();
	std::cout << "Lidar Value CM: " << lidarValueCM << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool ReadPWMLidarValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadPWMLidarValue::End() {
	startReceivingLidarValue = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadPWMLidarValue::Interrupted() {
	End();
}
