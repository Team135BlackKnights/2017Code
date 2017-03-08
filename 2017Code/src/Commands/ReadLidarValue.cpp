#include "ReadLidarValue.h"

ReadLidarValue::ReadLidarValue() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void ReadLidarValue::Initialize() {
	CommandBase::lidars->ResetLidarWholeProcessVariables();
	resetLidarVariables = true;
}

// Called repeatedly when this Command is scheduled to run
void ReadLidarValue::Execute() {
	if (resetLidarVariables == false) {
		CommandBase::lidars->ResetLidarWholeProcessVariables();
		resetLidarVariables = true;
	}
	lidarValue_IN = CommandBase::lidars->GetLidarValueWholeProcess(Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);
	//std::cout << "Lidar Value Inches: " << lidarValue_IN << std::endl;
}

// Make this return true when this Command no longer needs to run execute()
bool ReadLidarValue::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void ReadLidarValue::End() {
	resetLidarVariables = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ReadLidarValue::Interrupted() {
	End();
}
