#include "DriveUntilRobotAgainstBoiler.h"

DriveUntilRobotAgainstBoiler::DriveUntilRobotAgainstBoiler() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::lidars.get());
}

// Called just before this Command runs the first time
void DriveUntilRobotAgainstBoiler::Initialize() {
	CommandBase::lidars->ResetLidarWholeProcessVariables();
	lidarResetVaribales = true;
	receivedFirstLidarValue = false;
	againstBoiler = false;
}

// Called repeatedly when this Command is scheduled to run
void DriveUntilRobotAgainstBoiler::Execute() {
	if (lidarResetVaribales == false) {
		CommandBase::lidars->ResetLidarWholeProcessVariables();
		lidarResetVaribales = true;
	}

	lidarValue_IN = CommandBase::lidars->GetLidarValueWholeProcess(Lidars::DISTANCE_UNIT_ARRAY[Lidars::INCHES]);

	if (lidarValue_IN > 0.0 && receivedFirstLidarValue == false) {
		receivedFirstLidarValue = true;
	}
	else if (receivedFirstLidarValue && lidarValue_IN > LIDAR_DISTANCE_TO_TRAVEL_UNTIL) {
		CommandBase::driveTrain->DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
	}
	else if (receivedFirstLidarValue && lidarValue_IN <= LIDAR_DISTANCE_TO_TRAVEL_UNTIL) {
		CommandBase::driveTrain->DriveTank(0.0, 0.0);
		againstBoiler = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveUntilRobotAgainstBoiler::IsFinished() {
	return againstBoiler;
}

// Called once after isFinished returns true
void DriveUntilRobotAgainstBoiler::End() {
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	lidarResetVaribales = false;
	receivedFirstLidarValue = false;
	againstBoiler = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveUntilRobotAgainstBoiler::Interrupted() {
	End();
}
