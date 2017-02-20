#include "AutoGearOnPeg.h"

AutoGearOnPeg::AutoGearOnPeg() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
}

// Called just before this Command runs the first time
void AutoGearOnPeg::Initialize() {
	CommandBase::lidars->OpenCloseI2CChannelLines(Lidars::VALUE_TO_OPEN_LIDAR_CHANNEL_6_GEAR);
	startMeasuringLidarValue = true;
	lidarConfigureTimePassed = false;
	startMovingTowardsGear = false;
	startPuttingGearOnPeg = false;
	gearOnPeg = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoGearOnPeg::Execute() {
	if (startMeasuringLidarValue) {
		if (lidarConfigureTimePassed == false) {
			CommandBase::lidars->ConfigureLidar();
			lidarConfigureTimePassed = true;
		}
		else if (lidarConfigureTimePassed) {
			gearLidarUpperByte = CommandBase::lidars->GetUpperByte();
			gearLidarLowerByte = CommandBase::lidars->GetLowerByte();
			gearLidarValueInCentimeters = CommandBase::lidars->GetLidarValue(gearLidarLowerByte, gearLidarUpperByte);
			lidarConfigureTimePassed = false;
			startMovingTowardsGear = true;
		}
	}

	if (startMovingTowardsGear) {
		if (gearLidarValueInCentimeters > DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_CM) {
			CommandBase::driveTrain->DriveTank(DRIVE_TRAIN_MOTOR_POWER, DRIVE_TRAIN_MOTOR_POWER);
		}
		else if (gearLidarValueInCentimeters<= DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_CM) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			startMeasuringLidarValue = false;
			startMovingTowardsGear = false;
			startPuttingGearOnPeg = true;
		}
	}

	if (startPuttingGearOnPeg) {
		lowerLimitSwitchValue = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
		if (lowerLimitSwitchValue) {
			CommandBase::gearHolder->DriveGearHolder(0);
			startPuttingGearOnPeg = false;
			gearOnPeg = true;
		}
		else {
			CommandBase::gearHolder->DriveGearHolder(-GEAR_HOLDER_MOTOR_POWER);
		}
	}


}

// Make this return true when this Command no longer needs to run execute()
bool AutoGearOnPeg::IsFinished() {
	return gearOnPeg;
}

// Called once after isFinished returns true
void AutoGearOnPeg::End() {
	startMeasuringLidarValue = true;
	lidarConfigureTimePassed = false;
	startMovingTowardsGear = false;
	startPuttingGearOnPeg = false;
	gearOnPeg = false;
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	CommandBase::gearHolder->DriveGearHolder(0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGearOnPeg::Interrupted() {
	End();
}
