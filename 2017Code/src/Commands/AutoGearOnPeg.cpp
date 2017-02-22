#include "AutoGearOnPeg.h"

AutoGearOnPeg::AutoGearOnPeg() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::driveTrain.get());
	Requires(CommandBase::ultrasonicSensor.get());
	Requires(CommandBase::gearHolder.get());
}

// Called just before this Command runs the first time
void AutoGearOnPeg::Initialize() {
	startMovingTowardsGear = true;
	startPuttingGearOnPeg = false;
	gearOnPeg = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoGearOnPeg::Execute() {
	std::cout << "Auto Gear On Peg" << std::endl;

	ultrasonicValue = CommandBase::ultrasonicSensor->GetUltrasonicSensorValueInches();
	std::cout << "Ultrasonic Sensor Value: " << ultrasonicValue << std::endl;

	if (startMovingTowardsGear) {
		if (ultrasonicValue > DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_IN) {
			CommandBase::driveTrain->DriveTank(-DRIVE_TRAIN_MOTOR_POWER, -DRIVE_TRAIN_MOTOR_POWER);
		}
		else if (ultrasonicValue <= DISTANCE_AWAY_FROM_PEG_TO_DROP_GEAR_IN) {
			CommandBase::driveTrain->DriveTank(0.0, 0.0);
			startMovingTowardsGear = false;
			startPuttingGearOnPeg = true;
		}
	}

	if (startPuttingGearOnPeg) {
		lowerLimitSwitchValue = CommandBase::gearHolder->GetLimitSwitchValue(GearHolder::LOWER_LIMIT_SWITCH_PORT);
		std::cout << "Limit Switch Value: " << lowerLimitSwitchValue << std::endl;
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
	startMovingTowardsGear = true;
	startPuttingGearOnPeg = false;
	gearOnPeg = false;
	CommandBase::driveTrain->DriveTank(0.0, 0.0);
	CommandBase::gearHolder->DriveGearHolder(0.0);
	std::cout << "Add On Gear Endeddddddddddddddddddddd" << std::endl;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoGearOnPeg::Interrupted() {
	End();
}
