#include "DriveCollection.h"

DriveCollection::DriveCollection(bool driveForwards) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::collection.get());
	this->driveForwards = driveForwards;
}

// Called just before this Command runs the first time
void DriveCollection::Initialize() {

}

// Called repeatedly when this Command is scheduled to run
void DriveCollection::Execute() {
	if (this->driveForwards) {
		CommandBase::collection->DriveCollection(COLLECTION_MOTOR_POWER);
	}
	else if (this->driveForwards == false) {
		CommandBase::collection->DriveCollection(-COLLECTION_MOTOR_POWER);
	}
}

// Make this return true when this Command no longer needs to run execute()
bool DriveCollection::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void DriveCollection::End() {
	CommandBase::collection->DriveCollection(0.0);
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void DriveCollection::Interrupted() {
	End();
}
