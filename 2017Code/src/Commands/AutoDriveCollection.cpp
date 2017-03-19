#include "AutoDriveCollection.h"

AutoDriveCollection::AutoDriveCollection() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::collection.get());
}

// Called just before this Command runs the first time
void AutoDriveCollection::Initialize() {
	CommandBase::collection->SetAutoDriveCollection(true);
	initializeAutoDriveCollection = true;
	doneAutoDrivingCollection = false;
}

// Called repeatedly when this Command is scheduled to run
void AutoDriveCollection::Execute() {
	if (initializeAutoDriveCollection == false) {
		CommandBase::collection->SetAutoDriveCollection(true);
		initializeAutoDriveCollection = true;
	}

	autoDriveCollection = CommandBase::collection->GetAutoDriveCollection();

	if (autoDriveCollection) {
		CommandBase::collection->DriveCollection(COLLECTION_MOTOR_POWER);
	}
	else if (autoDriveCollection == false) {
		CommandBase::collection->DriveCollection(0.0);
		doneAutoDrivingCollection = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoDriveCollection::IsFinished() {
	return doneAutoDrivingCollection;
}

// Called once after isFinished returns true
void AutoDriveCollection::End() {
	CommandBase::collection->DriveCollection(0.0);
	initializeAutoDriveCollection = false;
	doneAutoDrivingCollection = false;
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoDriveCollection::Interrupted() {
	End();
}
