#include "Collection.h"
#include "../RobotMap.h"

Collection::Collection() : Subsystem("Collection") {

}

void Collection::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Collection::InitializeCollectionMotor(bool competitionBot) {
	if (competitionBot) {
		collectionMotor = new frc::VictorSP(CB_COLLECTION_VICTOR_PWM_PORT);
		collectionMotor->SetInverted(CB_COLLECTION_INVERTED);
	}
	else if (competitionBot == false) {
		collectionMotor = new frc::VictorSP(PB_COLLECTION_VICTOR_PWM_PORT);
		collectionMotor->SetInverted(PB_COLLECTION_INVERTED);
	}
}

void Collection::DriveCollection(double motorPower) {
	collectionMotor->Set(motorPower);
}

void Collection::SetAutoDriveCollection(bool autoDriveCollection) {
	this->autoDriveCollection = autoDriveCollection;
}

bool Collection::GetAutoDriveCollection() {
	return (this->autoDriveCollection);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
