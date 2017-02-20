#include "Agitator.h"
#include "../RobotMap.h"

Agitator::Agitator() : Subsystem("Agitator") {

}

void Agitator::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void Agitator::InitializeAgitatorMotor(bool competitionBot) {
	if (competitionBot) {
		agitatorMotor = new frc::VictorSP(CB_AGITATOR_VICTOR_PWM_PORT);
		agitatorMotor->SetInverted(CB_AGITATOR_INVERTED);
	}
	else if (competitionBot == false) {
		agitatorMotor = new frc::VictorSP(PB_AGITATOR_VICTOR_PWM_PORT);
		agitatorMotor->SetInverted(PB_AGITATOR_INVERTED);
	}
}

void Agitator::DriveAgitator(double motorPower) {
	agitatorMotor->Set(motorPower);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
