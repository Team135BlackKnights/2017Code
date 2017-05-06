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
	agitatorMotorPart2 = new frc::VictorSP(CB_AGITATOR_2_VICTOR_PWM_PORT);
	agitatorMotorPart2->SetInverted(CB_AGITATOR_2_INVERTED);
}

void Agitator::DriveAgitator(double agitator1MotorPower, double agitator2MotorPower) {
	agitatorMotor->Set(agitator1MotorPower);
	agitatorMotorPart2->Set(agitator2MotorPower);
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
