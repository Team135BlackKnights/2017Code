#include "GearHolder.h"
#include "../RobotMap.h"

GearHolder::GearHolder() : Subsystem("GearHolder") {
	upperLimitSwitch = new frc::DigitalInput(UPPER_LIMIT_SWITCH_PORT);
	lowerLimitSwitch = new frc::DigitalInput(LOWER_LIMIT_SWITCH_PORT);
}

void GearHolder::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void GearHolder::InitializeGearHolderMotor(bool competitionBot) {
	if (competitionBot) {
		gearHolderMotor = new frc::VictorSP(CB_GEAR_VICTOR_PWM_PORT);
		gearHolderMotor->SetInverted(CB_GEAR_HOLDER_INVERTED);
	}
	else if (competitionBot == false) {
		gearHolderMotor = new frc::VictorSP(PB_GEAR_VICTOR_PWM_PORT);
		gearHolderMotor->SetInverted(PB_GEAR_HOLDER_INVERTED);
	}
}

void GearHolder::DriveGearHolder(double motorPower) {
	upperLimitSwitchValue = !upperLimitSwitch->Get();
	lowerLimitSwitchValue = !lowerLimitSwitch->Get();

	if (upperLimitSwitchValue) {
		gearHolderMotorPower = fmin(0.0, motorPower);
	}
	else if (lowerLimitSwitchValue) {
		gearHolderMotorPower = fmax(0.0, motorPower);
	}
	else {
		gearHolderMotorPower = motorPower;
	}
	gearHolderMotor->Set(gearHolderMotorPower);
}

bool GearHolder::GetLimitSwitchValue(int limitSwitchDigitalInputNumber) {
	if (limitSwitchDigitalInputNumber == 1) {
		limitSwitchValue = !upperLimitSwitch->Get();
	}
	else if (limitSwitchDigitalInputNumber == 2) {
		limitSwitchValue = !lowerLimitSwitch->Get();
	}
	return limitSwitchValue;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
