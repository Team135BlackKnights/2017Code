#include "GearHolder.h"
#include "../RobotMap.h"
#include <iostream>
#include "Commands/DriveGearHolder.h"

GearHolder::GearHolder() : Subsystem("GearHolder") {

}

void GearHolder::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());

	SetDefaultCommand(new DriveGearHolder());
}

void GearHolder::InitializeGearHolderMotor(bool competitionBot) {
	if (competitionBot) {
		gearHolderMotor = new frc::VictorSP(CB_GEAR_VICTOR_PWM_PORT);
		gearHolderMotor->SetInverted(CB_GEAR_HOLDER_INVERTED);
		gearHolderServo = new frc::Servo(CB_GEAR_HOLDER_SERVO_PWM_PORT);
	}
	else if (competitionBot == false) {
		gearHolderMotor = new frc::VictorSP(PB_GEAR_VICTOR_PWM_PORT);
		gearHolderMotor->SetInverted(PB_GEAR_HOLDER_INVERTED);
		gearHolderServo = new frc::Servo(PB_GEAR_HOLDER_SERVO_PWM_PORT);
	}
	upperLimitSwitch = new frc::DigitalInput(UPPER_LIMIT_SWITCH_PORT);
	lowerLimitSwitch = new frc::DigitalInput(LOWER_LIMIT_SWITCH_PORT);

	photoElectricSensor = new frc::DigitalInput(PHOTO_ELECTRIC_SENSOR_PORT);
}

void GearHolder::DriveGearHolderMotor(double motorPower) {
	upperLimitSwitchValue = this->GetLimitSwitchValue(UPPER_LIMIT_SWITCH_PORT);
	lowerLimitSwitchValue = this->GetLimitSwitchValue(LOWER_LIMIT_SWITCH_PORT);

	if (upperLimitSwitchValue) {
		if (overrideUpperLimitSwitch == false) {
			gearHolderMotorPower = fmin(0.0, motorPower);
		}
		else if (overrideUpperLimitSwitch) {
			gearHolderMotorPower = motorPower;
		}
	}
	else if (lowerLimitSwitchValue) {
		if (overrideUpperLimitSwitch == false) {
			gearHolderMotorPower = fmax(0.0, motorPower);
		}
		else if (overrideUpperLimitSwitch) {
			gearHolderMotorPower = motorPower;
		}
	}
	else {
		gearHolderMotorPower = motorPower;
	}
	gearHolderMotor->Set(gearHolderMotorPower);
}

void GearHolder::OverrideUpperLimitSwitch() {
	overrideUpperLimitSwitch = !overrideUpperLimitSwitch;
}

bool GearHolder::GetLimitSwitchValue(int limitSwitchDigitalInputNumber) {
	if (limitSwitchDigitalInputNumber == UPPER_LIMIT_SWITCH_PORT) {
		limitSwitchValue = !upperLimitSwitch->Get();
	}
	else if (limitSwitchDigitalInputNumber == LOWER_LIMIT_SWITCH_PORT) {
		limitSwitchValue = !lowerLimitSwitch->Get();
	}
	return limitSwitchValue;
}

double GearHolder::GetGearHolderServoValue() {
	return gearHolderServo->GetAngle();
}

void GearHolder::SetGearHolderServoValue(double servoPosition) {
	gearHolderServo->SetAngle(servoPosition);
}

bool GearHolder::GetPhotoElectricSensorValue() {
	return !photoElectricSensor->Get();
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
