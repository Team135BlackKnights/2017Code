#include "Shooter.h"
#include "../RobotMap.h"
#include "Commands/DriveShooter.h"

Shooter::Shooter() : Subsystem("Shooter") {

}

void Shooter::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new DriveShooter());
}

void Shooter::InitializeShooterMotor(bool competitionBot) {
	if (competitionBot) {
		shooterMotor = new CANTalon(CB_SHOOTER_MOTOR_TALON_ID);
		shooterMotor->SetInverted(CB_SHOOTER_MOTOR_INVERTED);
	}
	else if (competitionBot == false) {
		shooterMotor = new CANTalon(PB_SHOOTER_MOTOR_TALON_ID);
		shooterMotor->SetInverted(PB_SHOOTER_MOTOR_INVERTED);
	}
}

void Shooter::ConfigureShooterVoltageMode() {
	shooterMotor->SetControlMode(CANTalon::ControlMode::kVoltage);
	shooterMotor->Set(0.0);
}

void Shooter::ConfigureShooterPID() {
	shooterMotor->SetControlMode(CANTalon::ControlMode::kSpeed);
	shooterMotor->Set(0.0);
	shooterMotor->ConfigNominalOutputVoltage(0.0, -0.0);
	shooterMotor->ConfigPeakOutputVoltage(12.0, -6.0);
}

void Shooter::DriveShooterMotor(double motorPower) {
	shooterMotor->Set(motorPower);
}

void Shooter::ConfigureShooterMotorEncoder() {
	shooterMotor->ConfigEncoderCodesPerRev(SHOOTER_ENCODER_COUNT);
	shooterMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
	shooterMotor->SetSensorDirection(REVERSE_SHOOTER_ENCODER_DIRECTION);
	shooterMotor->SetPosition(0.0);
	shooterMotor->SetNominalClosedLoopVoltage(SHOOTER_MAX_VOLTAGE);
	shooterMotor->SelectProfileSlot(0);
	shooterMotor->SetP(.4);
	shooterMotor->SetI(.0008);
	shooterMotor->SetD(10.0);
	shooterMotor->SetF(0.0);
	shooterMotor->SetIzone(2500.0);
}

int Shooter::GetShooterWheelRPM() {
	return shooterMotor->GetSpeed();
}

int Shooter::GetShooterWheelNUPer100Ms() {
	return shooterMotor->GetEncVel();
}

double Shooter::GetShooterMotorOutputCurrent() {
	return shooterMotor->GetOutputCurrent();
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
