#include "LiftHang.h"
#include "../RobotMap.h"
#include "Commands/ReadLiftHangEncoderValue.h"

LiftHang::LiftHang() : Subsystem("LiftHang") {

}

void LiftHang::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new ReadLiftHangEncoderValue());
}

void LiftHang::InitializeLiftHang(bool competitionBot) {
	if (competitionBot) {
		liftHangMotor = new frc::VictorSP(CB_HANG_VICTOR_PWM_PORT);
		liftHangMotor->SetInverted(CB_LIFT_HANG_MOTOR_INVERTED);
	}
	else if (competitionBot == false) {
		liftHangMotor = new frc::VictorSP(PB_HANG_VICTOR_PWM_PORT);
		liftHangMotor->SetInverted(PB_LIFT_HANG_MOTOR_INVERTED);
	}
}

void LiftHang::DriveLiftHang(double motorPower) {
	liftHangMotor->Set(motorPower);
}

void LiftHang::InitializeLiftHangEncoder() {
	liftHangEncoder = new frc::Encoder(LIFT_HANG_ENCODER_A_CHANNEL, LIFT_HANG_ENCODER_B_CHANNEL, REVERSE_ENCODER_DIRECTION, QUADRATURE_ENCODER);
}

void LiftHang::ZeroLiftHangEncoder() {
	liftHangEncoder->Reset();
}

int LiftHang::GetLiftHangEncoderRawValue() {
	return liftHangEncoder->GetRaw();
}

int LiftHang::GetLiftHangEncoderValue() {
	return liftHangEncoder->Get();
}

double LiftHang::GetNumberOfRotationsOfLiftHang() {
	liftHangEncoderValue = this->GetLiftHangEncoderValue();
	numberOfRotationsOfLiftHang = (((double)liftHangEncoderValue)/QUADRATURE_ENCODER_COUNT);
	return numberOfRotationsOfLiftHang;
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
