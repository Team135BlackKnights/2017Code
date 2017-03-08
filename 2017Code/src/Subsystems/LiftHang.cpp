#include "LiftHang.h"
#include "../RobotMap.h"

LiftHang::LiftHang() : Subsystem("LiftHang") {

}

void LiftHang::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
}

void LiftHang::InitializeLiftHang(bool competitionBot) {
	if (competitionBot) {
		liftHangMotor = new frc::VictorSP(CB_HANG_VICTOR_PWM_PORT);
		liftHangMotor->SetInverted(CB_LIFT_HANG_MOTOR_INVERTED);
		//liftHangMotor2 = new frc::VictorSP(CB_HANG_VICTOR_2_PWM_PORT);
		//liftHangMotor2->SetInverted(CB_LIFT_HANG_MOTOR_2_INVERTED);
	}
	else if (competitionBot == false) {
		liftHangMotor = new frc::VictorSP(PB_HANG_VICTOR_PWM_PORT);
		liftHangMotor->SetInverted(PB_LIFT_HANG_MOTOR_INVERTED);
		//liftHangMotor2 = new frc::VictorSP(PB_HANG_VICTOR_2_PWM_PORT);
		//liftHangMotor2->SetInverted(PB_LIFT_HANG_MOTOR_2_INVERTED);
	}
}

void LiftHang::DriveLiftHang(double motorPower) {
	liftHangMotor->Set(motorPower);
	//liftHangMotor2->Set(motorPower);
}
// Put methods for controlling this subsystem
// here. Call these from Commands.
