#ifndef LiftHang_H
#define LiftHang_H

#include <Commands/Subsystem.h>
#include <VictorSP.h>
#include <Encoder.h>

class LiftHang : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::VictorSP* liftHangMotor;

	frc::Encoder* liftHangEncoder;

	static const int LIFT_HANG_ENCODER_A_CHANNEL = 3;
	static const int LIFT_HANG_ENCODER_B_CHANNEL = 4;

	static const bool REVERSE_ENCODER_DIRECTION = false;

	const frc::Encoder::EncodingType QUADRATURE_ENCODER = frc::Encoder::EncodingType::k4X;

public:
	LiftHang();
	void InitDefaultCommand();

	void InitializeLiftHang(bool);
	void DriveLiftHang(double);

	int GetEncoderPosition();
	void ZeroEncoder();
};

#endif  // LiftHang_H
