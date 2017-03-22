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

	static const int LIFT_HANG_ENCODER_A_CHANNEL = 6;
	static const int LIFT_HANG_ENCODER_B_CHANNEL = 7;
	static const bool REVERSE_ENCODER_DIRECTION = false;
	const frc::Encoder::EncodingType QUADRATURE_ENCODER = frc::Encoder::EncodingType::k4X;

	int liftHangEncoderValue = 0;
	double numberOfRotationsOfLiftHang = 0.0;

public:
	LiftHang();
	void InitDefaultCommand();

	void InitializeLiftHang(bool);
	void DriveLiftHang(double);

	void InitializeLiftHangEncoder();
	void ZeroLiftHangEncoder();
	int GetLiftHangEncoderRawValue();
	int GetLiftHangEncoderValue();
	double GetNumberOfRotationsOfLiftHang();

	static const int ENCODER_COUNT = 1024;
	static const int QUADRATURE_ENCODER_COUNT = (ENCODER_COUNT * 4);
	static const int LIFT_HANG_READY_ENCODER_POSITION = (QUADRATURE_ENCODER_COUNT / 4);
};

#endif  // LiftHang_H
