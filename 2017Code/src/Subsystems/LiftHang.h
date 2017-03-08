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
	//frc::VictorSP* liftHangMotor2;

public:
	LiftHang();
	void InitDefaultCommand();

	void InitializeLiftHang(bool);
	void DriveLiftHang(double);
};

#endif  // LiftHang_H
