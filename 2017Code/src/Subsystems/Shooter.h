#ifndef Shooter_H
#define Shooter_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class Shooter : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	CANTalon* shooterMotor;

	static const int SHOOTER_ENCODER_COUNT = 1024;
	static const int SHOOTER_ENCODER_QUADRATURE_COUNT = (SHOOTER_ENCODER_COUNT * 4);

	static const bool REVERSE_SHOOTER_ENCODER_DIRECTION = true;

	static constexpr double SHOOTER_MAX_VOLTAGE = 12.0;

	static constexpr double SHOOTER_GEAR_RATIO = (1.5/1.0);

public:
	Shooter();
	void InitDefaultCommand();

	void InitializeShooterMotor(bool);

	void ConfigureShooterVoltageMode();
	void ConfigureShooterPID();

	void DriveShooterMotor(double);

	void ConfigureShooterMotorEncoder();
	int GetShooterWheelRPM();
	int GetShooterWheelNUPer100Ms();

	double GetShooterMotorOutputCurrent();
};

#endif  // Shooter_H
