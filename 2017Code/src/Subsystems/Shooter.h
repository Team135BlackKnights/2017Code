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

	static constexpr double DESIRED_VOLTAGE_CLOSE_SHOT = 7.7;
	static constexpr double SHOOTER_SETPOINT_NU_PER_100MS_CLOSE_SHOT = 19450.0;
	double percentVoltageCloseShot = (DESIRED_VOLTAGE_CLOSE_SHOT/SHOOTER_MAX_VOLTAGE);
	static constexpr double MAX_MOTOR_OUTPUT_VALUE = 1023.0;

	double motorOutputCloseShot = 0.0;
	double calculatedFeedforwardTerm = 0.0;
	double feedforwardTerm = 0.0;

public:
	Shooter();
	void InitDefaultCommand();

	void InitializeShooterMotor(bool);

	void ConfigureShooterVoltageMode();
	void ConfigureShooterPID();

	void DriveShooterMotor(double);

	void ConfigureShooterMotorEncoder();

	double ConfigureFeedForwardTerm();
	int GetShooterWheelRPM();
	int GetShooterWheelNUPer100Ms();
	void ZeroAccumulatedError();

	double GetShooterMotorOutputCurrent();

	static constexpr double SHOOTER_SETPOINT_RPM_CLOSE_SHOT = 2850.0;
};

#endif  // Shooter_H
