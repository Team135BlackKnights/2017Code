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

	static constexpr double SHOOTER_GEAR_RATIO = (30.0/24.0);

	static constexpr double DESIRED_VOLTAGE_CLOSE_SHOT = 7.7;
	static constexpr double SHOOTER_SETPOINT_NU_PER_100MS_CLOSE_SHOT = 19450.0;
	static constexpr double PERCENT_VOLTAGE_CLOSE_SHOT = (DESIRED_VOLTAGE_CLOSE_SHOT/SHOOTER_MAX_VOLTAGE);
	static constexpr double MAX_MOTOR_OUTPUT_VALUE = 1023.0;

	double motorOutputCloseShot = 0.0;
	double calculatedFeedforwardTerm = 0.0;
	double feedforwardTerm = 0.0;

	static const int CLOSE_SHOT_PID_VALUES = 0;
	static const int FAR_SHOT_PID_VALUES = 1;

	bool closeShotPIDProfileSlot = false;
public:
	Shooter();
	void InitDefaultCommand();

	void InitializeShooterMotor(bool);

	void DriveShooterMotor(double);

	void ConfigureShooterMotorEncoder();
	void ConfigureShooterVoltageMode();
	void ConfigureShooterPID();

	double ConfigureFeedForwardTerm();
	int GetShooterWheelRPM();
	int GetShooterWheelNUPer100Ms();
	void ZeroAccumulatedError();
	void SelectPIDProfileSlot(int);
	bool GetPIDProfileSlot();

	double GetShooterMotorOutputCurrent();

	static constexpr double SHOOTER_SETPOINT_RPM_CLOSE_SHOT = 2850.0;
};

#endif  // Shooter_H
