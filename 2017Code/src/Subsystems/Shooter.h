#ifndef Shooter_H
#define Shooter_H

#include <Commands/Subsystem.h>
#include <CANTalon.h>

class Shooter : public Subsystem {
public:
	static constexpr double DESIRED_VOLTAGE_CLOSE_SHOT = 7.7;
	static constexpr double DESIRED_VOLTAGE_FAR_SHOT = 9.125;

	static constexpr double SHOOTER_SETPOINT_MIN_RPM_CLOSE_SHOT = 2800.0; //  2600
	static constexpr double SHOOTER_SETPOINT_MAX_RPM_CLOSE_SHOT = 3300.0;  //  2900
	static constexpr double RANGE_OF_CLOSE_SHOT_SHOOTER_RPM = (SHOOTER_SETPOINT_MAX_RPM_CLOSE_SHOT - SHOOTER_SETPOINT_MIN_RPM_CLOSE_SHOT);

	static constexpr double SHOOTER_SETPOINT_RPM_CLOSE_SHOT = 2715.0;
	static constexpr double SHOOTER_SETPOINT_RPM_FAR_SHOT = 3040.0;

	static const int CLOSE_SHOT_PID_VALUES = 0;
	static const int FAR_SHOT_PID_VALUES = 1;

private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	CANTalon* shooterMotor;

	static const int SHOOTER_ENCODER_COUNT = 1024;
	static const int SHOOTER_ENCODER_QUADRATURE_COUNT = (SHOOTER_ENCODER_COUNT * 4);

	static const bool REVERSE_SHOOTER_ENCODER_DIRECTION = true;

	static constexpr double SHOOTER_GEAR_RATIO = (30.0/24.0);

	static constexpr double SHOOTER_MAX_VOLTAGE = 12.0;
	static constexpr double MAX_MOTOR_OUTPUT_VALUE = 1023.0;

	static constexpr double SHOOTER_SETPOINT_NU_PER_100MS_CLOSE_SHOT = 17476.0;
	static constexpr double PERCENT_VOLTAGE_CLOSE_SHOT = (DESIRED_VOLTAGE_CLOSE_SHOT/SHOOTER_MAX_VOLTAGE);
	static constexpr double MOTOR_OUTPUT_VALUE_FEEDFORWARD_CLOSE_SHOT = (PERCENT_VOLTAGE_CLOSE_SHOT * MAX_MOTOR_OUTPUT_VALUE);
	static constexpr double FEEDFORWARD_TERM_CLOSE_SHOT = (MOTOR_OUTPUT_VALUE_FEEDFORWARD_CLOSE_SHOT/SHOOTER_SETPOINT_NU_PER_100MS_CLOSE_SHOT);

	static constexpr double SHOOTER_SETPOINT_NU_PER_100MS_FAR_SHOT = 20800.0;
	static constexpr double PERCENT_VOLTAGE_FAR_SHOT = (DESIRED_VOLTAGE_FAR_SHOT/SHOOTER_MAX_VOLTAGE);
	static constexpr double MOTOR_OUTPUT_VALUE_FEEDFORWARD_FAR_SHOT = (PERCENT_VOLTAGE_FAR_SHOT * MAX_MOTOR_OUTPUT_VALUE);
	static constexpr double FEEDFORWARD_TERM_FAR_SHOT = (MOTOR_OUTPUT_VALUE_FEEDFORWARD_FAR_SHOT/SHOOTER_SETPOINT_NU_PER_100MS_FAR_SHOT);

	double closeShotKp = 0.0;
	double closeShotKi = 0.0;
	double closeShotKd = 0.0;
	double closeShotKf = 0.0;
	double closeShotPositivePeakVoltage = 0.0;
	double closeShotNegativePeakVoltage = 0.0;

	double farShotKp = 0.0;
	double farShotKi = 0.0;
	double farShotKd = 0.0;
	double farShotKf = 0.0;
	double farShotPositivePeakVoltage = 0.0;
	double farShotNegativePeakVoltage = 0.0;

	static constexpr double PB_CLOSE_SHOT_Kp = .25;  //  1.3
	static constexpr double PB_CLOSE_SHOT_Ki = 0.0;
	static constexpr double PB_CLOSE_SHOT_Kd = .5;  //  8.0
	static constexpr double PB_CLOSE_SHOT_Kf = .0352;  //  .0361
	static constexpr double PB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE = 9.0;
	static constexpr double PB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	static constexpr double PB_FAR_SHOT_Kp = 1.0;
	static constexpr double PB_FAR_SHOT_Ki = 0.0;
	static constexpr double PB_FAR_SHOT_Kd = 0.0;
	static constexpr double PB_FAR_SHOT_Kf = .0359;
	static constexpr double PB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE = 10.2;
	static constexpr double PB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	static constexpr double CB_CLOSE_SHOT_Kp = .25;  //  1.3
	static constexpr double CB_CLOSE_SHOT_Ki = 0.0;
	static constexpr double CB_CLOSE_SHOT_Kd = .5;  //  8.0
	static constexpr double CB_CLOSE_SHOT_Kf = .0352;  //  .0361
	static constexpr double CB_CLOSE_SHOT_POSITIVE_PEAK_VOLTAGE = 9.0;
	static constexpr double CB_CLOSE_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	static constexpr double CB_FAR_SHOT_Kp = 1.0;  //  1.0
	static constexpr double CB_FAR_SHOT_Ki = 0.0;  //  0.0
	static constexpr double CB_FAR_SHOT_Kd = 0.0;  //  0.0
	static constexpr double CB_FAR_SHOT_Kf = .0359;  //  /.0359
	static constexpr double CB_FAR_SHOT_POSITIVE_PEAK_VOLTAGE = 10.2;  //
	static constexpr double CB_FAR_SHOT_NEGATIVE_PEAK_VOLTAGE = -6.0;

	//  Variable for ShooterUpToSpeed() and GetShooterUpToSpeed()
	bool shooterUpToSpeed = false;

	double convertedThrottleValue = 0.0;
	double desiredCloseShotShooterSetpoint = 0.0;

	bool closeShot = true;
public:
	Shooter();
	void InitDefaultCommand();

	void InitializeShooterMotor(bool);

	void DriveShooterMotor(double);

	void ConfigureShooterMotorEncoder(bool);
	void ConfigureShooterVoltageMode();
	void ConfigureShooterPID();

	int GetShooterWheelRPM();
	int GetShooterWheelNUPer100Ms();
	void ZeroAccumulatedError();

	void SelectPIDProfileSlot(int);

	double GetShooterVoltage();
	double GetShooterMotorOutputCurrent();

	void ShooterUpToSpeed(bool);
	bool GetShooterUpToSpeed();

	double GetCloseShotShooterRPMGivenThrottleValue(double);
	void SetSwitchFarAndCloseShotShooterRPM(bool);
	bool GetSwitchBetweenFarAndCloseShotShooterRPM();
};

#endif  // Shooter_H
