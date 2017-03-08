#ifndef GearHolder_H
#define GearHolder_H

#include <Commands/Subsystem.h>
#include <VictorSP.h>
#include <DigitalInput.h>
#include <Servo.h>

class GearHolder : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::VictorSP* gearHolderMotor;

	frc::DigitalInput* upperLimitSwitch;
	frc::DigitalInput* lowerLimitSwitch;

	bool upperLimitSwitchValue = false;
	bool lowerLimitSwitchValue = false;

	double gearHolderMotorPower = 0.0;

	bool limitSwitchValue = false;

	frc::Servo* gearHolderServo;

	//  Practice Bot PWM Port 4
	static const int GEAR_HOLDER_SERVO_PWM_PORT = 0;

public:
	GearHolder();
	void InitDefaultCommand();

	void InitializeGearHolderMotor(bool);

	void DriveGearHolderMotor(double);

	bool GetLimitSwitchValue(int);

	double GetGearHolderServoValue();
	void SetGearHolderServoValue(double);

	static const int UPPER_LIMIT_SWITCH_PORT = 1;
	static const int LOWER_LIMIT_SWITCH_PORT = 0;

	static constexpr double SERVO_OUT_POSITION = 0.0;
	static constexpr double SERVO_IN_POSITION = 180.0;
};

#endif  // GearHolder_H
