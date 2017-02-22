#ifndef GearHolder_H
#define GearHolder_H

#include <Commands/Subsystem.h>
#include <VictorSP.h>
#include <DigitalInput.h>

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

public:
	GearHolder();
	void InitDefaultCommand();

	void InitializeGearHolderMotor(bool);

	void DriveGearHolder(double);

	bool GetLimitSwitchValue(int);

	static const int UPPER_LIMIT_SWITCH_PORT = 1;
	static const int LOWER_LIMIT_SWITCH_PORT = 0;
};

#endif  // GearHolder_H
