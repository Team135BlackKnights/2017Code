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

	static const int UPPER_LIMIT_SWITCH_PORT = 1;
	static const int LOWER_LIMIT_SWITCH_PORT = 2;

	bool upperLimitSwitchValue = false;
	bool lowerLimitSwitchValue = false;

	double gearHolderMotorPower = 0.0;

public:
	GearHolder();
	void InitDefaultCommand();

	void InitializeGearHolderMotor(bool);

	void DriveGearHolder(double);
};

#endif  // GearHolder_H
