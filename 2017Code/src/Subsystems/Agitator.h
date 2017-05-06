#ifndef Agitator_H
#define Agitator_H

#include <Commands/Subsystem.h>
#include <VictorSP.h>

class Agitator : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::VictorSP* agitatorMotor;
	frc::VictorSP* agitatorMotorPart2;

public:
	Agitator();
	void InitDefaultCommand();

	void InitializeAgitatorMotor(bool);

	void DriveAgitator(double, double);
};

#endif  // Agitator_H
