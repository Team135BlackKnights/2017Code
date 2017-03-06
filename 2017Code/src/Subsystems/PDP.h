#ifndef PDP_H
#define PDP_H

#include <Commands/Subsystem.h>
#include <PowerDistributionPanel.h>

class PDP : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::PowerDistributionPanel* pdp;
	static const int PDP_ID = 0;

public:
	PDP();
	void InitDefaultCommand();
	void InitializePDP();
	double GetCurrentOfPDPPort(int);

	//  4 or 6 or 7
	static const int AGITATOR_TALON_PDP_PORT = 7;

	static const int HANGING_MOTOR_PDP_PORT = 10;
};

#endif  // PDP_H
