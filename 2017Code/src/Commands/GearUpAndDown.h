#ifndef GearUpAndDown_H
#define GearUpAndDown_H

#include "../CommandBase.h"

class GearUpAndDown : public CommandBase {
private:
	static constexpr double GEAR_HOLDER_MOTOR_POWER = .5;

	bool minLimitSwitchPressed = false;
	bool maxLimitSwitchPressed = false;

	int gearDirectionCounter = 1;
public:
	GearUpAndDown();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // GearUpAndDown_H
