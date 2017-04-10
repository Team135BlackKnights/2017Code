#ifndef AutoMoveGearHolder_H
#define AutoMoveGearHolder_H

#include "../CommandBase.h"

class AutoMoveGearHolder : public CommandBase {
private:
	bool moveGearHolderUpwards;

	static constexpr double GEAR_HOLDER_MOTOR_POWER = .9;

	bool minLimitSwitchPressed = false;
	bool maxLimitSwitchPressed = false;

	bool movedGearHolderToDesiredLimitSwitch = false;

	static constexpr double AUTO_MOVE_GEAR_HOLDER_TIMEOUT = 2.3;
public:
	AutoMoveGearHolder(bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoMoveGearHolder_H
