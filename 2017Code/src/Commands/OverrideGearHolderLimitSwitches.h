#ifndef OverrideGearHolderLimitSwitches_H
#define OverrideGearHolderLimitSwitches_H

#include "../CommandBase.h"

class OverrideGearHolderLimitSwitches : public CommandBase {
public:
	OverrideGearHolderLimitSwitches();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // OverrideGearHolderLimitSwitches_H
