#ifndef SwitchShooterPIDProfileSlot_H
#define SwitchShooterPIDProfileSlot_H

#include "../CommandBase.h"

class SwitchShooterPIDProfileSlot : public CommandBase {
private:
	int pidProfileSlotCounter = 2;
	bool doneSwitchPIDProfile = false;
public:
	SwitchShooterPIDProfileSlot();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // SwitchShooterPIDProfileSlot_H
