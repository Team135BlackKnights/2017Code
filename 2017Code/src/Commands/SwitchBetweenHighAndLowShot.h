#ifndef SwitchBetweenHighAndLowShot_H
#define SwitchBetweenHighAndLowShot_H

#include "../CommandBase.h"

class SwitchBetweenHighAndLowShot : public CommandBase {
private:
	bool closeShot = true;
	bool switchedFarAndCloseShotRPM = false;
public:
	SwitchBetweenHighAndLowShot();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // SwitchBetweenHighAndLowShot_H
