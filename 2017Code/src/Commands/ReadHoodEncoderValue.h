#ifndef ReadHoodEncoderValue_H
#define ReadHoodEncoderValue_H

#include "../CommandBase.h"

class ReadHoodEncoderValue : public CommandBase {
private:
	int shooterHoodEncoderValue = 0;
	int maxAngleLimitSwitchValue = 0;
	int minAngleLimitSwitchValue = 0;
public:
	ReadHoodEncoderValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadHoodEncoderValue_H
