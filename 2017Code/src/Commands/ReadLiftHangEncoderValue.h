#ifndef ReadLiftHangEncoderValue_H
#define ReadLiftHangEncoderValue_H

#include "../CommandBase.h"

class ReadLiftHangEncoderValue : public CommandBase {
private:
	int liftHangRawEncoderValue = 0;
public:
	ReadLiftHangEncoderValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadLiftHangEncoderValue_H
