#ifndef ReadPWMLidarValue_H
#define ReadPWMLidarValue_H

#include "../CommandBase.h"

class ReadPWMLidarValue : public CommandBase {
private:
	bool startReceivingLidarValue = false;

	double lidarValueCM = 0.0;
public:
	ReadPWMLidarValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadPWMLidarValue_H
