#ifndef ReadLidarValue_H
#define ReadLidarValue_H

#include "../CommandBase.h"
#include <Timer.h>

class ReadLidarValue : public CommandBase {
private:
	bool resetLidarVariables = false;
	double lidarValue_IN = 0;
public:
	ReadLidarValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadLidarValue_H
