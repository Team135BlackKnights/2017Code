#ifndef ReadLidarValue_H
#define ReadLidarValue_H

#include "../CommandBase.h"
#include <Timer.h>

class ReadLidarValue : public CommandBase {
private:
	bool openLidarChannel = false;
	bool configuredLidar = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	int lidarValue_CM = 0;
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
