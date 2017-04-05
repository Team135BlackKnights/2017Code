#ifndef ReadLidarValues_H
#define ReadLidarValues_H

#include "../CommandBase.h"

class ReadLidarValues : public CommandBase {
private:
	bool openFrontLidarChannel = false;
	bool readLidarValueForFirstTime = false;
	bool configuredLidar = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	double lidarValueIN = 0;
public:
	ReadLidarValues();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadLidarValues_H
