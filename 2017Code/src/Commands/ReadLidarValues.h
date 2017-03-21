#ifndef ReadLidarValues_H
#define ReadLidarValues_H

#include "../CommandBase.h"

class ReadLidarValues : public CommandBase {
private:
	bool configuredLidar = false;

	int leftLidarUpperByte = 0;
	int leftLidarLowerByte = 0;
	double leftLidarValueIN = 0.0;

	int rightLidarUpperByte = 0;
	int rightLidarLowerByte = 0;
	double rightLidarValueIN = 0.0;

	static const bool GET_LEFT_LIDAR_VALUE_CHANNEL_6 = false;
	static const bool GET_RIGHT_LIDAR_VALUE_CHANNEL_7 = false;
public:
	ReadLidarValues();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadLidarValues_H
