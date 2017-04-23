#ifndef GetLidarValueForHopperAndShoot_H
#define GetLidarValueForHopperAndShoot_H

#include "../CommandBase.h"

class GetLidarValueForHopperAndShoot : public CommandBase {
private:
	bool configureLidar = false;
	int lidarLowerByte = 0;
	int lidarUpperByte = 0;
	double lidarValueIN = 0.0;

	static constexpr double GET_LIDAR_VALUE_FOR_HOPPER_AND_SHOOT_TIMEOUT = .15;
public:
	GetLidarValueForHopperAndShoot();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // GetLidarValueForHopperAndShoot_H
