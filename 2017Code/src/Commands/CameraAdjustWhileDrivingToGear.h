#ifndef CameraAdjustWhileDrivingToGear_H
#define CameraAdjustWhileDrivingToGear_H

#include "../CommandBase.h"

class CameraAdjustWhileDrivingToGear : public CommandBase {
public:
	CameraAdjustWhileDrivingToGear();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
private:
	double GetTableOffset(double ultrasonicInput);
	double ultrasonicValue = 135.0;
	static constexpr double DISTANCE_TO_STOP = 24;
	double voltageLeft = 0.0;
	double voltageRight = 0.0;
	double tableOffset = 0.0;
	double actualOffset = 0.0;
	static constexpr double driveSpeed = -.35;
	double curveValue = 0.0;
	static constexpr double proportionalConstant = .01;
	bool done = false;
};

#endif  // CameraAdjustWhileDrivingToGear_H
