#ifndef DriveHoodAndShooterWithLidar_H
#define DriveHoodAndShooterWithLidar_H

#include "../CommandBase.h"

class DriveHoodAndShooterWithLidar : public CommandBase {
private:
	bool turnOnLidar = false;
	bool openI2CMultiplexerChannelForLidar = false;
	bool configureLidar = false;

	int lidarUpperByte = 0;
	int lidarLowerByte = 0;
	double lidarValueIN = 0.0;

	bool startGettingDesiredHoodEncoderValues = false;

	int desiredHoodEncoderValue = 0;

	bool configurePID = false;
	bool configureVoltageMode = false;
	bool selectedPIDSlot = false;

	bool intializeDesiredHoodEncoderPosition = false;
	bool hoodAtDesiredEncoderPosition = false;
public:
	DriveHoodAndShooterWithLidar();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveHoodAndShooterWithLidar_H
