#ifndef ReadUltrasonicSensorValue_H
#define ReadUltrasonicSensorValue_H

#include "../CommandBase.h"

class ReadUltrasonicSensorValue : public CommandBase {
private:
	double leftUltrasonicSensorValueInches = 0.0;
	double rightUltrasonicSensorValueInches = 0.0;

	int readingUltrasonicSensorsCounter = 1;
public:
	ReadUltrasonicSensorValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadUltrasonicSensorValue_H
