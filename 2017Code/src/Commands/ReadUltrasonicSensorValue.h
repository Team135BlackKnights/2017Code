#ifndef ReadUltrasonicSensorValue_H
#define ReadUltrasonicSensorValue_H

#include "../CommandBase.h"

class ReadUltrasonicSensorValue : public CommandBase {
private:
	double ultrasonicSensorValueInches = 0.0;
public:
	ReadUltrasonicSensorValue();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // ReadUltrasonicSensorValue_H
