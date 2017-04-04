#ifndef AimToGear_H
#define AimToGear_H

#include <Commands/CommandGroup.h>
#include "AimBot.h"
#include "AimBotWithUltrasonicSensors.h"

class AimToGear : public CommandGroup {
public:
	AimToGear(int camera_number);
	bool sonarGood = true;

};

#endif  // AimToGear_H
