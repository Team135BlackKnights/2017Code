#ifndef DriveCollection_H
#define DriveCollection_H

#include "../CommandBase.h"

class DriveCollection : public CommandBase {
private:
	bool driveForwards;
	static constexpr double COLLECTION_MOTOR_POWER = 1.0;
public:
	DriveCollection(bool);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveCollection_H
