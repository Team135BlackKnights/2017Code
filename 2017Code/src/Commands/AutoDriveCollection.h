#ifndef AutoDriveCollection_H
#define AutoDriveCollection_H

#include "../CommandBase.h"

class AutoDriveCollection : public CommandBase {
private:
	bool initializeAutoDriveCollection = false;
	bool autoDriveCollection = false;
	static constexpr double COLLECTION_MOTOR_POWER = 1.0;
	bool doneAutoDrivingCollection = false;
public:
	AutoDriveCollection();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoDriveCollection_H
