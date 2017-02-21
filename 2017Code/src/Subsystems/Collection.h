#ifndef Collection_H
#define Collection_H

#include <Commands/Subsystem.h>
#include <VictorSP.h>

class Collection : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	frc::VictorSP* collectionMotor;
public:
	Collection();
	void InitDefaultCommand();

	void InitializeCollectionMotor(bool);

	void DriveCollection(double);
};

#endif  // Collection_H
