#ifndef DriveLiftHang_H
#define DriveLiftHang_H

#include "../CommandBase.h"

class DriveLiftHang : public CommandBase {
private:
	double liftHangMotorPower;
public:
	DriveLiftHang(double);
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveLiftHang_H
