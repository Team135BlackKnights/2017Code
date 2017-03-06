#ifndef DriveLiftHang_H
#define DriveLiftHang_H

#include "../CommandBase.h"

class DriveLiftHang : public CommandBase {
private:
	static constexpr double LIFT_HANG_MOTOR_SPEED = 1.0;

	double liftHangCurrentValue = 0.0;
public:
	DriveLiftHang();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // DriveLiftHang_H
