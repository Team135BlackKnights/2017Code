#ifndef AutoLiftHang_H
#define AutoLiftHang_H

#include "../CommandBase.h"

class AutoLiftHang : public CommandBase {
private:
	static constexpr double LIFT_HANG_MOTOR_POWER = .7;
	double liftHangCurrentValue = 0.0;
public:
	AutoLiftHang();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoLiftHang_H
