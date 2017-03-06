#ifndef AutoLiftHang_H
#define AutoLiftHang_H

#include "../CommandBase.h"
#include <Timer.h>

class AutoLiftHang : public CommandBase {
private:
	static constexpr double LIFT_HANG_MOTOR_POWER = .7;
	double currentLiftHangCurrentValue = 0.0;

	double noRopeLiftHangRPMCurrent = 0.0;
	double startLiftHangRPMCurrent = 0.0;
	double panelPressedRPMCurrent = 0.0;
	static constexpr double LIFT_HANG_CURRENT_CHANGE_FROM_NO_ROPE_TO_ROPE = 5.0;

	frc::Timer* timer;
	double timerValue = 0.0;
	static constexpr double WAIT_FOR_LIFT_HANG_MOTOR_START_UP = .5;
	static constexpr double TIME_TO_WAIT_TO_CALCULATE_NO_ROPE_LIFT_HANG_CURRENT = .5;
	static constexpr double

	bool initializeTimerAndLiftHangCurrentBooleans = false;
	bool initializeTimer = false;
	bool waitForLiftHangMotorToStartUp = false;
	bool calculatedNoRopeLiftHangCurrentValue = false;
	bool startTimeWaitForCalculatingNoRopeLiftHangCurrentValue = false;
	bool liftHangOnRope = true;
public:
	AutoLiftHang();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // AutoLiftHang_H
