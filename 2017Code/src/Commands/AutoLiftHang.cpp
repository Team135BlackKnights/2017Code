#include "AutoLiftHang.h"

AutoLiftHang::AutoLiftHang() {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	Requires(CommandBase::liftHang.get());
	Requires(CommandBase::pdp.get());

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void AutoLiftHang::Initialize() {
	initializeTimerAndLiftHangCurrentBooleans = true;
	initializeTimer = false;
	waitForLiftHangMotorToStartUp = false;
	calculatedNoRopeLiftHangCurrentValue = false;
	startTimeWaitForCalculatingNoRopeLiftHangCurrentValue = false;
	timer->Reset();
	timer->Start();
}

// Called repeatedly when this Command is scheduled to run
void AutoLiftHang::Execute() {
	if (initializeTimerAndLiftHangCurrentBooleans == false) {
		initializeTimerAndLiftHangCurrentBooleans = true;
		initializeTimer = false;
		waitForLiftHangMotorToStartUp = false;
		calculatedNoRopeLiftHangCurrentValue = false;
		startTimeWaitForCalculatingNoRopeLiftHangCurrentValue = false;
		timer->Reset();
		timer->Start();
	}

	timerValue = timer->Get();
	currentLiftHangCurrentValue = CommandBase::pdp->GetCurrentOfPDPPort(PDP::HANGING_MOTOR_PDP_PORT);
	CommandBase::liftHang->DriveLiftHang(LIFT_HANG_MOTOR_POWER);

	if (waitForLiftHangMotorToStartUp == false) {
		if (initializeTimer == false) {
			timer->Reset();
			timer->Start();
			initializeTimer = true;
		}
		else if (initializeTimer) {
			if (timerValue >= WAIT_FOR_LIFT_HANG_MOTOR_START_UP) {
				waitForLiftHangMotorToStartUp = true;
			}
		}
	}
	else if (calculatedNoRopeLiftHangCurrentValue == false && waitForLiftHangMotorToStartUp) {
		if (startTimeWaitForCalculatingNoRopeLiftHangCurrentValue == false) {
			timer->Reset();
			timer->Start();
			startTimeWaitForCalculatingNoRopeLiftHangCurrentValue = true;
		}
		else if (startTimeWaitForCalculatingNoRopeLiftHangCurrentValue) {
			if (timerValue >= TIME_TO_WAIT_TO_CALCULATE_NO_ROPE_LIFT_HANG_CURRENT) {
				noRopeLiftHangRPMCurrent = currentLiftHangCurrentValue;
				calculatedNoRopeLiftHangCurrentValue = true;
			}
		}
	}
	else if (calculatedNoRopeLiftHangCurrentValue && liftHangOnRope == false) {
		if (currentLiftHangCurrentValue >= (noRopeLiftHangRPMCurrent + LIFT_HANG_CURRENT_CHANGE_FROM_NO_ROPE_TO_ROPE)) {
			liftHangOnRope = true;
		}
	}
	else if (liftHangOnRope) {

	}
}

// Make this return true when this Command no longer needs to run execute()
bool AutoLiftHang::IsFinished() {
	return false;
}

// Called once after isFinished returns true
void AutoLiftHang::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void AutoLiftHang::Interrupted() {

}
