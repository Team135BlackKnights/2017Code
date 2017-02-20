#include "WaitTime.h"

WaitTime::WaitTime(double timeToWait) {
	// Use Requires() here to declare subsystem dependencies
	// eg. Requires(Robot::chassis.get());
	this->timeToWait = timeToWait;

	timer = new frc::Timer();
}

// Called just before this Command runs the first time
void WaitTime::Initialize() {
	timer->Reset();
	timer->Start();
	doneWaiting = false;
}

// Called repeatedly when this Command is scheduled to run
void WaitTime::Execute() {
	currentTimerValue = timer->Get();
	if (currentTimerValue >= timeToWait) {
		doneWaiting = true;
	}
}

// Make this return true when this Command no longer needs to run execute()
bool WaitTime::IsFinished() {
	return doneWaiting;
}

// Called once after isFinished returns true
void WaitTime::End() {
	doneWaiting = false;
	timer->Stop();
	timer->Reset();
}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void WaitTime::Interrupted() {
	End();
}
