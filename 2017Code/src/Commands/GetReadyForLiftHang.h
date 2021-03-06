#ifndef GetReadyForLiftHang_H
#define GetReadyForLiftHang_H

#include "../CommandBase.h"

class GetReadyForLiftHang : public CommandBase {
private:
	int liftHangCurrentEncoderPosition = 0;
	double currentLiftHangNumberOfRotations = 0.0;

	double lowNumberOfCurrentRotations = 0.0;
	double highNumberOfCurrentRotations = 0.0;
	int liftHangDesiredEncoderValueForLowNumberOfRotations = 0;

	bool calculateDesiredEncoderPosition = false;
	int desiredEncoderPosition = 0;

	int differenceBetweenCurrentAndDesiredLiftHangEncoderValue = 0;

	bool liftHangInDesiredPosition = false;

	static constexpr double LIFT_HANG_MOTOR_POWER = .2;

	static constexpr int VARIABILITY_IN_DESIRED_LIFT_HANG_ENCODER_POSITION = 135;
public:
	GetReadyForLiftHang();
	void Initialize();
	void Execute();
	bool IsFinished();
	void End();
	void Interrupted();
};

#endif  // GetReadyForLiftHang_H
