#ifndef Lidars_H
#define Lidars_H

#include <Commands/Subsystem.h>
#include <I2C.h>
#include <DigitalOutput.h>

class Lidars : public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities

	const frc::I2C::Port MXP_PORT = frc::I2C::Port::kMXP;
	const frc::I2C::Port I2C_PORT = frc::I2C::Port::kOnboard;

	//  Lidar Pointer and Variables
	frc::I2C* lidar;

	static const int LIDAR_DEVICE_ADDRESS = 0x62;

	static const int CONFIGURE_REGISTER_ADDRESS = 0x00;
	static const int CONFIGURE_VALUE_TO_WRITE = 0x04;

	static const int UPPER_BYTE_REGISTER_ADDRESS = 0x0f;
	static const int LOWER_BYTE_REGISTER_ADDRESS = 0x10;

	static const int BOTH_BYTE_REGISTER_ADDRESS = 0x8f;

	static const int POWER_CONSUMPTION_MODE_AFTER_READING_VALUES_ADDRESS = 0x04;
	static const int VALUE_TO_SEND_TO_TURN_OFF_DETECTOR_BIAS_BETWEEN_ACQUISITIONS = 0b00001000;

	uint8_t* upperByteDataPointer;
	uint8_t* lowerByteDataPointer;

	int convertedUpperByteData = 0;
	int convertedLowerByteData = 0;

	int shiftedUpperByte = 0;

	int finalValue = 0;
	int convertedFinalValue = 0;

	char* charPointerConvertedData = 0;
	char charPointerData = 0;
	int finalIntConvertedData = 0;


	//  I2C Multiplexer Pointer and Variables
	frc::I2C* i2CMultiplexer;

	static const int I2C_MULTIPLEXER_DEVICE_ADDRESS = 0x70;

	uint8_t* convertedValueToSendToI2CMultiplexer;
	uint8_t convertedByte = 0;

	int differenceBetweenLIDARValues = 0.0;
	double angleBetweenLIDARValues = 0.0;
	double extraDistanceOfLIDAR_M = 0.0;

	int finalNonZeroLidarValue = 0;

	double returnLidarValue = 0.0;

	double savedLidarValue = 0.0;

	frc::DigitalOutput* lidarPowerEnabledDO;
	static const int LIDAR_POWER_ENABLE_DIGITAL_OUTPUT_PORT = 16;
	bool lidarTurnedOn = false;

	double storedLidarValue = 0.0;
	double distanceFromBumperToGuardrail = 0.0;
	static constexpr double DISTANCE_FROM_FRONT_ULTRASONIC_SENSOR_TO_BUMPER = 2.75;
	static constexpr double START_OF_DESIRED_LIDAR_VALUE = 35.5;  //  37 //  35
	double desiredLidarValue = 0.0;
	double desiredDistanceToTravelToHopper = 0.0;
public:
	Lidars();
	void InitDefaultCommand();

	void InitializeLidarsAndI2CMultiplexer();

	void InitializeLidarPowerEnablePin();
	void TurnLidarOnOff(bool);

	void OpenLidarChannelOnMultiplexer(int);
	void ConfigureLidar();
	void TurnOffDetectorBiasBetweenLidarAcquisitions();
	int GetUpperByte();
	int GetLowerByte();
	double GetLidarValue(int, int, int);
	void ResetLidarWholeProcessVariables();

	double ConvertCentimetersToInches(int);
	double ConvertCentimetersToMeters(int);
	int ConvertUint8_tPointer_To_Int(uint8_t*);
	uint8_t* ConvertUint8_t_To_Uint8_tPointer(uint8_t);

	void StoreLidarValueForHopperAndShoot(double);
	double GetDistanceToTravelToHopper(double);

	static const int NUM_OF_UNITS = 3;
	static const int CENTIMETERS = 0;
	static const int INCHES = 1;
	static const int METERS = 2;

	static constexpr int DISTANCE_UNIT_ARRAY[NUM_OF_UNITS] = {CENTIMETERS, INCHES, METERS};

	static const uint8_t VALUE_TO_OPEN_LIDAR_CHANNEL_6_RIGHT_LIDAR = 0b01000000;
	static const uint8_t VALUE_TO_OPEN_LIDAR_CHANNEL_7_LEFT_LIDAR = 0b10000000;
	static const uint8_t VALUE_TO_OPEN_FRONT_LIDAR_CHANNEL_7 = 0b10000000;

	static const bool TURN_LIDAR_ON = true;
	static const bool TURN_LIDAR_OFF = !TURN_LIDAR_ON;
};

#endif  // Lidars_H
