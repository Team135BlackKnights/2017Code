#ifndef Lidars_H
#define Lidars_H

#include <Commands/Subsystem.h>
#include <I2C.h>

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

	static constexpr double DISTANCE_BETWEEN_LIDARS_IN = 5;
	static constexpr double DISTANCE_BETWEN_LIDARS_CM = (DISTANCE_BETWEEN_LIDARS_IN * 2.54);
	static constexpr double DISTANCE_BETWEEN_CENTER_OF_BOILER_TO_EDGE_IN = 17.5;
	static constexpr double DISTANCE_BETWEEN_CENTER_OF_BOILER_TO_EDGE_CM = (DISTANCE_BETWEEN_CENTER_OF_BOILER_TO_EDGE_IN * 2.54);
	static constexpr double DISTANCE_BETWEEN_CENTER_OF_BOILER_TO_EDGE_M = (DISTANCE_BETWEEN_CENTER_OF_BOILER_TO_EDGE_CM/100.0);

	int differenceBetweenLIDARValues = 0.0;
	double angleBetweenLIDARValues = 0.0;
	double extraDistanceOfLIDAR_M = 0.0;

	int finalNonZeroLidarValue = 0;
public:
	Lidars();
	void InitDefaultCommand();

	void InitializeLidars();

	void OpenLidarChannelOnMultplexer();
	void ConfigureLidar();
	int GetUpperByte();
	int GetLowerByte();
	int GetLidarValue(int, int);

	double ConvertCentimetersToInches(int);
	double ConvertCentimetersToMeters(int);
	int ConvertUint8_tPointer_To_Int(uint8_t*);
	uint8_t* ConvertUint8_t_To_Uint8_tPointer(uint8_t);

	static const uint8_t VALUE_TO_OPEN_LIDAR_CHANNEL_6_SHOOTER = 0b01000000;
};

#endif  // Lidars_H
