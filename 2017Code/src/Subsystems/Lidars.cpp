#include "Lidars.h"
#include "../RobotMap.h"
#include "Commands/ReadLidarValues.h"

Lidars::Lidars() : Subsystem("Lidar") {

}

void Lidars::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	//SetDefaultCommand(new ReadLidarValues());
}

void Lidars::InitializeLidarsAndI2CMultiplexer() {
	lidar = new frc::I2C(MXP_PORT, LIDAR_DEVICE_ADDRESS);

	upperByteDataPointer = new uint8_t;
	lowerByteDataPointer = new uint8_t;

	charPointerConvertedData = new char;

	i2CMultiplexer = new frc::I2C(MXP_PORT, I2C_MULTIPLEXER_DEVICE_ADDRESS);

	convertedValueToSendToI2CMultiplexer = new uint8_t;
}

void Lidars::OpenLidarChannelOnMultiplexer(int byteToSend) {
	convertedValueToSendToI2CMultiplexer = this->ConvertUint8_t_To_Uint8_tPointer(byteToSend);
	i2CMultiplexer->WriteBulk(convertedValueToSendToI2CMultiplexer, 1);
}

void Lidars::ConfigureLidar() {
	lidar->Write(CONFIGURE_REGISTER_ADDRESS, CONFIGURE_VALUE_TO_WRITE);
}

int Lidars::GetUpperByte() {
	lidar->Read(UPPER_BYTE_REGISTER_ADDRESS, 1, upperByteDataPointer);
	convertedUpperByteData = this->ConvertUint8_tPointer_To_Int(upperByteDataPointer);
	return convertedUpperByteData;
}

int Lidars::GetLowerByte() {
	lidar->Read(LOWER_BYTE_REGISTER_ADDRESS, 1, lowerByteDataPointer);
	convertedLowerByteData = this->ConvertUint8_tPointer_To_Int(lowerByteDataPointer);
	return convertedLowerByteData;
}

double Lidars::GetLidarValue(int lowerByte, int upperByte, int distanceUnits) {
	shiftedUpperByte = upperByte << 8;
	finalValue = (shiftedUpperByte + lowerByte);

	if (distanceUnits == DISTANCE_UNIT_ARRAY[CENTIMETERS]) {
		returnLidarValue = ((double)finalValue);
	}
	else if (distanceUnits == DISTANCE_UNIT_ARRAY[INCHES]) {
		returnLidarValue = this->ConvertCentimetersToInches(finalValue);
	}
	else if (distanceUnits == DISTANCE_UNIT_ARRAY[METERS]) {
		returnLidarValue = this->ConvertCentimetersToMeters(finalValue);
	}
	return returnLidarValue;
}

double Lidars::ConvertCentimetersToInches(int valueInCM) {
	return (((double)valueInCM)/2.54);
}

double Lidars::ConvertCentimetersToMeters(int valueinCM) {
	return (valueinCM/100.0);
}

int Lidars::ConvertUint8_tPointer_To_Int(uint8_t* data) {
	charPointerConvertedData = reinterpret_cast<char*>(data);
	strncpy(&charPointerData, charPointerConvertedData, 1);
	finalIntConvertedData = ((int)charPointerData);
	return finalIntConvertedData;
}

uint8_t* Lidars::ConvertUint8_t_To_Uint8_tPointer(uint8_t data) {
	convertedByte = reinterpret_cast<uint8_t&>(data);
	return &convertedByte;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
