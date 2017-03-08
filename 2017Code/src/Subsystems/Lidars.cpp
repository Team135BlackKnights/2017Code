#include "Lidars.h"
#include "../RobotMap.h"
#include "Commands/ReadLidarValue.h"

Lidars::Lidars() : Subsystem("Lidar") {

}

void Lidars::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	//SetDefaultCommand(new ReadLidarValue());
}

void Lidars::InitializeLidars() {
	lidar = new frc::I2C(MXP_PORT, LIDAR_DEVICE_ADDRESS);

	upperByteDataPointer = new uint8_t;
	lowerByteDataPointer = new uint8_t;

	charPointerConvertedData = new char;

	i2CMultiplexer = new frc::I2C(MXP_PORT, I2C_MULTIPLEXER_DEVICE_ADDRESS);

	convertedValueToSendToI2CMultiplexer = new uint8_t;
}

void Lidars::OpenLidarChannelOnMultplexer() {
	convertedValueToSendToI2CMultiplexer = this->ConvertUint8_t_To_Uint8_tPointer(VALUE_TO_OPEN_LIDAR_CHANNEL_6_SHOOTER);
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

int Lidars::GetLidarValue(int lowerByte, int upperByte) {
	shiftedUpperByte = upperByte << 8;
	finalValue = (shiftedUpperByte + lowerByte);
	if (finalValue != 0.0) {
		finalNonZeroLidarValue = finalValue;
	}
	return finalNonZeroLidarValue;
}

void Lidars::ResetLidarWholeProcessVariables() {
	configuredLidar = false;
	waitConfigureLidar = false;
	receivedUpperByte = false;
	lidarUpperByte = 0;
	lidarLowerByte = 0;
	lidarValue_CM = 0;
	lidarValue_IN = 0.0;
	lidarValue_M = 0.0;
	returnLidarValue = 0.0;
}

double Lidars::GetLidarValueWholeProcess(int distanceUnit) {
	if (configuredLidar == false) {
		this->ConfigureLidar();
		configuredLidar = true;
	}
	else if (waitConfigureLidar == false && configuredLidar) {
		waitConfigureLidar = true;
	}
	else if (receivedUpperByte == false && waitConfigureLidar) {
		lidarUpperByte = this->GetUpperByte();
		receivedUpperByte = true;
	}
	else if (receivedUpperByte && configuredLidar) {
		lidarLowerByte = this->GetLowerByte();
		lidarValue_CM = this->GetLidarValue(lidarLowerByte, lidarUpperByte);
		lidarValue_IN = this->ConvertCentimetersToInches(lidarValue_CM);
		lidarValue_M = this->ConvertCentimetersToMeters(lidarValue_CM);
		configuredLidar = false;
		waitConfigureLidar = false;
		receivedUpperByte = false;
	}

	if (distanceUnit == DISTANCE_UNIT_ARRAY[CENTIMETERS]) {
		returnLidarValue = ((double)lidarValue_CM);
	}
	else if (distanceUnit == DISTANCE_UNIT_ARRAY[INCHES]) {
		returnLidarValue = lidarValue_IN;
	}
	else if (distanceUnit == DISTANCE_UNIT_ARRAY[METERS]) {
		returnLidarValue = lidarValue_M;
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
