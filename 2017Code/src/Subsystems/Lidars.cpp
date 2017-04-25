#include "Lidars.h"
#include "../RobotMap.h"
#include "Commands/ReadLidarValues.h"

Lidars::Lidars() : Subsystem("Lidar") {

}

void Lidars::InitDefaultCommand() {
	// Set the default command for a subsystem here.
	// SetDefaultCommand(new MySpecialCommand());
	SetDefaultCommand(new ReadLidarValues());
}

void Lidars::InitializeLidarsAndI2CMultiplexer() {
	lidar = new frc::I2C(MXP_PORT, LIDAR_DEVICE_ADDRESS);

	upperByteDataPointer = new uint8_t;
	lowerByteDataPointer = new uint8_t;

	charPointerConvertedData = new char;

	i2CMultiplexer = new frc::I2C(MXP_PORT, I2C_MULTIPLEXER_DEVICE_ADDRESS);

	convertedValueToSendToI2CMultiplexer = new uint8_t;
}

void Lidars::InitializeLidarPowerEnablePin() {
	lidarPowerEnabledDO = new frc::DigitalOutput(LIDAR_POWER_ENABLE_DIGITAL_OUTPUT_PORT);
}

void Lidars::TurnLidarOnOff(bool turnOn) {
	lidarPowerEnabledDO->Set(turnOn);
	lidarTurnedOn = turnOn;
}

void Lidars::OpenLidarChannelOnMultiplexer(int byteToSend) {
	convertedValueToSendToI2CMultiplexer = this->ConvertUint8_t_To_Uint8_tPointer(byteToSend);
	i2CMultiplexer->WriteBulk(convertedValueToSendToI2CMultiplexer, 1);
}

void Lidars::ConfigureLidar() {
	lidar->Write(CONFIGURE_REGISTER_ADDRESS, CONFIGURE_VALUE_TO_WRITE);
}

void Lidars::TurnOffDetectorBiasBetweenLidarAcquisitions() {
	lidar->Write(POWER_CONSUMPTION_MODE_AFTER_READING_VALUES_ADDRESS, VALUE_TO_SEND_TO_TURN_OFF_DETECTOR_BIAS_BETWEEN_ACQUISITIONS);
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

	frc::SmartDashboard::PutNumber("Actual LIDAR Value:", finalValue);

	if (distanceUnits == DISTANCE_UNIT_ARRAY[CENTIMETERS]) {
		returnLidarValue = ((double)finalValue);
	}
	else if (distanceUnits == DISTANCE_UNIT_ARRAY[INCHES]) {
		returnLidarValue = this->ConvertCentimetersToInches(finalValue);
		if (returnLidarValue > 200.0) {
			returnLidarValue = 0.0;
		}
	}
	else if (distanceUnits == DISTANCE_UNIT_ARRAY[METERS]) {
		returnLidarValue = this->ConvertCentimetersToMeters(finalValue);
	}

	if (returnLidarValue != 0) {
		savedLidarValue = returnLidarValue;
	}

	return savedLidarValue;
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

void Lidars::StoreLidarValueForHopperAndShoot(double lidarValue) {
	storedLidarValue = lidarValue;
}

double Lidars::GetDistanceToTravelToHopper(double frontUltrasonicSensorValue) {
	distanceFromBumperToGuardrail = (frontUltrasonicSensorValue - DISTANCE_FROM_FRONT_ULTRASONIC_SENSOR_TO_BUMPER);
	desiredLidarValue = (START_OF_DESIRED_LIDAR_VALUE + distanceFromBumperToGuardrail);
	std::cout << "Desired Lidar Value: " << desiredLidarValue << std::endl;
	desiredDistanceToTravelToHopper = (desiredLidarValue - storedLidarValue);
	std::cout << "Stored Lidar Value: " << storedLidarValue << std::endl;
	std::cout << "Desired Distance To Travel: " << desiredDistanceToTravelToHopper << std::endl;
	return desiredDistanceToTravelToHopper;
}

// Put methods for controlling this subsystem
// here. Call these from Commands.
