#include "SonicBoardController.h"
#include <SPI.h>
#include <Arduino.h>
#include <algorithm>

/****************************************************************************************************************
* Descrition: init() function initializes spi bus, sets the motors pid to default
* Pre: none
* Post: spi is configured to it's default [CPOL = 0, CPHA = 0] settings and is ready for data transmission   
        motors pid is set to default
*****************************************************************************************************************/
void SonicBoardController::init()
{
	Serial.println();
	Serial.println("initializing sonic board controller");

	/**********configure the spi pins and settings*************/
	pinMode(SPI_SETTINGS::SS_SONIC_BOARD_LEFT, OUTPUT);
	pinMode(SPI_SETTINGS::SS_SONIC_BOARD_RIGHT, OUTPUT);
	digitalWrite(SPI_SETTINGS::SS_SONIC_BOARD_LEFT, HIGH);
	digitalWrite(SPI_SETTINGS::SS_SONIC_BOARD_RIGHT, HIGH);
	SPI.begin();
	SPI.setFrequency(SPI_SETTINGS::SPI_FREQUENCY);
	/************************************************************/

	/***********set motors pid to default ***********************/
	Serial.println("setting motors pid to default:");
	set_motors_kp(1.5f, 1.5f, 1.5f, 1.5f);
	set_motors_ki(0, 0, 0, 0);
	set_motors_kd(0, 0, 0, 0);
	Serial.println("done setting motors pid to default");
	Serial.println("**************************************");
	/************************************************************/

	delay(2000);
}

/****************************************************************************************************************
* Descrition: update_motors() function calculates and sends new rmp value to the motors
* Pre:
* Post: new rmp value is transmited to the motors
*****************************************************************************************************************/
void SonicBoardController::update_motors(float forward_speed, float left_speed, float rotation_speed)
{
	/***********************************************************************************************
	* Calculate motor speeds
	* (40*PI)/180 is 40 degrees (angle of motors) in radians
	* 0.16375 is motor circumference at wheelbase in meters
	* 0.05 is circumference of wheel in meters
	************************************************************************************************/

	/* Front left motor */
	float wheel_speed_fl = (((forward_speed / cos((40 * PI) / 180))    // Forward speed part in m/s
		- (left_speed / sin((40 * PI) / 180))                          // Left speed part in m/s
		- ((rotation_speed*0.16375) / 2 * PI))                         // Rotation part in m/s
		/ 0.05) * 60;                                                  // From m/s to RPM

	/* Front right motor */
	float wheel_speed_fr = -(((forward_speed / cos((40 * PI) / 180))   // Forward speed part in m/s
		+ (left_speed / sin((40 * PI) / 180))                          // Left speed part in m/s
		+ ((rotation_speed*0.16375) / 2 * PI))                         // Rotation part in m/s
		/ 0.05) * 60;                                                  // From m/s to RPM

    /* Back left motor */
	float wheel_speed_bl = (((forward_speed / cos((40 * PI) / 180))    // Forward speed part in m/s
		+ (left_speed / sin((40 * PI) / 180))                          // Left speed part in m/s
		- ((rotation_speed*0.16375) / 2 * PI))                         // Rotation part in m/s
		/ 0.05) * 60;                                                  // From m/s to RPM

    /* Back right motor */
	float wheel_speed_br = -(((forward_speed / cos((40 * PI) / 180))   // Forward speed part in m/s
		- (left_speed / sin((40 * PI) / 180))                          // Left speed part in m/s
		+ ((rotation_speed*0.16375) / 2 * PI))                         // Rotation part in m/s
		/ 0.05) * 60;                                                  // From m/s to RPM

	// transmit calculated wheel speed to the sonic boards
	set_motors_rpm(wheel_speed_fl, wheel_speed_fr, wheel_speed_bl, wheel_speed_br);
}

/****************************************************************************************************************
* Descrition: transmit_receive_float() function is used to 
* transmit and receive commands and float data to and from the sonic boards
* Pre:  module id is in range from 0 to 1
*       motor id is in range from 0 to 1
*       command exists (refer to sonic board commands list)
* Post: command and float data is transmited to the specified module
*       float data is received from the specified module
*****************************************************************************************************************/
float SonicBoardController::transmit_receive_float(uint8_t module_id, uint8_t motor_id, 
	                                               uint8_t command, float data)
{
	uint8_t receivedData[4] = { 0 };

	// this union is used to send and receive float data
	union {
		float value;
		uint8_t bytes[4];
	} float_data;

	float_data.value = data;

	select_module(module_id);
	delayMicroseconds(50);
	// sending command 
	SPI.transfer(command);
	// sending command parameter
	SPI.transfer(motor_id);

	deselect_module(module_id);
	delayMicroseconds(50);
	select_module(module_id);

	// sending float data
	receivedData[0] = SPI.transfer(float_data.bytes[0]);
	receivedData[1] = SPI.transfer(float_data.bytes[1]);
	receivedData[2] = SPI.transfer(float_data.bytes[2]);
	receivedData[3] = SPI.transfer(float_data.bytes[3]);

	deselect_module(module_id);
	delayMicroseconds(50);

	// copy received data to the union in order to convert it to the float value
	std::copy(receivedData, receivedData + 4, float_data.bytes);

	// return received data as float
	return float_data.value;
}

/****************************************************************************************************************
* Descrition: set_motors_rpm() function sends rpm values to the motors
* Pre:
* Post: rmp data is transmited to the motors
*****************************************************************************************************************/
void SonicBoardController::set_motors_rpm(float wheel_speed_fl, float wheel_speed_fr, 
	                                      float wheel_speed_bl, float wheel_speed_br)
{
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_FR,
		SONIC_BOARD_CONST::SET_MOTOR_RPM,
		constrain(wheel_speed_fr, SONIC_BOARD_CONST::MIN_MOTOR_RPM, SONIC_BOARD_CONST::MAX_MOTOR_RPM) );

	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_FL,
		SONIC_BOARD_CONST::SET_MOTOR_RPM,
		constrain(wheel_speed_fl, SONIC_BOARD_CONST::MIN_MOTOR_RPM, SONIC_BOARD_CONST::MAX_MOTOR_RPM) );

	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_BL,
		SONIC_BOARD_CONST::SET_MOTOR_RPM,
		constrain(wheel_speed_bl, SONIC_BOARD_CONST::MIN_MOTOR_RPM, SONIC_BOARD_CONST::MAX_MOTOR_RPM) );

	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_BR,
		SONIC_BOARD_CONST::SET_MOTOR_RPM,
		constrain(wheel_speed_br, SONIC_BOARD_CONST::MIN_MOTOR_RPM, SONIC_BOARD_CONST::MAX_MOTOR_RPM) );
}

/****************************************************************************************************************
* Descrition: set_motors_kp() function sends kp  values to the motors
* Pre:
* Post: kp data is transmited to the motors
*****************************************************************************************************************/
void SonicBoardController::set_motors_kp(float kp_fr, float kp_fl, float kp_bl, float kp_br)
{
	Serial.print(" setting front right motor kp to: ");
	Serial.println(kp_fr);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_FR,
		SONIC_BOARD_CONST::SET_MOTOR_KP, kp_fr);

	Serial.print(" setting front left  motor kp to: ");
	Serial.println(kp_fl);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_FL,
		SONIC_BOARD_CONST::SET_MOTOR_KP, kp_fl);

	Serial.print(" setting back left  motor kp to: ");
	Serial.println(kp_bl);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_BL,
		SONIC_BOARD_CONST::SET_MOTOR_KP, kp_bl );

	Serial.print(" setting back right motor kp to: ");
	Serial.println(kp_br);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_BR,
		SONIC_BOARD_CONST::SET_MOTOR_KP, kp_br );
}

/****************************************************************************************************************
* Descrition: set_motors_ki() function sends ki  values to the motors
* Pre:
* Post: ki data is transmited to the motors
*****************************************************************************************************************/
void SonicBoardController::set_motors_ki(float ki_fr, float ki_fl, float ki_bl, float ki_br)
{
	Serial.print(" setting front right motor ki to: ");
	Serial.println(ki_fr);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_FR,
		SONIC_BOARD_CONST::SET_MOTOR_KI, ki_fr );

	Serial.print(" setting front left  motor ki to: ");
	Serial.println(ki_fl);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_FL,
		SONIC_BOARD_CONST::SET_MOTOR_KI, ki_fl );

	Serial.print(" setting back left  motor ki to: ");
	Serial.println(ki_bl);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_BL,
		SONIC_BOARD_CONST::SET_MOTOR_KI, ki_bl );

	Serial.print(" setting back right  motor ki to: ");
	Serial.println(ki_br);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_BR,
		SONIC_BOARD_CONST::SET_MOTOR_KI, ki_br );
}

/****************************************************************************************************************
* Descrition: set_motors_kd() function sends kd  values to the motors
* Pre:
* Post: kd data is transmited to the motors
*****************************************************************************************************************/
void SonicBoardController::set_motors_kd(float kd_fr, float kd_fl, float kd_bl, float kd_br)
{
	Serial.print(" setting front right motor kd to: ");
	Serial.println(kd_fr);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_FR,
		SONIC_BOARD_CONST::SET_MOTOR_KD, kd_fr );

	Serial.print(" setting front left  motor kd to: ");
	Serial.println(kd_fl);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_FL,
		SONIC_BOARD_CONST::SET_MOTOR_KD, kd_fl );

	Serial.print(" setting back left  motor kd to: ");
	Serial.println(kd_bl);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_LEFT,
		SONIC_BOARD_CONST::MOTOR_ID_BL,
		SONIC_BOARD_CONST::SET_MOTOR_KD, kd_bl );

	Serial.print(" setting back right  motor kd to: ");
	Serial.println(kd_br);
	transmit_receive_float(
		SONIC_BOARD_CONST::SONIC_MODULE_RIGHT,
		SONIC_BOARD_CONST::MOTOR_ID_BR,
		SONIC_BOARD_CONST::SET_MOTOR_KD, kd_br );
}

/****************************************************************************************************************
* Descrition: select_module() function enables specified spi module 
* Pre :  module id is in range from 0 to 1
* Post : slave select pin for the specified module is in low state (active)
*****************************************************************************************************************/
void SonicBoardController::select_module(uint8_t module_id)
{
	if (module_id == SONIC_BOARD_CONST::SONIC_MODULE_LEFT) {
		digitalWrite(SPI_SETTINGS::SS_SONIC_BOARD_LEFT, LOW);
	}
	else if (module_id == SONIC_BOARD_CONST::SONIC_MODULE_RIGHT) {
		digitalWrite(SPI_SETTINGS::SS_SONIC_BOARD_RIGHT, LOW);
	}
	else 
	{
		Serial.print("ERROR: module with id = ");
		Serial.print(module_id);
		Serial.println(" not found");
	}
}

/****************************************************************************************************************
* Descrition: select_module() function disables specified spi module
* Pre :  module id is in range from 0 to 1
* Post : slave select pin for the specified module is in high state (disabled)
*****************************************************************************************************************/
void SonicBoardController::deselect_module(uint8_t module_id)
{
	if (module_id == SONIC_BOARD_CONST::SONIC_MODULE_LEFT) {
		digitalWrite(SPI_SETTINGS::SS_SONIC_BOARD_LEFT, HIGH);
	}
	else if (module_id == SONIC_BOARD_CONST::SONIC_MODULE_RIGHT) {
		digitalWrite(SPI_SETTINGS::SS_SONIC_BOARD_RIGHT, HIGH);
	}
	else
	{
		Serial.print("ERROR: module with id = ");
		Serial.print(module_id);
		Serial.println(" not found");
	}
}
