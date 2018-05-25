#pragma once
#include <SPI.h>
#include <stdint.h>

/***********************************************************************************************/
// SONIC_BOARD_CONST namespace contains a set of constants for the sonic boards
namespace SONIC_BOARD_CONST {

/************************* sonic boards module id ****************************/
	const uint8_t SONIC_MODULE_LEFT = 0x00;
	const uint8_t SONIC_MODULE_RIGHT = 0x01;
/*****************************************************************************/
	
/***********************sonic boards motor id ********************************/	
    // note that MOTOR_ID_FR != MOTOR_ID_FL because the sonic boards are
	// inverted by 180 degrees on the motherboard
	const uint8_t MOTOR_ID_FR = 0x01;        // front right motor id
	const uint8_t MOTOR_ID_FL = 0x00;        // front left motor id
	const uint8_t MOTOR_ID_BL = 0x01;        // back left motor id
	const uint8_t MOTOR_ID_BR = 0x00;        // back right motor id
/****************************************************************************/

	const float MAX_MOTOR_RPM = 2500.0f;
	const float MIN_MOTOR_RPM = -2500.0f;

/************************ sonic board commands list *************************/
	const uint8_t DUMMY = 0x00;                            
	const uint8_t SET_MOTOR_RPM = 0x01;                   
	const uint8_t GET_MOTOR_RPM = 0x02;                    
	const uint8_t SET_MOTOR_ENABLE = 0x03;                
	const uint8_t GET_MOTOR_ENABLE = 0x04;                 
	const uint8_t SET_MOTOR_BRAKE = 0x05;                  
	const uint8_t GET_MOTOR_BRAKE = 0x06;                  
	const uint8_t SET_MOTOR_DAC = 0x07;                    
	const uint8_t GET_MOTOR_DAC = 0x08;                    
	const uint8_t SET_MOTOR_KP = 0x09;                     
	const uint8_t GET_MOTOR_KP = 0x0A;                     
	const uint8_t SET_MOTOR_KI = 0x0B;                     
	const uint8_t GET_MOTOR_KI = 0x0C;                     
	const uint8_t SET_MOTOR_KD = 0x0D;                     
	const uint8_t GET_MOTOR_KD = 0x0E;                    
	const uint8_t GET_MOTOR_ENCODER_COUNTER = 0x0F;       
	const uint8_t GET_MOTOR_CONTROLLER_TEMPERATURE = 0x10;
/*****************************************************************************/

};
/***********************************************************************************************/


/***********************************************************************************************/
namespace SPI_SETTINGS {
	const uint8_t  SS_SONIC_BOARD_LEFT  = 2;          // gpio 2
	const uint8_t  SS_SONIC_BOARD_RIGHT = 15;         // gpio 15
	const uint32_t SPI_FREQUENCY        = 100000;     // 100 kHz
};
/***********************************************************************************************/


/***********************************************************************************************
* SonicBoardController class is responsible for sonic board modules management
* This class implements following futures:
* 1. Initializes spi communication between ESP32 and sonic boards
* 2. Calculates motors rpm
* 3. Transmits data from ESP32 to sonic boards and vice versa 
************************************************************************************************/
class SonicBoardController
{
public:
	void init();
	void update_motors(float, float, float);
	float transmit_receive_float(uint8_t, uint8_t, uint8_t, float);
	void set_motors_kp(float, float, float, float);
	void set_motors_ki(float, float, float, float);
	void set_motors_kd(float, float, float, float);

private:
	void set_motors_rpm(float, float, float, float);
	void select_module(uint8_t);
	void deselect_module(uint8_t);
};

