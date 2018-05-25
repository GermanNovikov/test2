#pragma once
#include "SonicBoardController.h"
#include "NetworkController.h"
#include "ProtobufParser.h"
#include "BallController.h"
#include <stdint.h>

// to get udp buffer size
using namespace udp_settings;

/***********************************************************************
* Robot class is the main class in the project. It is responsible for
* robot initialization and robot state update
* This class implements following futures:
* 1. Manages all the robot peripherals (sonic boards, ball conroller...)
* 2. Initializes the robot
* 3. Updates the robot state
* 3. Halts the robot if connection with the server is lost
************************************************************************/
class Robot
{
public:
	void init_robot(const char*, const char*);
	void update_robot();

private:
	SonicBoardController sonic_board_controller;
	NetworkController network_controller;
	ProtobufParser protobuf_parser;
	BallController ball_controller;

	// data structure that stores decoded protobuf data
	Command robot_command = Command_init_zero;

	// buffer that stores data received from server 
	uint8_t udp_buffer[udp_settings::UDP_BUFFER_SIZE];

	void halt_robot();
};


