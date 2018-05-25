#include "Robot.h"

// global variable to monitor the wifi connection status between robot and server
volatile uint8_t is_connected;

/*************************************************************************************************************************
* Description: init_robot() function initializes gpio pins, spi bus, motors and wifi
* Pre:  ssid and password must match with server network ssid and password
* Post: gpio's are initialized 
*       spi is configured to it's default settings and is ready for data transmission
*       motors pid is set to default
*       wifi connection is established between robot and server       
**************************************************************************************************************************/
void Robot::init_robot(const char* ssid, const char* password)
{
	sonic_board_controller.init();
	ball_controller.init();
	network_controller.connect_to_wifi(ssid, password);
}

/*************************************************************************************************************************
* Description: update_robot() function receives new data from the server and updates the robot using received data
* Pre:  none
* Post: robot state is updated based on received data
**************************************************************************************************************************/
void Robot::update_robot()
{
	if (is_connected) {
		uint32_t received_data_length = 0;
		bool is_protobuf_packet = false;

		received_data_length = network_controller.receive_udp_packet(udp_buffer);
		
		if (received_data_length) {

			is_protobuf_packet = protobuf_parser.parse_udp_packet(udp_buffer, robot_command, received_data_length);

			if (is_protobuf_packet) {
				sonic_board_controller.update_motors(robot_command.move.x, robot_command.move.y, robot_command.move.r);
				ball_controller.write_data_to_ball_controller(robot_command.action.kick, robot_command.action.chip, 
					                                          robot_command.action.dribble);
			}
		}
	}
	// if the wifi connection with the server is lost halt the robot
	else {
		halt_robot();
	}
}

/*************************************************************************************************************************
* Descrition: halt_robot() function stops the robot
* Pre: none
* Post: the robot is in halt state
**************************************************************************************************************************/
void Robot::halt_robot()
{
	sonic_board_controller.update_motors(0.0f, 0.0f, 0.0f);
	ball_controller.write_data_to_ball_controller(0.0f, 0.0f, 0.0f);
}
