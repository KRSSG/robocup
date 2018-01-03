#include <iostream>
#include "serial.h"

#include "ros/ros.h"
#include "std_msgs/"

const int NUM_BYTES = 6;
const string port("/dev/ttyUSB0");
unsigned static char buf[NUM_BYTES];

HAL::Serial serial;

using namespace std;
int main(int argc, char **argv) {

	if(!serial.Open(port.c_str(), baud_rate)) {
		cout << "Couldn't open the fucking port: " << port.c_str() << "\n";
	}

	ros::init(argc, argv, "bot_data");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<>("/bot_IR_data", 10000);

	while (true) {
		serial.Read(buf, NUM_BYTES);
	}
}
