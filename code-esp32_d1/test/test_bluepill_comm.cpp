#include <Arduino.h>
#include <HardwareSerial.h>
#include "bluepill_comm.hpp"

HardwareSerial             debug_serial(0);
BluepillCommunication*     bluepill_comm;
nav_msgs::ros_p2os_data_t* mock_p2os_data;
uint32_t                   print_time;

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();

    bluepill_comm = new BluepillCommunication(debug_serial);
    mock_p2os_data = new nav_msgs::ros_p2os_data_t();

    debug_serial.println("Ready!");
    print_time = millis();
}

void loop() {
    bluepill_comm->loop();

    if ((millis() - print_time) > 500) {
        geometry_msgs::Twist current_vel = bluepill_comm->get_velocity();
        double               linear_vel = current_vel.linear.x;
        double               angular_vel = current_vel.angular.z;
        ConnectionStates     connection = bluepill_comm->is_bluepill_connected();

        debug_serial.printf("Lin vel: %lf | Ang vel: %lf | Connection: %d\n", linear_vel, angular_vel, connection);

        print_time = millis();
    }

    mock_p2os_data->position.pose.pose.position.x = 10;
    mock_p2os_data->position.pose.pose.position.y = 5;
    mock_p2os_data->sonar.ranges_count = 8;
    mock_p2os_data->sonar.ranges = {0, 1, 2, 3, 4, 5, 6, 7};
    bluepill_comm->update_p2dx_data(*mock_p2os_data);
}
