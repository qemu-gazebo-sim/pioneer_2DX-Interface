#ifndef _BLUEPILL_COMM_HPP_
#define _BLUEPILL_COMM_HPP_

#include "p2os_msgs.hpp"
#include "pinout_config.hpp"
#include <Arduino.h>
#include <HardwareSerial.h>

enum ConnectionStates {
    NOT_CONNECTED,
    CONNECTED
};

enum DDS_POSITIONS {
    DDS_1_POS = 0,
    DDS_2_POS,
    DDS_3_POS,
    DDS_4_POS,
    DDS_5_POS,
    DDS_6_POS,
    DDS_7_POS,
    DDS_8_POS,
};

struct MotorSpeed {
    int8_t left;
    int8_t right;
};

class BluepillCommunication {
    // variables

private:
    /* Connection params */
    uint32_t         connection_pin_time;
    uint32_t         connection_pin_sample_sum;
    uint32_t         connection_pin_counter;
    ConnectionStates connection_pin_state;

    /* Velocity params */
    uint64_t vel_time;
    // uint32_t             polling_time;
    uint32_t             motor_sample_sum[NUM_MOTOR_COMMS];
    uint32_t             vel_sample_counter;
    bool                 motor_gpio_state[NUM_MOTOR_COMMS];
    geometry_msgs::Twist current_velocity;
    MotorSpeed           current_motors_speed;

    /* DDS values */
    p2os_msgs::SonarArray current_ultrasonic_data;
    uint8_t               dds_gpio_state[NUM_DDS];

    /* Encoders values */
    nav_msgs::Odometry current_position;
    uint8_t            encoders_gpio_state[NUM_ENCODERS_PIN];

    // functions

public:
    BluepillCommunication();

    ~BluepillCommunication();

    ConnectionStates is_bluepill_connected();

    void send_bluepill_connection(ConnectionStates connection_state);

    void loop();

    MotorSpeed get_each_motor_speed();

    geometry_msgs::Twist get_velocity();

    void update_p2dx_data(nav_msgs::ros_p2os_data_t data);

protected:
    void update_encoder_data(nav_msgs::Odometry position);

    void update_dds_data(p2os_msgs::SonarArray dds_data);

    double getYaw(nav_msgs::Odometry position);

    int8_t to_signed(int value, int bits);
};

#endif  // _BLUEPILL_COMM_HPP_
