#ifndef _BLUEPILL_COMM_HPP_
#define _BLUEPILL_COMM_HPP_

#include "p2os_msgs.hpp"
#include <Arduino.h>
#include <HardwareSerial.h>
#include "bluepill_config.hpp"

enum ConnectionStates {
    NOT_CONNECTED,
    CONNECTED
};

class BluepillCommunication {
    // variables

private:
    HardwareSerial* debug_serial;

    /* Connection params */
    uint32_t         connection_pin_time;
    uint32_t         connection_pin_sample_sum;
    uint32_t         connection_pin_counter;
    ConnectionStates connection_pin_state;

    /* Velocity params */
    uint32_t             vel_time;
    uint32_t             motor_sample_sum[NUM_MOTOR_COMMS];
    uint32_t             vel_sample_counter;
    bool                 motor_gpio_state[NUM_MOTOR_COMMS];
    geometry_msgs::Twist current_velocity;

    /* DDS values */
    p2os_msgs::SonarArray current_ultrasonic_data;
    uint8_t               dds_gpio_state[NUM_DDS];

    /* Encoders values */
    nav_msgs::Odometry current_position;
    uint8_t            encoders_gpio_state[NUM_ENCODERS_PIN];

    // functions

public:
    BluepillCommunication(HardwareSerial& debug_serial);

    ~BluepillCommunication();

    bool is_bluepill_connected();

    void loop();

    geometry_msgs::Twist get_velocity();

    void update_p2dx_data(nav_msgs::ros_p2os_data_t data);

protected:
    void update_encoder_data(nav_msgs::Odometry position);

    void update_dds_data(p2os_msgs::SonarArray dds_data);

    int16_t scale(int16_t value, int16_t old_min, int16_t old_max, int16_t new_min, int16_t new_max);

    double getYaw(nav_msgs::Odometry position);
};

#endif  // _BLUEPILL_COMM_HPP_
