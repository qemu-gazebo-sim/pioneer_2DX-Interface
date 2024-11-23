// #define USE_PS5_CONTROL

#ifdef USE_PS5_CONTROL
    #include <ps5Controller.h>
    #include "utils.hpp"
    #define PS5_CONTROLLER_MAC_ADDRESS "10:18:49:7D:EC:F1"  // replace with MAC address of your controller
#else
    #include "bluepill_comm.hpp"
#endif

#include <Arduino.h>
#include <HardwareSerial.h>
#include <p2os.hpp>
#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0);    // define a Serial for UART0
HardwareSerial pioneer_serial(2);  // define a Serial for UART2

P2OS*    p2os;
uint32_t last_time_motor_state = 0;

#ifndef USE_PS5_CONTROL
BluepillCommunication* bluepill_comm;
#endif

void setup() {
    debug_serial.begin(9600);
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    debug_serial.flush();
    pioneer_serial.flush();

#ifdef USE_PS5_CONTROL
    ps5.begin(PS5_CONTROLLER_MAC_ADDRESS);
#else
    bluepill_comm = new BluepillCommunication(debug_serial);
#endif

    p2os = new P2OS(debug_serial, pioneer_serial);
    debug_serial.println("Ready!");
}

void loop() {
#ifdef USE_PS5_CONTROL
    bool     is_connected = 0;
    uint16_t current_r2_val = 0;
    uint16_t current_l2_val = 0;
    int16_t  current_rs_x_val = 0;

    geometry_msgs::Twist  msg_vel;
    p2os_msgs::MotorState msg_motor_state;

    while (ps5.isConnected() == true) {
        if (ps5.Up() && (is_connected < 1)) {
            is_connected = !(p2os->setup());
        }

        if (ps5.Down() && (is_connected > 0)) {
            is_connected = p2os->shutdown();
        }

        if (is_connected) {
            p2os->loop();

            current_r2_val = scale(ps5.R2Value(), 0, 255, 0, 400);  // max it is 500, but we are using 400
            current_l2_val = scale(ps5.L2Value(), 0, 255, 0, 400);  // max it is 500, but we are using 400
            current_rs_x_val =
                (-1) * scale(ps5.RStickX(), -128, 128, -170, 170);  // max it is 180, but we are using 170

            msg_vel.linear.x = double(double(current_r2_val - current_l2_val) / 1000);
            msg_vel.angular.z = double(double(current_rs_x_val) / 100);

            p2os->set_vel(&msg_vel);

            if (millis() - last_time_motor_state > 100) {
                msg_motor_state.state = 1;
                p2os->set_motor_state(&msg_motor_state);
                last_time_motor_state = millis();
            }
        }
    }
#else
    geometry_msgs::Twist  current_vel;
    p2os_msgs::MotorState msg_motor_state;
    ConnectionStates      current_connected_state = NOT_CONNECTED;

    while (true) {
        bluepill_comm->loop();

        if (current_connected_state != bluepill_comm->is_bluepill_connected()) {
            switch (bluepill_comm->is_bluepill_connected()) {
                case CONNECTED:
                    current_connected_state = !(p2os->setup()) ? CONNECTED : NOT_CONNECTED;
                    break;
                case NOT_CONNECTED:
                    current_connected_state = p2os->shutdown() ? NOT_CONNECTED : CONNECTED;
                    break;
                default:
                    debug_serial.println("Error: Cry! - switch (bluepill_comm->is_bluepill_connected())");
                    break;
            }
        }

        if (current_connected_state == CONNECTED) {
            p2os->loop();
            current_vel = bluepill_comm->get_velocity();
            p2os->set_vel(&current_vel);
            if (millis() - last_time_motor_state > 100) {
                msg_motor_state.state = 1;
                p2os->set_motor_state(&msg_motor_state);
                last_time_motor_state = millis();
            }
        }
    }
#endif
}
