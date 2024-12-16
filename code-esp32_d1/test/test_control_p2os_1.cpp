#include <ps5Controller.h>
#include <Arduino.h>
#include <p2os.hpp>
#include <HardwareSerial.h>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0);    // define a Serial for UART0
HardwareSerial pioneer_serial(2);  // define a Serial for UART2

P2OS*    p2os;
uint32_t last_time_motor_state = 0;
uint32_t current_time;
uint32_t current_loop_time;

int scale_test(int16_t value, int16_t old_min, int16_t old_max, int16_t new_min, int16_t new_max) {
    return int(new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min));
}

void setup() {
    debug_serial.begin(9600);
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    debug_serial.flush();
    pioneer_serial.flush();

    p2os = new P2OS(debug_serial, pioneer_serial);

    ps5.begin("10:18:49:7D:EC:F1");  // replace with MAC address of your controller

    debug_serial.println("Ready!");
}

void loop() {
    bool    is_connected = 0;
    uint8_t current_r2_val = 0;
    uint8_t current_l2_val = 0;
    int16_t current_rs_x_val = 0;

    geometry_msgs::Twist  msg_vel;
    p2os_msgs::MotorState msg_motor_state;

    
    while (ps5.isConnected() == true) {
        current_loop_time = millis();

        if (ps5.Up()) {
            is_connected = !(p2os->setup());
        }

        if (ps5.Down()) {
            is_connected = p2os->shutdown();
        }

        if (is_connected) {
            current_time = millis();
            p2os->loop();

            current_r2_val = ps5.R2Value();  // value 0 - 255
            current_l2_val = ps5.L2Value();  // value 0 - 255
            current_rs_x_val = (-1) * scale_test(ps5.RStickX(), -128, 128, -170, 170);

            msg_vel.linear.x = double(double(current_r2_val - current_l2_val) / 100);
            msg_vel.angular.z = double(current_rs_x_val) / 100;

            p2os->set_vel(&msg_vel);

            if (current_time - last_time_motor_state > 100) {
                msg_motor_state.state = 1;
                p2os->set_motor_state(&msg_motor_state);
                last_time_motor_state = current_time;
            }
        }

        debug_serial.printf("current_loop_time: %ld \n",  (millis() - current_loop_time));
    }
}
