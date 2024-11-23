#include <Arduino.h>
#include <p2os_comm.hpp>
#include <HardwareSerial.h>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0);    // define a Serial for UART0
HardwareSerial pioneer_serial(2);  // define a Serial for UART2

P2OSCommunication* p2os_communication;
unsigned long      last_time_pulse = 0;
unsigned long      last_time_vel = 0;

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    pioneer_serial.flush();

    debug_serial.println("Leets go!");

    p2os_communication = new P2OSCommunication(debug_serial, pioneer_serial);

    while (p2os_communication->Setup()) {
        debug_serial.println("p2os setup failed...");
    }
}

void loop() {
    unsigned long current_time = millis();

    unsigned long pulse_interval = current_time - last_time_pulse;
    if (pulse_interval > 1500) {
        Serial.println("Sending pulse");
        p2os_communication->SendPulse();
        last_time_pulse = current_time;
    }

    unsigned long vel_interval = current_time - last_time_vel;
    p2os_communication->send_motor_state(1);
    if (vel_interval < 10000) {
        p2os_communication->send_vel(400, 0);
    } else if (vel_interval < 20000) {
        p2os_communication->send_vel(100, 0);
    } else if (vel_interval < 30000) {
        p2os_communication->send_vel(-400, 0);
    } else {
        last_time_vel = current_time;
    }

    delay(500);
}
