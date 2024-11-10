#include <ps5Controller.h>
#include <Arduino.h>
#include <p2os_comm.hpp>
#include <HardwareSerial.h>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0); // define a Serial for UART0
HardwareSerial pioneer_serial(2); // define a Serial for UART2

P2OSCommunication* p2os_communication;
uint32_t last_time_pulse = 0;
uint32_t last_time_vel = 0;
uint32_t current_time;

int scale(int16_t value, int16_t old_min, int16_t old_max, int16_t new_min, int16_t new_max) {
    return int(new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min));
}

void setup() {
  debug_serial.begin(9600);
  pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
  debug_serial.flush();
  pioneer_serial.flush();

  p2os_communication = new P2OSCommunication(
    debug_serial,
    pioneer_serial
  );

  ps5.begin("10:18:49:7D:EC:F1"); //replace with MAC address of your controller

  debug_serial.println("Ready!");
}

void loop() {
  bool is_connected = 0;
  uint8_t current_r2_val = 0;
  uint8_t current_l2_val = 0;
  int16_t current_rs_x_val = 0;

  while (ps5.isConnected() == true) {

    if (ps5.Up()) {
      is_connected = !(p2os_communication->Setup());
    }

    if (ps5.Down()) {
      is_connected = p2os_communication->Shutdown();
    }

    if(is_connected) {
      current_time = millis();
      
      if ((current_time - last_time_pulse) > 500) {
        Serial.println("Sending pulse");
        p2os_communication->SendPulse();
        last_time_pulse = current_time;
      }

      if ((current_time - last_time_vel) > 50) {
        p2os_communication->check_and_set_motor_state();

        if (ps5.R2()) {
          current_r2_val = ps5.R2Value(); // value 0 - 255
          if (abs(current_r2_val) < 20) {
            current_r2_val = 0;
          }
        } else {
          current_r2_val = 0;
        }

        if (ps5.L2()) {
          current_l2_val = ps5.L2Value(); // value 0 - 255
          if (abs(current_l2_val) < 20) {
            current_l2_val = 0;
          }
        } else {
          current_l2_val = 0;
        }
        
        current_rs_x_val = scale(ps5.RStickX(), -128, 128, -100, 100);
        current_rs_x_val = (-1)*current_rs_x_val; 
        
        if (abs(current_rs_x_val) < 10) {
          current_rs_x_val = 0;
        }

        p2os_communication->check_and_set_vel(
          current_r2_val - current_l2_val, 
          current_rs_x_val
        );

        last_time_vel = current_time;
      }

    }
  }
}
