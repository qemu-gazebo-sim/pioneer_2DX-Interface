#include <Arduino.h>
#include <ps5Controller.h>
#include <p2os.hpp>
#include <HardwareSerial.h>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0); // define a Serial for UART0
HardwareSerial pioneer_serial(2); // define a Serial for UART2

P2OSCommunication* p2os_communication;

void setup() {
  debug_serial.begin(9600);
  
  pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
  debug_serial.println("Leets go!");
  
  p2os_communication = new P2OSCommunication(
    debug_serial,
    pioneer_serial
  );

  p2os_communication->Setup();
}

void loop() {
  // debug_serial.println("Loop!");
}


