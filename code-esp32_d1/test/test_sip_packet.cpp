#include <Arduino.h>
#include "sip.hpp"
#include <HardwareSerial.h>
#include <p2os_msgs.hpp>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0); // define a Serial for UART0

byte data[] = {
    0xfa, 0xfb, 0x21, 0x32, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 0x90, 
    0xfe, 0xfe, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x02, 0x04, 0x00, 
    0x7d, 0x07, 0x00, 0x7d, 0x05, 
    0x00, 0x59, 0x30, 0xf0, 0x11, 
    0xb4
};

SIP * sip_packet;
nav_msgs::ros_p2os_data_t p2os_data;

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();
    
    debug_serial.println("Leets go!");
    
    sip_packet = new SIP(0, debug_serial);
    
    debug_serial.println("setup done!");

    sip_packet->ParseStandard(&data[3]);
    sip_packet->FillStandard(&p2os_data);
}

void loop() {
}
