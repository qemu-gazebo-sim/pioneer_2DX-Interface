#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <Arduino.h>

#include <packet.hpp>

P2OSPacket::P2OSPacket(
  HardwareSerial& debug_serial, 
  HardwareSerial& pioneer_serial
) {
  this->debug_serial = &debug_serial;
  this->pioneer_serial = &pioneer_serial;  
}

P2OSPacket::~P2OSPacket() { /** Destructor **/}

void P2OSPacket::set_pioneer_serial(HardwareSerial& pioneer_serial) {
  this->pioneer_serial = &pioneer_serial;  
}

void P2OSPacket::Print() {
  if (this->packet) {
    for (int i = 0; i < this->size; i++) {
      this->debug_serial->printf("%u ", packet[i]);
    }
    this->debug_serial->printf("\n");
  }
}

void P2OSPacket::PrintHex() {
  if (this->packet) {
    for (int i = 0; i < this->size; i++) {
     this->debug_serial->printf("0x%.2x ", packet[i]);
    }
   this->debug_serial->printf("\n");
  }
}

bool P2OSPacket::Check() {
  const int16_t chksum = CalcChkSum();
  return (
    chksum == (
      this->packet[this->size - 2] << 8
      )
    ) | this->packet[this->size - 1];
}

int P2OSPacket::CalcChkSum() {
  unsigned char * buffer = &(this->packet[3]);
  int c = 0;
  int n;

  for (n = this->size - 5; n > 1; ) {
    c += (*(buffer) << 8) | *(buffer + 1);
    c = c & 0xffff;
    n -= 2;
    buffer += 2;
  }

  if (n > 0) {
    c ^= static_cast<int>(*(buffer++));
  }

  return c;
}

int P2OSPacket::Receive() {
  this->debug_serial->println("Receive 0");
  unsigned char prefix[3];
  int cnt;

  ::memset(this->packet, 0, sizeof(this->packet));

  do {
    ::memset(prefix, 0, sizeof(prefix));
    this->debug_serial->println("Receive 1");

    int retries_0 = 20; 
    while (retries_0 > 0) {
      cnt = 0;
      this->debug_serial->println("Receive 2");
      int retries_1 = 20; 

      
      while (cnt != 1 && retries_1 > 0) {

        // if (this->pioneer_serial->available() > 0) {
        //   int readValue = this->pioneer_serial->read();
        //   // Read the value into the prefix array starting from the third position
        //   prefix[2] = (char)readValue;
        //   cnt++;
        // }

        int read_result = this->pioneer_serial->read(&prefix[2], 1);
        cnt += read_result;
        if (cnt < 0) {
          this->debug_serial->printf("Error: error reading packet.header from robot connection: P2OSPacket():Receive():read():\n");
          return 1;
        }

        if (prefix[2] == 0 || prefix[2] == 255) {
          cnt = 0;
        }
        // this->debug_serial->printf("%i \n", read_result);
        // retries_1--;
      }
      this->debug_serial->printf("%i \n", prefix[2]);
      this->debug_serial->println("Receive 3");
      if (prefix[0] == 0xFA && prefix[1] == 0xFB) {
        break;
      }

      // timestamp = ros::Time::now();
      // GlobalTime->GetTimeDouble(&timestamp);

      prefix[0] = prefix[1];
      prefix[1] = prefix[2];
      // skipped++;

      // retries_0--;
    }
    // // if (skipped>3) ROS_INFO("Skipped %d bytes\n", skipped);

    this->size = prefix[2] + 3;
    memcpy(this->packet, prefix, 3);

    cnt = 0;
    while (cnt != prefix[2]) {
      int read_result = this->pioneer_serial->read(
        &packet[3 + cnt], 
        prefix[2] - cnt
      );
      cnt += read_result;

      if (cnt < 0) {
        this->debug_serial->printf(
          "Error reading packet body from robot connection: P2OSPacket():Receive():read():\n");
        return 1;
      }
    }
    this->debug_serial->println("Receive 4");
  } while (!Check());

  return 0;
}

int P2OSPacket::Build(
  unsigned char * data, 
  unsigned char datasize
) {
  int16_t chksum;

  this->size = datasize + 5;

  /* header */
  this->packet[0] = 0xFA;
  this->packet[1] = 0xFB;

  if (this->size > 198) {
    this->debug_serial->printf("Error: Packet to P2OS can't be larger than 200 bytes\n");
    return 1;
  }
  this->packet[2] = datasize + 2;

  memcpy(&(this->packet[3]), data, datasize);

  chksum = CalcChkSum();
  this->packet[3 + datasize] = chksum >> 8;
  this->packet[3 + datasize + 1] = chksum & 0xFF;

  if (!Check()) {
    this->debug_serial->printf("Error: DAMN\n");
    return 1;
  }
  return 0;
}

int P2OSPacket::Send() {
  int cnt = 0;

  while (cnt != this->size) {
    int read_result = this->pioneer_serial->write(this->packet, this->size);
    cnt += read_result;
    if (cnt < 0) {
      this->debug_serial->printf("Error: Send \n");
      return 1;
    }
  }
  this->Print();
  return 0;
}
