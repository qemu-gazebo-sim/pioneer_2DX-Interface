/**  P2OS for Arduino **/

#include <p2os.hpp>
// #include <termios.h>
#include <Arduino.h>

#define PIONEER_SERIAL_RX 9
#define PIONEER_SERIAL_TX 10

P2OSCommunication::P2OSCommunication(HardwareSerial& debug_serial, HardwareSerial& pioneer_serial) {
    
    this->debug_serial = &debug_serial;
    this->pioneer_serial = &pioneer_serial;   
    
    std::string def = DEFAULT_P2OS_PORT;
    std::string host = DEFAULT_P2OS_TCP_REMOTE_HOST;

    this->psos_serial_port = def;
    this->psos_tcp_host = host;

    double spd;
    // max xpeed
    spd = MOTOR_DEF_MAX_SPEED;
    this->motor_max_speed = static_cast<int>(rint(1e3 * spd));
    // max_yawspeed
    spd = MOTOR_DEF_MAX_TURNSPEED;
    this->motor_max_turnspeed = static_cast<int16_t>(rint(RTOD(spd)));
  // max_xdecel
    spd = 0.0;
    this->motor_max_trans_decel = static_cast<int16_t>(rint(1e3 * spd));
    // max_yawaccel
    spd = 0.0;
    this->motor_max_rot_accel = static_cast<int16_t>(rint(RTOD(spd)));
    // max_yawdecel
    spd = 0.0;
    this->motor_max_rot_decel = static_cast<int16_t>(rint(RTOD(spd)));

    // n_private.param("max_yawspeed", spd, MOTOR_DEF_MAX_TURNSPEED);
    // n_private.param("max_xspeed", spd, MOTOR_DEF_MAX_SPEED);

    // initialize_robot_params();

    this->debug_serial->println("Info: P2OSCommunication setted");
}

P2OSCommunication::~P2OSCommunication() { /** Destructor **/}

int P2OSCommunication::Setup() {
    int i;
    unsigned long bauds[] = {9600, 38400, 19200, 115200, 57600};
    int numbauds = 5; // sizeof(bauds);
    int currbaud = 0;
    //   sippacket = NULL;
    lastPulseTime = 0.0;

    // struct termios term;
    unsigned char command;
    P2OSPacket* packet = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );

    P2OSPacket* receivedpacket = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );

    // int flags = 0;
    bool sent_close = false;

    enum {
        NO_SYNC,
        AFTER_FIRST_SYNC,
        AFTER_SECOND_SYNC,
        READY
    } psos_state;

    psos_state = NO_SYNC;

    char name[20], type[20], subtype[20];
    int cnt;

    if (!this->pioneer_serial) {
        this->debug_serial->println("Error: Serial port initialization failed.");
        while (true);
    }

    this->pioneer_serial->updateBaudRate(bauds[currbaud]);
    this->debug_serial->printf("Info: P2OS connection opening serial with baud rate %i... \n", bauds[currbaud]);
    packet->set_pioneer_serial(*(this->pioneer_serial));
    receivedpacket->set_pioneer_serial(*(this->pioneer_serial));

    this->pioneer_serial->flush();

    int num_sync_attempts = 3;
    while (psos_state != READY) {

        switch (psos_state) {
            case NO_SYNC:
                this->debug_serial->println("NO_SYNC");
                command = SYNC0;
                packet->Build(&command, 1);
                packet->Send();
                delay(1000);
                // delayMicroseconds(P2OS_CYCLETIME_USEC);
                break;
            case AFTER_FIRST_SYNC:
                this->debug_serial->println("Info: turning off NONBLOCK mode...");
                // if (!this->pioneer_serial->available()) {
                //     this->debug_serial->println("Error: P2OS::Setup():fcntl()");
                //     this->psos_fd = -1;
                //     return 1;
                // }
                command = SYNC1;
                packet->Build(&command, 1);
                packet->Send();
                delay(1000);
                break;
            case AFTER_SECOND_SYNC:
                this->debug_serial->println("SYNC2"); 
                command = SYNC2;
                packet->Build(&command, 1);
                packet->Send();
                delay(1000);
                break;
            default:
                this->debug_serial->println("P2OS::Setup():shouldn't be here... \n");
                break;
        }

        // delayMicroseconds(P2OS_CYCLETIME_USEC);
        // delay(1000);

        // this->debug_serial->println("setup 0");

        if (receivedpacket->Receive()) {
            // this->debug_serial->println("setup 1");

            if ((psos_state == NO_SYNC) && (num_sync_attempts >= 0)) {
                num_sync_attempts--;
                delay(1000);
                // delayMicroseconds(P2OS_CYCLETIME_USEC);
                continue;
            } else {
                if (++currbaud < numbauds) {
                    this->pioneer_serial->updateBaudRate(bauds[currbaud]);
                    packet->set_pioneer_serial(*(this->pioneer_serial));
                    receivedpacket->set_pioneer_serial(*(this->pioneer_serial));
                    num_sync_attempts = 3;
                    continue;
                } else {
                    // tried all speeds; bail
                    break;
                }
            }
        }

        // this->debug_serial->println("setup 2");

        switch (receivedpacket->packet[3]) {
            case SYNC0:
                this->debug_serial->println("SYNC0");
                psos_state = AFTER_FIRST_SYNC;
                break;
            case SYNC1:
                this->debug_serial->println("SYNC1");
                psos_state = AFTER_SECOND_SYNC;
                break;
            case SYNC2:
                this->debug_serial->println("SYNC2");
                psos_state = READY;
                break;
            default:
                // maybe P2OS is still running from last time.  let's try to CLOSE
                // and reconnect
                if (!sent_close) {
                    this->debug_serial->println("sending CLOSE");
                    command = CLOSE;
                    packet->Build(&command, 1);
                    packet->Send();
                    sent_close = true;
                    delay(2000);;
                    this->pioneer_serial->flush();
                    psos_state = NO_SYNC;
                }
                break;
        }

        // this->debug_serial->println("setup 3");
        delay(1000);
    }

    if (psos_state != READY) {
        if (this->psos_use_tcp) {
            this->debug_serial->printf(
                "Couldn't synchronize with P2OS.\n"
                "  Most likely because the robot is not connected %s %s",
                this->psos_use_tcp ? "to the ethernet-serial bridge device " : "to the serial port",
                this->psos_use_tcp ? this->psos_tcp_host.c_str() : this->psos_serial_port.c_str()
            );
        }
        this->psos_fd = -1;
        return 1;
    }
    cnt = 4;
    cnt += snprintf(name, sizeof(name), "%s", &(receivedpacket->packet[cnt]));
    cnt++;
    cnt += snprintf(type, sizeof(type), "%s", &(receivedpacket->packet[cnt]));
    cnt++;
    cnt += snprintf(subtype, sizeof(subtype), "%s", &(receivedpacket->packet[cnt]));
    cnt++;

    // std::string hwID = std::string(name) + ": " + std::string(type) + "/" + std::string(subtype);

    command = OPEN;
    packet->Build(&command, 1);
    packet->Send();
    // delayMicroseconds(P2OS_CYCLETIME_USEC);
    delay(1000);
    command = PULSE;
    packet->Build(&command, 1);
    packet->Send();
    // delayMicroseconds(P2OS_CYCLETIME_USEC);
    delay(1000);

    this->debug_serial->println("Done.");
    this->debug_serial->printf(
        "-> Connected to:\n  name: %s, type: %s, subtype: %s\n", name, type, subtype
    );

    P2OSPacket* accel_packet = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );
    unsigned char accel_command[4];
    // if (this->motor_max_trans_accel > 0) {
        accel_command[0] = SETA;
        accel_command[1] = ARGINT;
        accel_command[2] = this->motor_max_trans_accel & 0x00FF;
        accel_command[3] = (this->motor_max_trans_accel & 0xFF00) >> 8;
        accel_packet->Build(accel_command, 4);
        this->SendReceive(accel_packet, false);
    // }
    
    // first, receive a packet so we know we're connected.
//     if (!sippacket) {
//     // sippacket = new SIP(param_idx);
//     sippacket->odom_frame_id = odom_frame_id;
//     sippacket->base_link_frame_id = base_link_frame_id;
//   }

    while(1){}
}

int P2OSCommunication::Shutdown() {

}

int P2OSCommunication::SendReceive(
    P2OSPacket * pkt, 
    bool publish_data) {

    P2OSPacket* packet_send_recieve = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );

//   if ((this->psos_fd >= 0) && this->sippacket) {
    if (pkt) {
      pkt->Send();
    }

    /* receive a packet */
    // pthread_testcancel();
    if (packet_send_recieve->Receive()) {
      this->debug_serial->printf("RunPsosThread(): Receive errored\n");
      pthread_exit(NULL);
    }

    const bool packet_check =
      packet_send_recieve->packet[0] == 0xFA && packet_send_recieve->packet[1] == 0xFB &&
      (packet_send_recieve->packet[3] == 0x30 || packet_send_recieve->packet[3] == 0x31 ||
      packet_send_recieve->packet[3] == 0x32 || packet_send_recieve->packet[3] == 0x33 ||
      packet_send_recieve->packet[3] == 0x34);
    const bool ser_aux =
      (packet_send_recieve->packet[0] == 0xFA && packet_send_recieve->packet[1] == 0xFB && packet_send_recieve->packet[3] == SERAUX);
    if (packet_check) {
      /* It is a server packet, so process it */
      
    //   this->sippacket->ParseStandard(&packet_send_recieve->packet[3]);
    //   this->sippacket->FillStandard(&(this->p2os_data));

    //   if (publish_data) {
    //     this->StandardSIPPutData(packet.timestamp);
    //   }
    } else if (ser_aux) {
      // This is an AUX serial packet


    //   if (ptz_.isOn()) {
    //     int len = packet.packet[2] - 3;
    //     if (ptz_.cb_.gotPacket()) {
    //       this->debug_serial->printf("PTZ got a message, but alread has the complete packet.\n");
    //     } else {
    //       for (int i = 4; i < 4 + len; ++i) {
    //         ptz_.cb_.putOnBuf(packet.packet[i]);
    //       }
    //     }
    //   }


    } else {
      this->debug_serial->printf("Received other packet!\n");
      packet_send_recieve->PrintHex();
    }
//   }

  return 0;
}

void P2OSCommunication::ToggleSonarPower(unsigned char val) {
  unsigned char command[4];
    P2OSPacket* sonar_power_packet = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );


  command[0] = SONAR;
  command[1] = ARGINT;
  command[2] = val;
  command[3] = 0;
  sonar_power_packet->Build(command, 4);
  SendReceive(sonar_power_packet, false);
}