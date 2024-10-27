/**  P2OS for Arduino **/

#include <p2os.hpp>
// #include <termios.h>
#include <Arduino.h>


P2OSCommunication::P2OSCommunication(HardwareSerial& debug_serial, HardwareSerial& pioneer_serial) {
    
    this->debug_serial = &debug_serial;
    this->pioneer_serial = &pioneer_serial;   
    
    // read in config options
    this->bumpstall = -1;   // bumpstall
    this->pulse = -1.0;     // pulse
    this->rot_kp = -1;      // rot_kp
    this->rot_kv = -1;      // rot_kv
    this->rot_ki = -1;      // rot_ki
    this->trans_kp = -1;    // trans_kp
    this->trans_kv = -1;    // trans_kv
    this->trans_ki = -1;    // trans_ki
    std::string def = DEFAULT_P2OS_PORT; // !!! port !!!
    this->psos_serial_port = def;
    this->psos_use_tcp = false;
    std::string host = DEFAULT_P2OS_TCP_REMOTE_HOST;
    this->psos_tcp_host = host;
    this->psos_tcp_port = DEFAULT_P2OS_TCP_REMOTE_PORT;
    this->radio_modemp = 0; // radio
    this->joystick = 0;     // joystick
    this->direct_wheel_vel_control = 0; // direct_wheel_vel_control
    double spd; // max xpeed
    spd = MOTOR_DEF_MAX_SPEED;
    this->motor_max_speed = static_cast<int>(rint(1e3 * spd));
    
    spd = MOTOR_DEF_MAX_TURNSPEED; // max_yawspeed
    this->motor_max_turnspeed = static_cast<int16_t>(rint(RTOD(spd)));
    spd = 0.0;  // max_xdecel
    this->motor_max_trans_decel = static_cast<int16_t>(rint(1e3 * spd));
    spd = 0.0;  // max_yawaccel
    this->motor_max_rot_accel = static_cast<int16_t>(rint(RTOD(spd)));
    spd = 0.0;  // max_yawdecel
    this->motor_max_rot_decel = static_cast<int16_t>(rint(RTOD(spd)));

    initialize_robot_params();
    this->debug_serial->println("Info: P2OSCommunication setted");
}

P2OSCommunication::~P2OSCommunication() { /** Destructor **/}

int P2OSCommunication::Setup() {
    int i;
    unsigned long bauds[] = {9600, 38400, 19200, 115200, 57600};
    int numbauds = 5; // sizeof(bauds);
    int currbaud = 0;
    // sippacket = NULL;
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
    this->debug_serial->printf("Info: P2OS connection serial with baudrate %i... \n", bauds[currbaud]);
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
                delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);
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
                delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);
                break;
            case AFTER_SECOND_SYNC:
                this->debug_serial->println("SYNC2"); 
                command = SYNC2;
                packet->Build(&command, 1);
                packet->Send();
                delay(200);
                break;
            default:
                this->debug_serial->println("P2OS::Setup():shouldn't be here... \n");
                break;
        }
        delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);

        if (receivedpacket->Receive()) {
            if ((psos_state == NO_SYNC) && (num_sync_attempts >= 0)) {
                num_sync_attempts--;
                delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);
                continue;
            } else {
                if (++currbaud < numbauds) {
                    this->debug_serial->printf("Info: P2OS connection serial with baudrate %i... \n", bauds[currbaud]);
                    this->pioneer_serial->updateBaudRate(bauds[currbaud]);
                    this->pioneer_serial->flush();

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
                    delay(2*200); // delayMicroseconds(2*P2OS_CYCLETIME_USEC);
                    this->pioneer_serial->flush();
                    psos_state = NO_SYNC;
                }
                break;
        }
        delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);
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
    // diagnostic_.setHardwareID(hwID);

    command = OPEN;
    packet->Build(&command, 1);
    packet->Send();
    delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);
    command = PULSE;
    packet->Build(&command, 1);
    packet->Send();
    delay(200); // delayMicroseconds(P2OS_CYCLETIME_USEC);

    this->debug_serial->println("Done.");
    this->debug_serial->printf(
        "-> Connected to:\n  name: %s, type: %s, subtype: %s\n", name, type, subtype
    );

    // now, based on robot type, find the right set of parameters
    for (i = 0; i < PLAYER_NUM_ROBOT_TYPES; i++) {
        if (!strcasecmp(PlayerRobotParams[i].Class.c_str(), type) &&
        !strcasecmp(PlayerRobotParams[i].Subclass.c_str(), subtype)) {
            param_idx = i;
            break;
        }
    }
    if (i == PLAYER_NUM_ROBOT_TYPES) {
        this->debug_serial->println("P2OS: Warning: couldn't find parameters for this robot; ");
        this->debug_serial->println("using defaults");
        param_idx = 0;
    }

    // if (!sippacket) {
    //     sippacket = new SIP(param_idx);
    //     sippacket->odom_frame_id = odom_frame_id;
    //     sippacket->base_link_frame_id = base_link_frame_id;
    // }
    // /*
    //     sippacket->x_offset = 0;
    //     sippacket->y_offset = 0;
    //     sippacket->angle_offset = 0;

    //     SendReceive((P2OSPacket*)NULL,false);
    // */
    // turn off the sonars at first
    this->ToggleSonarPower(0);
    P2OSPacket* accel_packet = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );
    unsigned char accel_command[4];
    if (this->motor_max_trans_accel > 0) {
        accel_command[0] = SETA;
        accel_command[1] = ARGINT;
        accel_command[2] = this->motor_max_trans_accel & 0x00FF;
        accel_command[3] = (this->motor_max_trans_accel & 0xFF00) >> 8;
        accel_packet->Build(accel_command, 4);
        this->SendReceive(accel_packet, false);
    }

    if (this->motor_max_trans_decel < 0) {
        accel_command[0] = SETA;
        accel_command[1] = ARGNINT;
        accel_command[2] = -1 * (this->motor_max_trans_decel) & 0x00FF;
        accel_command[3] = (-1 * (this->motor_max_trans_decel) & 0xFF00) >> 8;
        accel_packet->Build(accel_command, 4);
        this->SendReceive(accel_packet, false);
    }

    if (this->motor_max_rot_accel > 0) {
        accel_command[0] = SETRA;
        accel_command[1] = ARGINT;
        accel_command[2] = this->motor_max_rot_accel & 0x00FF;
        accel_command[3] = (this->motor_max_rot_accel & 0xFF00) >> 8;
        accel_packet->Build(accel_command, 4);
        this->SendReceive(accel_packet, false);
    }

    if (this->motor_max_rot_decel < 0) {
        accel_command[0] = SETRA;
        accel_command[1] = ARGNINT;
        accel_command[2] = -1 * (this->motor_max_rot_decel) & 0x00FF;
        accel_command[3] = (-1 * (this->motor_max_rot_decel) & 0xFF00) >> 8;
        accel_packet->Build(accel_command, 4);
        this->SendReceive(accel_packet, false);
    }

    // if requested, change PID settings
    P2OSPacket* pid_packet = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );
    unsigned char pid_command[4];
    if (this->rot_kp >= 0) {
        pid_command[0] = ROTKP;
        pid_command[1] = ARGINT;
        pid_command[2] = this->rot_kp & 0x00FF;
        pid_command[3] = (this->rot_kp & 0xFF00) >> 8;
        pid_packet->Build(pid_command, 4);
        this->SendReceive(pid_packet);
    }

    if (this->rot_kv >= 0) {
        pid_command[0] = ROTKV;
        pid_command[1] = ARGINT;
        pid_command[2] = this->rot_kv & 0x00FF;
        pid_command[3] = (this->rot_kv & 0xFF00) >> 8;
        pid_packet->Build(pid_command, 4);
        this->SendReceive(pid_packet);
    }

    if (this->rot_ki >= 0) {
        pid_command[0] = ROTKI;
        pid_command[1] = ARGINT;
        pid_command[2] = this->rot_ki & 0x00FF;
        pid_command[3] = (this->rot_ki & 0xFF00) >> 8;
        pid_packet->Build(pid_command, 4);
        this->SendReceive(pid_packet);
    }

    if (this->trans_kp >= 0) {
        pid_command[0] = TRANSKP;
        pid_command[1] = ARGINT;
        pid_command[2] = this->trans_kp & 0x00FF;
        pid_command[3] = (this->trans_kp & 0xFF00) >> 8;
        pid_packet->Build(pid_command, 4);
        this->SendReceive(pid_packet);
    }

    if (this->trans_kv >= 0) {
        pid_command[0] = TRANSKV;
        pid_command[1] = ARGINT;
        pid_command[2] = this->trans_kv & 0x00FF;
        pid_command[3] = (this->trans_kv & 0xFF00) >> 8;
        pid_packet->Build(pid_command, 4);
        this->SendReceive(pid_packet);
    }

    if (this->trans_ki >= 0) {
        pid_command[0] = TRANSKI;
        pid_command[1] = ARGINT;
        pid_command[2] = this->trans_ki & 0x00FF;
        pid_command[3] = (this->trans_ki & 0xFF00) >> 8;
        pid_packet->Build(pid_command, 4);
        this->SendReceive(pid_packet);
    }

    while(1){
        command = PULSE;
        packet->Build(&command, 1);
        packet->Send();
        delay(200);
    }
}

int P2OSCommunication::Shutdown() {
    unsigned char command[20], buffer[20];
    P2OSPacket* packet_shutdown = new P2OSPacket(
        *(this->debug_serial),
        *(this->pioneer_serial)
    );

    // if (ptz_.isOn()) {
    //     ptz_.shutdown();
    // }

    memset(buffer, 0, 20);

    if (this->psos_fd == -1) {
        return -1;
    }

    command[0] = STOP;
    packet_shutdown->Build(command, 1);
    packet_shutdown->Send();
    delay(200); // usleep(P2OS_CYCLETIME_USEC);

    command[0] = CLOSE;
    packet_shutdown->Build(command, 1);
    packet_shutdown->Send();
    delay(200); // usleep(P2OS_CYCLETIME_USEC);

    close(this->psos_fd);
    this->psos_fd = -1;
    this->debug_serial->println("P2OS has been shutdown");
    // delete this->sippacket;
    // this->sippacket = NULL;

    return 0;
}

/* send the packet, then receive and parse an SIP */
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
    //   pthread_exit(NULL);
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
      this->debug_serial->println("Received packet!");

    //   this->sippacket->ParseStandard(&packet_send_recieve->packet[3]);
    //   this->sippacket->FillStandard(&(this->p2os_data));

    //   if (publish_data) {
    //     this->StandardSIPPutData(packet.timestamp);
    //   }
    } else if (ser_aux) {
        this->debug_serial->println("Received ser_aux packet!");
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
      this->debug_serial->println("Received other packet!");
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