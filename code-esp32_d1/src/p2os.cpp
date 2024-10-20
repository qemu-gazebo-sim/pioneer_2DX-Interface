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

    int flags = 0;
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

    this->debug_serial->printf("Info: P2OS connection opening serial port %s... \n", this->psos_serial_port.c_str());
    this->pioneer_serial->updateBaudRate(bauds[currbaud]);
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
                delayMicroseconds(P2OS_CYCLETIME_USEC);
                break;
            case AFTER_FIRST_SYNC:
                this->debug_serial->println("Info: turning off NONBLOCK mode...");
                if (!this->pioneer_serial->available()) {
                    this->debug_serial->println("Error: P2OS::Setup():fcntl()");
                    this->psos_fd = -1;
                    return 1;
                }
                command = SYNC1;
                packet->Build(&command, 1);
                packet->Send();
                break;
            case AFTER_SECOND_SYNC:
                command = SYNC2;
                packet->Build(&command, 1);
                packet->Send();
                break;
            default:
                this->debug_serial->println("P2OS::Setup():shouldn't be here... \n");
                break;
        }

        delayMicroseconds(P2OS_CYCLETIME_USEC);

        this->debug_serial->println("setup 0");

        if (receivedpacket->Receive()) {
            this->debug_serial->println("setup 1");

            if ((psos_state == NO_SYNC) && (num_sync_attempts >= 0)) {
                num_sync_attempts--;
                delayMicroseconds(P2OS_CYCLETIME_USEC);
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

        this->debug_serial->println("setup 2");

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
                    delayMicroseconds(2 * P2OS_CYCLETIME_USEC);
                    this->pioneer_serial->flush();
                    psos_state = NO_SYNC;
                }
                break;
        }

        this->debug_serial->println("setup 3");
        delayMicroseconds(P2OS_CYCLETIME_USEC);
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
    cnt += snprintf(name, sizeof(name), "%s", &receivedpacket->packet[cnt]);
    cnt++;
    cnt += snprintf(type, sizeof(type), "%s", &receivedpacket->packet[cnt]);
    cnt++;
    cnt += snprintf(subtype, sizeof(subtype), "%s", &receivedpacket->packet[cnt]);
    cnt++;

    // std::string hwID = std::string(name) + ": " + std::string(type) + "/" + std::string(subtype);

    command = OPEN;
    packet->Build(&command, 1);
    packet->Send();
    delayMicroseconds(P2OS_CYCLETIME_USEC);
    command = PULSE;
    packet->Build(&command, 1);
    packet->Send();
    delayMicroseconds(P2OS_CYCLETIME_USEC);

    this->debug_serial->printf(
        "Done.\n   Connected to %s, a %s %s", name, type, subtype
    );

    while(1){}
}

int P2OSCommunication::Shutdown() {

}

int P2OSCommunication::SendReceive(
    P2OSPacket * pkt, 
    bool publish_data) {

}