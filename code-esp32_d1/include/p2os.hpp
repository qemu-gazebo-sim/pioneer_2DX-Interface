/**  P2OS for Arduino **/

#ifndef _P2OS_HPP_
#define _P2OS_HPP_


#include <Arduino.h>
#include "packet.hpp"
#include "p2os_msgs.hpp"
#include "robot_params.hpp"
#include "sip.hpp"

class P2OSCommunication {
    private:
        HardwareSerial* debug_serial;
        HardwareSerial* pioneer_serial;
        int motor_max_speed;
        int motor_max_turnspeed;
        int16_t motor_max_trans_accel;
        int16_t motor_max_trans_decel;
        int16_t motor_max_rot_accel;
        int16_t motor_max_rot_decel;

    public:
        SIP * sippacket;
        nav_msgs::ros_p2os_data_t p2os_data;

        P2OSCommunication(
            HardwareSerial& debug_serial, 
            HardwareSerial& pioneer_serial
        );
        ~P2OSCommunication();

    public:
        //! Setup the robot for use. Communicates with the robot directly.
        int Setup();
        //! Prepare for shutdown.
        int Shutdown();

        int SendReceive(P2OSPacket * pkt, bool publish_data = true);

        void updateDiagnostics();


        void SendPulse(void);

        void check_and_set_vel(int lin_vel, int ang_vel);
        //   void cmdvel_cb(const geometry_msgs::TwistConstPtr &);

        void check_and_set_motor_state();
        //   void cmdmotor_state(const p2os_msgs::MotorStateConstPtr &);

        // diagnostic messages
        //   void check_voltage(diagnostic_updater::DiagnosticStatusWrapper & stat);
        //   void check_stall(diagnostic_updater::DiagnosticStatusWrapper & stat);
        void ToggleSonarPower(unsigned char val);
        void ToggleMotorPower(unsigned char val);

    protected:
        std::string psos_serial_port;
        std::string psos_tcp_host;
        std::string odom_frame_id;
        std::string base_link_frame_id;
        // int psos_fd;
        bool psos_use_tcp;
        int psos_tcp_port;
        bool vel_dirty, motor_dirty;
        bool gripper_dirty_ = false;
        int param_idx;
        // PID settings
        int rot_kp, rot_kv, rot_ki, trans_kp, trans_kv, trans_ki;

        //! Stall I hit a wall?
        int bumpstall;   // should we change the bumper-stall behavior?
        //! Use Joystick?
        // int joystick;
        //! Control wheel velocities individually?
        int direct_wheel_vel_control;
        // int radio_modemp;
        //! Pulse time
        // double pulse;
        // double desired_freq;
        //! Last time the node received or sent a pulse.
        // double lastPulseTime;
        //! Use the sonar array?
        // bool use_sonar_;

};

#endif  // _P2OS_HPP_
