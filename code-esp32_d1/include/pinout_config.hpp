#ifndef _PINOUT_CONFIG_HPP_
#define _PINOUT_CONFIG_HPP_

/* PS5 Controller mac address */
#define PS5_CONTROLLER_MAC_ADDRESS "10:18:49:7D:EC:F1"

/* P2OS Serial */
#define P2OS_BAUDRATE 9600
#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

/* DDS Config */
#define DDS_THRESHOLD 4

#define NUM_DDS 6
#define DDS_1 10
#define DDS_2 13
#define DDS_3 5
#define DDS_4 23
#define DDS_5 19
#define DDS_6 18
#define DDS_7 26
// #define DDS_8

/* Velocity Config */
#define VELOCITY_SAMPLE_INTERVAL 20
// #define VELOCITY_MIN_SAMPLE_NUMBER 5
// #define MOTOR_SAMPLE_INTERVAL 20

#define NUM_MOTOR_COMMS 6
#define MOTOR1_COMM1 32
#define MOTOR1_COMM2 12
#define MOTOR1_COMM3 34
// #define MOTOR1_COMM4 DDS_1 /** The same pins as DDS_1 */

#define MOTOR2_COMM1 33  // 39
#define MOTOR2_COMM2 35  // 35
#define MOTOR2_COMM3 39  // 33
// #define MOTOR2_COMM4 DDS_1 /** The same pins as DDS_2 */

/* Encoder Config */
#define NUM_ENCODERS_PIN 6
#define ENCODER1_S1 27
#define ENCODER1_S2 25
#define ENCODER1_S3 2
#define ENCODER2_S1 14
#define ENCODER2_S2 22
#define ENCODER2_S3 4

/* Connection pin Config */
#define CONNECTION_SAMPLE_INTERVAL 10
#define P2DX_CON 15
#define P2DX_CON_2 21

/* Connection pin Config */
// #define LED_CONN 36

/* Switch */
#define MODE_1_PIN 19
#define MODE_2_PIN 23

#endif  // _PINOUT_CONFIG_HPP_
