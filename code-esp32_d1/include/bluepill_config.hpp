#ifndef _BLUEPILL_CONFIG_HPP_
#define _BLUEPILL_CONFIG_HPP_

/* Debug Config */
#define BLUEPILL_DEBUG_PRINT  // to use p2os debug prints uncomment this line
#define BLUEPILL_INFO_PRINT   // to use p2os info prints uncomment this line
#define BLUEPILL_ERROR_PRINT  // to use p2os error prints uncomment this line

/* Velocity Config */
#define VELOCITY_SAMPLE_INTERVAL 10

#define NUM_MOTOR_COMMS 6
#define MOTOR1_COMM1 12
#define MOTOR1_COMM2 4
#define MOTOR1_COMM3 0
#define MOTOR2_COMM1 27
#define MOTOR2_COMM2 25
#define MOTOR2_COMM3 32

/* DDS Config */
#define DDS_THRESHOLD 4

#define NUM_DDS 8
#define DDS_1 10
#define DDS_2 13
#define DDS_3 5
#define DDS_4 23
#define DDS_5 19
#define DDS_6 18
#define DDS_7 26
#define DDS_8 36

/* Encoder Config */
#define NUM_ENCODERS_PIN 6
#define ENCODER1_S1 6
#define ENCODER1_S2 8
#define ENCODER1_S3 2
#define ENCODER2_S1 14
#define ENCODER2_S2 9
#define ENCODER2_S3 11

/* Connection pin Config */
#define CONNECTION_SAMPLE_INTERVAL 10
#define P2DX_CON 15
#define P2DX_CON_2 7

#endif  // _BLUEPILL_CONFIG_HPP_
