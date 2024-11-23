#ifndef _BLUEPILL_CONFIG_HPP_
#define _BLUEPILL_CONFIG_HPP_

/* Debug Config */
#define BLUEPILL_DEBUG_PRINT  // to use p2os debug prints uncomment this line
#define BLUEPILL_INFO_PRINT   // to use p2os info prints uncomment this line
#define BLUEPILL_ERROR_PRINT  // to use p2os error prints uncomment this line

/* Velocity Config */
#define VELOCITY_SAMPLE_INTERVAL 10

#define NUM_MOTOR_COMMS 6
#define MOTOR1_COMM1 0
#define MOTOR1_COMM2 0
#define MOTOR1_COMM3 0
#define MOTOR2_COMM1 0
#define MOTOR2_COMM2 0
#define MOTOR2_COMM3 0

/* DDS Config */
#define DDS_THRESHOLD 4

#define NUM_DDS 8
#define DDS_1 0
#define DDS_2 0
#define DDS_3 0
#define DDS_4 0
#define DDS_5 0
#define DDS_6 0
#define DDS_7 0
#define DDS_8 0

/* Encoder Config */
#define NUM_ENCODERS_PIN 6
#define ENCODER1_S1 0
#define ENCODER1_S2 0
#define ENCODER1_S3 0
#define ENCODER2_S1 0
#define ENCODER2_S2 0
#define ENCODER2_S3 0

/* Connection pin Config */
#define CONNECTION_SAMPLE_INTERVAL 10
#define P2DX_CON 0

#endif  // _BLUEPILL_CONFIG_HPP_
