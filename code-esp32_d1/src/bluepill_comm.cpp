#include "pinout_config.hpp"
#include <Arduino.h>
#include <bluepill_comm.hpp>
#include "utils.hpp"
#include <ArduinoLog.h>

BluepillCommunication::BluepillCommunication() {
    /* Connection pin settings */
    pinMode(P2DX_CON, INPUT);
    pinMode(P2DX_CON_2, OUTPUT);
    // pinMode(LED_CONN, OUTPUT);

    /* Encoders settings */
    pinMode(ENCODER1_S1, OUTPUT);
    pinMode(ENCODER1_S2, OUTPUT);
    pinMode(ENCODER1_S3, OUTPUT);
    pinMode(ENCODER2_S1, OUTPUT);
    pinMode(ENCODER2_S2, OUTPUT);
    pinMode(ENCODER2_S3, OUTPUT);
    this->current_position = nav_msgs::Odometry();

    /* DDS settings */
    pinMode(DDS_1, OUTPUT);
    pinMode(DDS_2, OUTPUT);
    pinMode(DDS_3, OUTPUT);
    pinMode(DDS_4, OUTPUT);
    pinMode(DDS_5, OUTPUT);
    pinMode(DDS_6, OUTPUT);
    // pinMode(DDS_7, OUTPUT);
    // pinMode(DDS_8, OUTPUT);
    this->current_ultrasonic_data = p2os_msgs::SonarArray();
    ::memset(this->dds_gpio_state, LOW, sizeof(this->dds_gpio_state));

    /* Velocity settings */
    pinMode(MOTOR1_COMM1, INPUT);
    pinMode(MOTOR1_COMM2, INPUT);
    pinMode(MOTOR1_COMM3, INPUT);

    pinMode(MOTOR2_COMM1, INPUT);
    pinMode(MOTOR2_COMM2, INPUT);
    pinMode(MOTOR2_COMM3, INPUT);
    // pinMode(MOTOR2_COMM4, INPUT);

    ::memset(this->motor_sample_sum, 0, sizeof(this->motor_sample_sum));
    ::memset(this->motor_gpio_state, 0, sizeof(this->motor_gpio_state));

    this->current_velocity = geometry_msgs::Twist();
    this->current_motors_speed = MotorSpeed();

    this->vel_sample_counter = 0;
    this->vel_time = millis();
    // this->polling_time  = millis();
    this->connection_pin_time = millis();

    this->connection_pin_state = NOT_CONNECTED;
}

BluepillCommunication::~BluepillCommunication() { }

ConnectionStates BluepillCommunication::is_bluepill_connected() {
    return this->connection_pin_state;
}

int8_t BluepillCommunication::to_signed(int value, int bits) {
    if (value & (1 << (bits - 1))) {
        return value | (~((1 << bits) - 1));
    }
    return value;
}

void BluepillCommunication::loop() {
    if (
            // ((millis() - this->vel_time) < VELOCITY_SAMPLE_INTERVAL)
          (this->vel_sample_counter < 4)
    ) {
            for (int i = 0; i < 5; i++) {
        this->motor_sample_sum[0] += digitalRead(MOTOR1_COMM1);
        this->motor_sample_sum[1] += digitalRead(MOTOR1_COMM2);
        this->motor_sample_sum[2] += digitalRead(MOTOR1_COMM3);
        this->motor_sample_sum[3] += digitalRead(MOTOR2_COMM1);
        this->motor_sample_sum[4] += digitalRead(MOTOR2_COMM2);
        this->motor_sample_sum[5] += digitalRead(MOTOR2_COMM3);
        this->vel_sample_counter++;
            }
    } else {
        for (int i = 0; i < NUM_MOTOR_COMMS; i++) {
            if (this->vel_sample_counter == 0) {
                this->motor_gpio_state[i] = 0;
            } else {
                this->motor_gpio_state[i] = (double(this->motor_sample_sum[i]) / this->vel_sample_counter) > 0.5;
            }
        }

        this->current_motors_speed.left = this->to_signed(
            (this->motor_gpio_state[0] << 0) | (this->motor_gpio_state[1] << 1) | (this->motor_gpio_state[2] << 2), 3
        );

        this->current_motors_speed.right = this->to_signed(
            (this->motor_gpio_state[3] << 0) | (this->motor_gpio_state[4] << 1) | (this->motor_gpio_state[5] << 2), 3
        );

        int16_t linear_vel = this->current_motors_speed.right + this->current_motors_speed.left;
        linear_vel = scale(linear_vel, -8, 8, -500, 500) / 2;  // scale(linear_vel, -4, 4, -400, 400);

        int16_t angular_vel = this->current_motors_speed.right - this->current_motors_speed.left;
        angular_vel = scale(angular_vel, -8, 8, -340, 340) / 2;  // scale(angular_vel, -4, 4, -170, 170);

        this->current_velocity.linear.x = double(linear_vel / 1e3);
        this->current_velocity.angular.z = double(angular_vel / 1e3);

        ::memset(this->motor_sample_sum, 0, sizeof(this->motor_sample_sum));
        this->vel_sample_counter = 0;
        this->vel_time = millis();
    }

    if (((millis() - this->connection_pin_time) < CONNECTION_SAMPLE_INTERVAL) || (this->vel_sample_counter < 3)) {
        this->connection_pin_sample_sum += digitalRead(P2DX_CON);
        this->connection_pin_counter++;
    } else {
        this->connection_pin_state =
            (this->connection_pin_sample_sum > (this->connection_pin_counter * 5 / 10)) ? CONNECTED : NOT_CONNECTED;

        this->connection_pin_sample_sum = 0;
        this->connection_pin_counter = 0;
        this->connection_pin_time = millis();
    }
}

MotorSpeed BluepillCommunication::get_each_motor_speed() {
    return this->current_motors_speed;
}

geometry_msgs::Twist BluepillCommunication::get_velocity() {
    return this->current_velocity;
}

void BluepillCommunication::update_p2dx_data(nav_msgs::ros_p2os_data_t data) {
    if ((this->current_position.pose.pose.position.x != data.position.pose.pose.position.x) ||
        (this->current_position.pose.pose.position.y != data.position.pose.pose.position.y) ||
        (this->current_position.pose.pose.position.z != data.position.pose.pose.position.z)) {
        this->current_position = data.position;
        update_encoder_data(data.position);
    }

    if (this->current_ultrasonic_data.ranges != data.sonar.ranges) {
        this->current_ultrasonic_data = data.sonar;
        update_dds_data(data.sonar);
    }
}

void BluepillCommunication::update_encoder_data(nav_msgs::Odometry position) {
    double x = position.pose.pose.position.x;
    double y = position.pose.pose.position.y;

    double yaw = this->getYaw(position);  // Extract yaw from quaternion

    double wheel_radius = 0.1;  // meters
    double wheelbase = 0.5;     // meters

    // Compute the distance traveled and wheel rotations
    double d = sqrt(x * x + y * y);  // Approximate linear distance traveled
    double delta_theta = yaw;        // Change in orientation

    double left_wheel_position = (d - (delta_theta * wheelbase / 2)) / wheel_radius;
    double right_wheel_position = (d + (delta_theta * wheelbase / 2)) / wheel_radius;

    // Normalize to 8 discrete encoder values
    int left_encoder = static_cast<int>(fmod(left_wheel_position, 1.0) * 8) % 8;
    int right_encoder = static_cast<int>(fmod(right_wheel_position, 1.0) * 8) % 8;

    // Ensure non-negative encoder values
    u_int8_t encoder_1 = (left_encoder + 8) % 8;
    u_int8_t encoder_2 = (right_encoder + 8) % 8;

    digitalWrite(ENCODER1_S1, encoder_1 & 0b100);
    digitalWrite(ENCODER1_S2, encoder_1 & 0b010);
    digitalWrite(ENCODER1_S3, encoder_1 & 0b001);
    digitalWrite(ENCODER2_S1, encoder_2 & 0b100);
    digitalWrite(ENCODER2_S2, encoder_2 & 0b010);
    digitalWrite(ENCODER2_S3, encoder_2 & 0b001);
}

void BluepillCommunication::update_dds_data(p2os_msgs::SonarArray dds_data) {
    int32_t values_to_read = dds_data.ranges_count;
    if (dds_data.ranges_count < NUM_DDS) {
        Log.infoln("BluepillCommunication:update_dds_data range count smaller than 8");
    }

    if (dds_data.ranges_count > NUM_DDS) {
        Log.infoln("BluepillCommunication:update_dds_data range count bigger than 8");
        int32_t values_to_read = NUM_DDS;
    }

    for (int i = 0; i < NUM_DDS; i++) {
        if (i < values_to_read) {
            dds_gpio_state[i] = (dds_data.ranges[i] > 1) ? LOW : HIGH;
        } else {
            dds_gpio_state[i] = 0;
        }
    }
    dds_gpio_state[0] = LOW;

    digitalWrite(DDS_1, dds_gpio_state[DDS_1_POS]);
    digitalWrite(DDS_2, dds_gpio_state[DDS_2_POS]);
    digitalWrite(DDS_3, dds_gpio_state[DDS_3_POS]);
    digitalWrite(DDS_4, dds_gpio_state[DDS_4_POS]);
    digitalWrite(DDS_5, dds_gpio_state[DDS_5_POS]);
    digitalWrite(DDS_6, dds_gpio_state[DDS_6_POS]);
    // digitalWrite(DDS_7, dds_gpio_state[DDS_7_POS]);
    // digitalWrite(DDS_8, dds_gpio_state[7]);
}

double BluepillCommunication::getYaw(nav_msgs::Odometry position) {
    double y = position.pose.pose.position.y;
    double x = position.pose.pose.position.x;
    double yaw = atan2(y, x);
    return yaw;
}

void BluepillCommunication::send_bluepill_connection(ConnectionStates connection_state) {
    if (connection_state == CONNECTED) {
        digitalWrite(P2DX_CON_2, HIGH);
    } else {
        digitalWrite(P2DX_CON_2, LOW);
    }
}