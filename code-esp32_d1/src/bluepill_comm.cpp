#include "bluepill_config.hpp"
#include <Arduino.h>
#include <bluepill_comm.hpp>

BluepillCommunication::BluepillCommunication(HardwareSerial& debug_serial) {
    this->debug_serial = &debug_serial;

    /* Connection pin settings */
    pinMode(P2DX_CON, INPUT);

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
    pinMode(DDS_7, OUTPUT);
    pinMode(DDS_8, OUTPUT);
    this->current_ultrasonic_data = p2os_msgs::SonarArray();
    ::memset(this->dds_gpio_state, LOW, sizeof(this->dds_gpio_state));

    /* Velocity settings */
    pinMode(MOTOR1_COMM1, INPUT);
    pinMode(MOTOR1_COMM2, INPUT);
    pinMode(MOTOR1_COMM3, INPUT);
    pinMode(MOTOR2_COMM1, INPUT);
    pinMode(MOTOR2_COMM2, INPUT);
    pinMode(MOTOR2_COMM3, INPUT);

    ::memset(this->motor_sample_sum, 0, sizeof(this->motor_sample_sum));
    ::memset(this->motor_gpio_state, false, sizeof(this->motor_gpio_state));

    this->current_velocity = geometry_msgs::Twist();
    this->vel_sample_counter = 0;
    this->vel_time = millis();
    this->connection_pin_time = millis();
}

BluepillCommunication::~BluepillCommunication() { }

bool BluepillCommunication::is_bluepill_connected() {
    return this->connection_pin_state;
}

void BluepillCommunication::loop() {
    if (((millis() - this->vel_time) < VELOCITY_SAMPLE_INTERVAL) || (this->vel_sample_counter < 3)) {
        this->motor_sample_sum[0] += digitalRead(MOTOR1_COMM1);
        this->motor_sample_sum[1] += digitalRead(MOTOR1_COMM2);
        this->motor_sample_sum[2] += digitalRead(MOTOR1_COMM3);
        this->motor_sample_sum[3] += digitalRead(MOTOR2_COMM1);
        this->motor_sample_sum[4] += digitalRead(MOTOR2_COMM2);
        this->motor_sample_sum[5] += digitalRead(MOTOR2_COMM3);
        this->vel_sample_counter++;
    } else {
        this->motor_gpio_state[0] = double(this->motor_sample_sum[0] / this->vel_sample_counter) > 0.5;
        this->motor_gpio_state[1] = double(this->motor_sample_sum[1] / this->vel_sample_counter) > 0.5;
        this->motor_gpio_state[2] = double(this->motor_sample_sum[2] / this->vel_sample_counter) > 0.5;
        this->motor_gpio_state[3] = double(this->motor_sample_sum[3] / this->vel_sample_counter) > 0.5;
        this->motor_gpio_state[4] = double(this->motor_sample_sum[4] / this->vel_sample_counter) > 0.5;
        this->motor_gpio_state[5] = double(this->motor_sample_sum[5] / this->vel_sample_counter) > 0.5;

        int16_t linear_vel =
            (this->motor_gpio_state[0] << 2) | (this->motor_gpio_state[1] << 1) | this->motor_gpio_state[2];
        int16_t angular_vel =
            (this->motor_gpio_state[3] << 2) | (this->motor_gpio_state[4] << 1) | this->motor_gpio_state[5];

        this->current_velocity.linear.x = this->scale(linear_vel, -4, 4, -400, 400);
        this->current_velocity.angular.z = this->scale(angular_vel, -4, 4, -100, 100);

        ::memset(this->motor_sample_sum, 0, sizeof(this->motor_sample_sum));
        this->vel_sample_counter = 0;
        this->vel_time = millis();
    }

    if (((millis() - this->connection_pin_time) < CONNECTION_SAMPLE_INTERVAL) || (this->vel_sample_counter < 3)) {
        this->connection_pin_sample_sum += digitalRead(P2DX_CON);
        this->connection_pin_counter++;
    } else {
        this->connection_pin_state =
            (double(this->connection_pin_sample_sum / this->connection_pin_counter) > 0.5) ? CONNECTED : NOT_CONNECTED;

        this->connection_pin_sample_sum = 0;
        this->connection_pin_counter = 0;
        this->connection_pin_time = millis();
    }
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
#if BLUEPILL_INFO_PRINT
        this->debug_serial->println("Info: BluepillCommunication:update_dds_data range count smaller than 8");
#endif
    }

    if (dds_data.ranges_count > NUM_DDS) {
#if BLUEPILL_INFO_PRINT
        this->debug_serial->println("Info: BluepillCommunication:update_dds_data range count bigger than 8");
#endif
        int32_t values_to_read = NUM_DDS;
    }

    for (int i = 0; i < values_to_read; i++) {
        dds_gpio_state[i] = (dds_data.ranges[i] > DDS_THRESHOLD) ? HIGH : LOW;
    }

    digitalWrite(DDS_1, dds_gpio_state[0]);
    digitalWrite(DDS_2, dds_gpio_state[1]);
    digitalWrite(DDS_3, dds_gpio_state[2]);
    digitalWrite(DDS_4, dds_gpio_state[3]);
    digitalWrite(DDS_5, dds_gpio_state[4]);
    digitalWrite(DDS_6, dds_gpio_state[5]);
    digitalWrite(DDS_7, dds_gpio_state[6]);
    digitalWrite(DDS_8, dds_gpio_state[7]);
}

int16_t
    BluepillCommunication::scale(int16_t value, int16_t old_min, int16_t old_max, int16_t new_min, int16_t new_max) {
    return int(new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min));
}

double BluepillCommunication::getYaw(nav_msgs::Odometry position) {
    double y = position.pose.pose.position.y;
    double x = position.pose.pose.position.x;
    double yaw = atan2(y, x);
    return yaw;
}