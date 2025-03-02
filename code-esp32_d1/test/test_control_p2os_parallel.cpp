#include <ps5Controller.h>
#include <Arduino.h>
#include <p2os.hpp>
#include <HardwareSerial.h>

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0);    // define a Serial for UART0
HardwareSerial pioneer_serial(2);  // define a Serial for UART2

int scale_test(int16_t value, int16_t old_min, int16_t old_max, int16_t new_min, int16_t new_max) {
    return int(new_min + (value - old_min) * (new_max - new_min) / (old_max - old_min));
}

QueueHandle_t msg_queue_vel;
QueueHandle_t msg_queue_conn_command;

TaskHandle_t TaskBluetooth;
TaskHandle_t TaskP2OS;

enum BTConnectionStates {
    NOT_CONNECTED,
    CONNECTED
};

void task_bluetooth_code(void* pvParameters);
void task_p2os_code(void* pvParameters);

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();
    // xSemaphore = xSemaphoreCreateBinary();

    msg_queue_vel = xQueueCreate(1, sizeof(geometry_msgs::Twist));
    msg_queue_conn_command = xQueueCreate(1, sizeof(BTConnectionStates));

    int32_t size_base = 1024;

    xTaskCreatePinnedToCore(task_bluetooth_code, "TaskBluetooth", size_base * 4, NULL, 1, &TaskBluetooth, 0);

    xTaskCreatePinnedToCore(task_p2os_code, "TaskP2OS", size_base * 4, NULL, 1, &TaskP2OS, 1);

    debug_serial.println("Ready!");
}

void task_bluetooth_code(void* pvParameters) {
    ps5.begin("10:18:49:7D:EC:F1");  // replace with MAC address of your controller
    uint32_t             current_loop_time_bt;
    geometry_msgs::Twist msg_vel;

    while (true) {
        BTConnectionStates is_connected_bt = NOT_CONNECTED;
        uint8_t            current_r2_val = 0;
        uint8_t            current_l2_val = 0;
        int16_t            current_rs_x_val = 0;

        BTConnectionStates connection_command;
        bool               need_send_connect_msg = false;

        while (ps5.isConnected() == true) {
            current_loop_time_bt = millis();

            if (ps5.Up()) {
                connection_command = CONNECTED;
                need_send_connect_msg = true;
            }

            if (ps5.Down()) {
                connection_command = NOT_CONNECTED;
                need_send_connect_msg = true;
            }

            if (need_send_connect_msg && msg_queue_conn_command != 0) {
                if (xQueueSend(msg_queue_conn_command, &connection_command, portMAX_DELAY) == pdTRUE) {
                    need_send_connect_msg = false;
                    is_connected_bt = connection_command;
                }
            }

            if (is_connected_bt == CONNECTED) {
                current_r2_val = ps5.R2Value();  // value 0 - 255
                current_l2_val = ps5.L2Value();  // value 0 - 255
                current_rs_x_val = (-1) * scale_test(ps5.RStickX(), -128, 128, -170, 170);

                msg_vel.linear.x = double(double(current_r2_val - current_l2_val) / 100);
                msg_vel.angular.z = double(current_rs_x_val) / 100;

                if (xQueueOverwrite(msg_queue_vel, &msg_vel) == pdFALSE) {
                    debug_serial.println("Failed to send the data");
                }
            }

            // debug_serial.printf("BT: current_loop_time: %ld \n", (millis() - current_loop_time_bt));
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void task_p2os_code(void* pvParameters) {
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    pioneer_serial.flush();
    P2OS* p2os = new P2OS(debug_serial, pioneer_serial);

    bool                  is_connected_p2os = false;
    uint32_t              current_loop_time_p2os;
    uint32_t              last_time_motor_state = 0;
    BTConnectionStates    connection_command_p2os;
    geometry_msgs::Twist  msg_p2os_vel;
    p2os_msgs::MotorState msg_motor_state;

    double x_val = 0;
    while (true) {
        current_loop_time_p2os = millis();

        if (msg_queue_conn_command != 0) {
            if (xQueueReceive(msg_queue_conn_command, &connection_command_p2os, 0) == pdTRUE) {
                if (!(connection_command_p2os == is_connected_p2os)) {
                    if (connection_command_p2os == CONNECTED) {
                        is_connected_p2os = !(p2os->setup());
                        is_connected_p2os = true;
                        debug_serial.printf("P2OS: setup! \n");
                    } else if (connection_command_p2os == NOT_CONNECTED) {
                        is_connected_p2os = p2os->shutdown();
                        debug_serial.printf("P2OS: shutdown! \n");
                    }
                }
            }
        }

        if (is_connected_p2os) {
            p2os->loop();

            if (msg_queue_vel != 0) {
                if (xQueueReceive(msg_queue_vel, &msg_p2os_vel, portMAX_DELAY) == pdTRUE) {
                    p2os->set_vel(&msg_p2os_vel);
                }
            }

            if (millis() - last_time_motor_state > 100) {
                msg_motor_state.state = 1;
                p2os->set_motor_state(&msg_motor_state);
                last_time_motor_state = millis();
            }
        }

        // debug_serial.printf("P2OS: current_loop_time: %ld \n", (millis() - current_loop_time_p2os));
    }
}

void loop() { }