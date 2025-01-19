#include <Arduino.h>
#include <HardwareSerial.h>
#include <p2os.hpp>
#include "bluepill_comm.hpp"

#define PIONEER_SERIAL_RX 16
#define PIONEER_SERIAL_TX 17

HardwareSerial debug_serial(0);
HardwareSerial pioneer_serial(2);  // define a Serial for UART2

QueueHandle_t msg_queue_vel;
QueueHandle_t msg_queue_conn_command;
QueueHandle_t msg_queue_sensors;

TaskHandle_t TaskBluepill;
TaskHandle_t TaskP2OS;

void task_bluepill_code(void* pvParameters);
void task_p2os_code(void* pvParameters);

BluepillCommunication* bluepill_comm;
P2OS*                  p2os;

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();
    pioneer_serial.begin(9600, SERIAL_8N1, PIONEER_SERIAL_RX, PIONEER_SERIAL_TX);
    pioneer_serial.flush();

    bluepill_comm = new BluepillCommunication(debug_serial);
    p2os = new P2OS(debug_serial, pioneer_serial);

    msg_queue_vel = xQueueCreate(1, sizeof(geometry_msgs::Twist));
    msg_queue_conn_command = xQueueCreate(1, sizeof(ConnectionStates));
    msg_queue_sensors = xQueueCreate(1, sizeof(nav_msgs::ros_p2os_data_t));

    int32_t size_base = 1024;

    xTaskCreatePinnedToCore(task_bluepill_code, "TaskBluepill", size_base * 10, NULL, 1, &TaskBluepill, 0);

    xTaskCreatePinnedToCore(task_p2os_code, "TaskP2OS", size_base * 4, NULL, 1, &TaskP2OS, 1);

    debug_serial.println("Ready!");
}

void task_bluepill_code(void* pvParameters) {
    ConnectionStates          current_connected_state = NOT_CONNECTED;
    bool                      need_send_connect_msg = false;
    geometry_msgs::Twist      current_vel;
    nav_msgs::ros_p2os_data_t data_from_p2os;

    uint32_t connection_time = millis();
    uint32_t current_loop_time_bluepill;

    while (true) {
        current_loop_time_bluepill = millis();
        bluepill_comm->loop();

        if (current_connected_state != bluepill_comm->is_bluepill_connected() &&
            ((millis() - connection_time) > 1000)) {
            switch (bluepill_comm->is_bluepill_connected()) {
                case CONNECTED:
                    current_connected_state = CONNECTED;
                    need_send_connect_msg = true;
                    break;
                case NOT_CONNECTED:
                    current_connected_state = NOT_CONNECTED;
                    need_send_connect_msg = true;
                    break;
                default:
                    current_connected_state = CONNECTED;
                    // debug_serial.println("Error: Cry! - switch (bluepill_comm->is_bluepill_connected())");
                    break;
            }

            if (need_send_connect_msg && msg_queue_conn_command != 0) {
                if (xQueueSend(msg_queue_conn_command, &current_connected_state, portMAX_DELAY) == pdTRUE) {
                    need_send_connect_msg = false;
                }
            }

            bluepill_comm->send_bluepill_connection(current_connected_state);
        }

        current_vel = bluepill_comm->get_velocity();

        if (xQueueOverwrite(msg_queue_vel, &current_vel) == pdFALSE) {
            debug_serial.println("BP: Failed to send the data");
        }

        if (msg_queue_sensors != 0) {
            if (xQueueReceive(msg_queue_sensors, &data_from_p2os, (1 / portTICK_PERIOD_MS)) == pdTRUE) {
                bluepill_comm->update_p2dx_data(data_from_p2os);
            }
        }

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        // debug_serial.printf("BP: current_loop_time: %ld \n",  (millis() - current_loop_time_bluepill));
    }
}

void task_p2os_code(void* pvParameters) {
    nav_msgs::ros_p2os_data_t msg_p2os_sensors;

    bool                  is_connected_p2os = false;
    uint32_t              current_loop_time_p2os;
    uint32_t              last_time_motor_state = 0;
    ConnectionStates      connection_command_p2os;
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
                if (xQueueReceive(msg_queue_vel, &msg_p2os_vel, (10 / portTICK_PERIOD_MS)) == pdTRUE) {
                    p2os->set_vel(&msg_p2os_vel);
                }
            }

            msg_p2os_sensors = p2os->get_p2dx_data();
            if (xQueueOverwrite(msg_queue_sensors, &msg_p2os_sensors) == pdFALSE) {
                debug_serial.println("P2OS: Failed to send the  sensors P2OS data");
            }

            if (millis() - last_time_motor_state > 100) {
                msg_motor_state.state = 1;
                p2os->set_motor_state(&msg_motor_state);
                last_time_motor_state = millis();
            }
        }

        // debug_serial.printf("P2OS: current_loop_time: %ld \n",  (millis() - current_loop_time_p2os));
    }
}

void loop() { }
