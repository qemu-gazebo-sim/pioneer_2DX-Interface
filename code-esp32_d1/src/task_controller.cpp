#include <Arduino.h>
#include <ArduinoLog.h>

#include <ps5Controller.h>
#include "task_controller.hpp"

#include "pinout_config.hpp"
#include <p2os.hpp>
#include "bluepill_comm.hpp"
#include "utils.hpp"

static const QueueHandle_t msg_queue_vel = xQueueCreate(1, sizeof(geometry_msgs::Twist));
static const QueueHandle_t msg_queue_conn_command = xQueueCreate(1, sizeof(ConnectionStates));
static const QueueHandle_t msg_queue_sensors = xQueueCreate(1, sizeof(nav_msgs::ros_p2os_data_t));

/* Private functions */
OperationalModes TaskController::set_current_operational_mode() {
    switch (digitalRead(MODE_1_PIN)) {
        case 0:
            this->current_operational_mode = BLUETOOTH_MODE;
            break;

        case 1:
            this->current_operational_mode = BLUEPILL_MODE;
            break;

        default:
            Log.errorln("TaskController::get_current_operational_mode() not valid mode");
            this->current_operational_mode = BLUETOOTH_MODE;
            break;
    }

    return this->current_operational_mode;
}

/* Public functions */
TaskController::TaskController() {
    pinMode(MODE_1_PIN, INPUT);
    // pinMode(MODE_2_PIN, INPUT);
    // pinMode(LED_CONN, OUTPUT);

    this->set_current_operational_mode();
    this->size_base = 1024;
}

TaskController::~TaskController() { }

void TaskController::tasks_init() {
    Log.traceln("TaskController::tasks_init()");
    switch (this->get_current_operational_mode()) {
        case BLUEPILL_MODE:
            Log.infoln("Starting on Bluepill mode");
            xTaskCreatePinnedToCore(
                this->bluepill_task, "TaskBluepill", this->size_base * 10, NULL, 1, &(this->TaskBluepill), 0
            );
            // digitalWrite(LED_CONN, LOW);
            break;

        case BLUETOOTH_MODE:
            Log.traceln("Starting on BT mode");
            xTaskCreatePinnedToCore(
                this->bluetooth_task, "TaskBluetooth", this->size_base * 4, NULL, 1, &(this->TaskBluetooth), 0
            );
            // digitalWrite(LED_CONN, HIGH);
            break;

        default:
            Log.errorln("TaskController::TaskController() init error");
            return;
    }

    xTaskCreatePinnedToCore(this->p2os_task, "TaskP2OS", this->size_base * 4, NULL, 1, &(this->TaskP2OS), 1);
}

OperationalModes TaskController::get_current_operational_mode() {
    return this->current_operational_mode;
}

/* Protected functions */
void TaskController::bluetooth_task(void* pvParameters) {
    Log.traceln("Starting on BT task");

    ps5.begin(PS5_CONTROLLER_MAC_ADDRESS);  // replace with MAC address of your controller
    uint32_t             current_loop_time_bt;
    geometry_msgs::Twist msg_vel;

    while (true) {
        ConnectionStates is_connected_bt = NOT_CONNECTED;
        uint16_t         current_r2_val = 0;
        uint16_t         current_l2_val = 0;
        int16_t          current_rs_x_val = 0;

        ConnectionStates connection_command;
        bool             need_send_connect_msg = false;

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
                current_r2_val = scale(ps5.R2Value(), 0, 255, 0, 500);
                current_l2_val = scale(ps5.L2Value(), 0, 255, 0, 500);
                current_rs_x_val = (-1) * scale(ps5.RStickX(), -128, 128, -170, 170);

                msg_vel.linear.x = double(double(current_r2_val - current_l2_val) / 1000);
                msg_vel.angular.z = double(current_rs_x_val) / 100;

                if (xQueueOverwrite(msg_queue_vel, &msg_vel) == pdFALSE) {
                    Log.infoln("Failed to send the data");
                }
            }

            Log.verboseln("BT: current_loop_time: %l", (millis() - current_loop_time_bt));
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TaskController::bluepill_task(void* pvParameters) {
    ConnectionStates          current_connected_state = NOT_CONNECTED;
    bool                      need_send_connect_msg = false;
    geometry_msgs::Twist      current_vel;
    nav_msgs::ros_p2os_data_t data_from_p2os;

    BluepillCommunication* bluepill_comm = new BluepillCommunication();

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
                    // Log.infoln("Error: Cry! - switch (bluepill_comm->is_bluepill_connected())");
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
            Log.infoln("BP: Failed to send the data");
        }

        if (msg_queue_sensors != 0) {
            if (xQueueReceive(msg_queue_sensors, &data_from_p2os, (1 / portTICK_PERIOD_MS)) == pdTRUE) {
                bluepill_comm->update_p2dx_data(data_from_p2os);
            }
        }

        // vTaskDelay(10 / portTICK_PERIOD_MS);
        Log.verboseln("BP: current_loop_time: %ld", (millis() - current_loop_time_bluepill));
    }
}

void TaskController::p2os_task(void* pvParameters) {
    Log.traceln("Starting on p2os task");
    HardwareSerial pioneer_serial(2);
    P2OS*          p2os = new P2OS(pioneer_serial);

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
                        if (is_connected_p2os == 1) {
                            Log.infoln("P2OS: setup!");
                        } else {
                            Log.infoln("P2OS: failed to setup!");
                        }
                    } else if (connection_command_p2os == NOT_CONNECTED) {
                        is_connected_p2os = p2os->shutdown();
                        Log.infoln("P2OS: shutdown!");
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

            msg_p2os_sensors = p2os->get_p2dx_data();
            if (xQueueOverwrite(msg_queue_sensors, &msg_p2os_sensors) == pdFALSE) {
                Log.errorln("P2OS: Failed to send the sensors P2OS data");
            }

            if (millis() - last_time_motor_state > 100) {
                msg_motor_state.state = 1;
                p2os->set_motor_state(&msg_motor_state);
                last_time_motor_state = millis();
            }

            Log.verboseln("P2OS: current_loop_time: %ld", (millis() - current_loop_time_p2os));
        }
    }
}
