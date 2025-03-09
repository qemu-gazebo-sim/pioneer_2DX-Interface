#include "task_controller.hpp"
#include <ArduinoLog.h>

HardwareSerial debug_serial(0);  // define a Serial for UART0

void setup() {
    debug_serial.begin(9600);
    debug_serial.flush();

    Log.begin(LOG_LEVEL_TRACE, &debug_serial);

    TaskController* task_controller = new TaskController();

    task_controller->tasks_init();

    Log.infoln("Ready!");
}

void loop() { }