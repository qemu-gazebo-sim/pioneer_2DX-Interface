#ifndef _TASK_CONTROLLER_HPP_
#define _TASK_CONTROLLER_HPP_

#include <Arduino.h>

enum OperationalModes {
    BLUETOOTH_MODE,
    BLUEPILL_MODE,
    NOT_VALID_MODE
};

class TaskController {
private:
    /* Task definitions */
    TaskHandle_t TaskBluetooth;
    TaskHandle_t TaskBluepill;
    TaskHandle_t TaskP2OS;

    int32_t size_base;

    OperationalModes set_current_operational_mode();

public:
    TaskController();

    ~TaskController();

    void tasks_init();

    OperationalModes get_current_operational_mode();

protected:
    OperationalModes current_operational_mode;

    static void bluetooth_task(void* pvParameters);

    static void bluepill_task(void* pvParameters);

    static void p2os_task(void* pvParameters);
};

#endif  // _TASK_CONTROLLER_HPP_
