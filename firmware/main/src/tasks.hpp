#ifndef TASKS_HPP
#define TASKS_HPP
#include <Arduino.h>

class Tasks
{
    private:
    TaskFunction_t _Function_t;
    TaskHandle_t _Handle_t;
    const char* _Name;
    uint32_t _StackDepth;
    UBaseType_t _Priority;
    BaseType_t _Core;
    public:
    void setup(TaskFunction_t functionPrototype, const char* name, UBaseType_t priority, BaseType_t cpu_core)
    {
        _Function_t = functionPrototype;
        _Name = name;
        xTaskCreatePinnedToCore(_Function_t, name, 10000, NULL, priority, &_Handle_t, cpu_core);
    }

    void suspend(void)
    {
        vTaskSuspend(_Handle_t);
    }

    void resume(void)
    {
        vTaskResume(_Handle_t);
    }

    void deleteTask(void)
    {
        vTaskDelete(_Handle_t);
    }

    TaskHandle_t getHandle(void)
    {
        return _Handle_t;
    }
};

#endif // TASKS_HPP