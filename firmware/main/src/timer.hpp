#pragma once
#include <Arduino.h>

typedef void (*type_timer_interrupt_function_pointer)();

class HardwareTimer
{
    private:
    hw_timer_t * _p_timer = NULL;
    uint32_t _delay = 0;
    uint8_t _prescaler = 80;
    type_timer_interrupt_function_pointer _p_interrupt_function = NULL;
    uint8_t _timer_idx;
    bool _count_up = true;
    bool _rising_edge = true;
    bool _enable = true;
    bool _auto_reload = true;

    public:
    void setup(uint8_t timer_idx, type_timer_interrupt_function_pointer p_interrupt_function, bool autoreload)
    {
        _timer_idx = timer_idx;
        _p_interrupt_function = p_interrupt_function;
        _auto_reload = autoreload;

        _p_timer = timerBegin(timer_idx, _prescaler, _count_up);
        timerAttachInterrupt(_p_timer, p_interrupt_function, _rising_edge);
        timerSetAutoReload(_p_timer, _auto_reload);
    }

    void enable()
    {
        timerAlarmEnable(_p_timer);
    }

    void disable()
    {
        timerAlarmDisable(_p_timer);
    }

    void setDelay(uint32_t delay)
    {
        _delay = delay;
        timerAlarmWrite(_p_timer, _delay, _auto_reload);
    }

    void start()
    {
        timerStart(_p_timer);
    }

    void stop()
    {
        timerStop(_p_timer);
    }

    void restart()
    {
        timerRestart(_p_timer);
    }

    hw_timer_t * getPTimer()
    {
        return _p_timer;
    }
};

/*************************************************************************************************/

class timerDataReader;
class timerDataWriter;

class MultitaskTimerData
{
    friend class MultitaskTimer;

    private:
    unsigned long _previous_time;
    unsigned long _current_time = 0;
    unsigned long _period = 0;
    bool _auto_reload = false;
    bool _timer_active = false;

    SemaphoreHandle_t _semaphore;
    public:
    MultitaskTimerData()
    {
        _semaphore = xSemaphoreCreateMutex();
    }
};

class MultitaskTimer
{
    private:
    MultitaskTimerData &_data;
    SemaphoreHandle_t _semaphore;
    public:
    MultitaskTimer(MultitaskTimerData &data): _data(data)
    {
        _semaphore = _data._semaphore;
    }

    bool periodElapsed()
    {
        unsigned long current_time = micros();
        if(_data._timer_active && (current_time - _data._previous_time >= _data._period))
        {
            if(_data._auto_reload)
            {
                xSemaphoreTake(_semaphore, portMAX_DELAY);
                _data._previous_time = current_time;
                _data._timer_active = true;
                xSemaphoreGive(_semaphore);
            }
            else
            {
                xSemaphoreTake(_semaphore, portMAX_DELAY);
                _data._timer_active = false;
                xSemaphoreGive(_semaphore);
            }
            return true;
        }
        else return false;
    }

    void start()
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _data._timer_active = true;
        xSemaphoreGive(_semaphore);
    }

    void stop()
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _data._timer_active = false;
        xSemaphoreGive(_semaphore);
    }

    void setPeriod(unsigned long period)
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _data._period = period;
        xSemaphoreGive(_semaphore);
    }

    void setAutoReload(bool auto_reload)
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _data._auto_reload = auto_reload;
        xSemaphoreGive(_semaphore);
    }

    void reset()
    {
        xSemaphoreTake(_semaphore, portMAX_DELAY);
        _data._previous_time = micros();
        xSemaphoreGive(_semaphore);
    }
};

/*************************************************************************************************/

class SoftwareTimer
{
    private:
    unsigned long _previous_time;
    unsigned long _current_time = 0;
    unsigned long _period = 0;
    bool _auto_reload = false;
    bool _timer_active = false;
    public:
    
    void setup(unsigned long period, bool auto_reload)
    {
        _period = period;
        _auto_reload = auto_reload;
    }
    void start()
    {
        _timer_active = true;
    }

    void stop()
    {
        _timer_active = false;
    }

    void setPeriod(unsigned long period)
    {
        _period = period;
    }

    void setAutoReload(bool auto_reload)
    {
        _auto_reload = auto_reload;
    }

    void reset()
    {
        _previous_time = micros();
    }

    bool periodElapsed()
    {
        _current_time = micros();
        if(_current_time - _previous_time >= _period)
        {
            _previous_time = _current_time;
            return true;
        }
        else return false;
    }

        bool periodElapsed(unsigned long period)
    {
        _period = period;
        _current_time = micros();
        if(_current_time - _previous_time >= _period)
        {
            _previous_time = _current_time;
            return true;
        }
        else return false;
    }
};

/*************************************************************************************************/

void printLoopFrequency()
{
    const uint16_t _time = 1000;
    unsigned long _current_time = millis();
    static unsigned long _previous_time = 0;
    static uint32_t _frequency = 0; 
    if(_current_time - _previous_time >= _time)
    {
        Serial.println("Loop frequency:" + String(_frequency));
        _frequency = 0;
        _previous_time = _current_time;
    }
    _frequency += 1;
}