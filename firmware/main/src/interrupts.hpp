#ifndef INTERRUPT_HPP
#define INTERRUPT_HPP
#include <Arduino.h>

class Interrupt
{
    private:
    using interrupt_function_pointer_type = void (*)();
    interrupt_function_pointer_type _p_interrupt_function = NULL;
    uint8_t _pin;
    uint8_t _mode;

    public:
    void setup(uint8_t pin, interrupt_function_pointer_type p_interrupt_function, int mode)
    {
        _pin = pin;
        _p_interrupt_function = p_interrupt_function;
        _mode = mode;
        pinMode(_pin, INPUT_PULLDOWN);
        attachInterrupt(digitalPinToInterrupt(_pin), _p_interrupt_function, mode);
    }
};

#endif // INTERRUPT_HPP