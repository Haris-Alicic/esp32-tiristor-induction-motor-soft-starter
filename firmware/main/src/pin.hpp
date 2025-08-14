#ifndef PIN_HPP
#define PIN_HPP

#include <Arduino.h>

class Pin
{
    private:
    uint8_t _pin;
    public:
    void setup(uint8_t pin, uint8_t mode, uint8_t value)
    {
        _pin = pin;
        pinMode(_pin, mode);
    }

    void write(uint8_t value)
    {
        digitalWrite(_pin, value);
    }

    uint8_t read(void)
    {
        return digitalRead(_pin);
    }

    void toggle(void)
    {
        digitalWrite(_pin, !digitalRead(_pin));
    }

    void setHigh(void)
    {
        digitalWrite(_pin, HIGH);
    }

    void setLow(void)
    {
        digitalWrite(_pin, LOW);
    }

    void setMode(uint8_t mode)
    {
        pinMode(_pin, mode);
    }
};

#endif //PIN_HPP