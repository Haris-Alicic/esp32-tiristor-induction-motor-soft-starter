#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <Arduino.h>

class PID_Controller
{
    private:
    uint16_t _scaling = 1;
    bool _invert = false;

    uint16_t _proportional_gain = 0.0;
    uint16_t _integral_gain = 0.0;
    uint16_t _derivative_gain = 0.0;

    int16_t _desired_value = 0.0;

    int16_t _measured_value = 0.0;
    bool _measured_value_updated = false;

    int16_t _error_value = 0.0;
    int16_t _previous_error_value = 0.0;

    int64_t _integral_value = 0.0;
    int16_t _derivative_value = 0.0;

    int16_t _corrective_value = 0.0;

    int16_t _output_value = 0.0;
    int16_t _min_output_value = 0.0;
    int16_t _max_output_value = 0.0;

    public:
    void setupRange(int16_t initial_value ,int16_t min, int16_t max, bool invert = false)
    {
        _output_value = initial_value;
        _min_output_value = min;
        _max_output_value = max;
        _invert = invert;
    }
    void setupCoefficients(uint16_t proportional_gain, uint16_t integral_gain, uint16_t derivative_gain, uint16_t scaling)
    {
        _scaling = scaling;
        _proportional_gain = proportional_gain;
        _integral_gain = integral_gain;
        _derivative_gain = derivative_gain;
    }

    void setDesiredValue(int16_t desired_value)
    {
        _desired_value = desired_value;
    }

    void takeMeasuredValue(int16_t measured_value)
    {
        _measured_value_updated = true;
        _measured_value = measured_value;
    }
    
    void calculate()
    {
        if(_measured_value_updated)
        {
            _error_value = _desired_value - _measured_value;
            _integral_value += _error_value;
            _derivative_value = _error_value - _previous_error_value;
            _previous_error_value = _error_value;

            _corrective_value = (_proportional_gain * _error_value 
                                + _integral_gain * _integral_value 
                                + _derivative_gain * _derivative_value)/_scaling;
            _output_value += _corrective_value;
            
            if(_output_value >= _max_output_value)
                _output_value = _max_output_value;
            else if(_output_value <= _min_output_value)
                _output_value = _min_output_value;
            _measured_value_updated = false;
        }
    }

    int16_t giveOutputValue()
    {
        if(_invert)
            return _max_output_value - _output_value;
        else
            return _output_value;
    }

    void setOutputValue(int16_t output_value)
    {
        _output_value = output_value;
    }
};

#endif //PID_CONTROLLER_HPP