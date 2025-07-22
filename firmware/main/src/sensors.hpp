#pragma once

#include "Arduino.h"
#include <queue>

typedef void (*type_sensor_interrupt_function_pointer)();

//Enums
enum Polarity{POSITIVE, NEGATIVE};

//Classes
class Sensor
{
  private:
  u_int8_t _amplitude_pin = 0;
  u_int8_t _zero_crossing_pin = 0;
  type_sensor_interrupt_function_pointer _p_sensor_interrupt_function = NULL;
  volatile bool _zero_crossing = false;
  Polarity _polarity;

  volatile uint16_t _sample = 0;
  std::queue<uint16_t> _sample_queue;
  volatile uint16_t _number_of_samples = 0;

  volatile uint32_t _rms_sample = 0;
  volatile uint64_t _rms_sum = 0;
  volatile double _rms_average = 0;
  volatile double _rms_value = 0;

  public:
  void setup(uint8_t amplitude_pin, uint8_t zero_crossing_pin, type_sensor_interrupt_function_pointer p_sensor_interrupt_function)
  {
    _amplitude_pin = amplitude_pin;
    _zero_crossing_pin = zero_crossing_pin;
    _p_sensor_interrupt_function = p_sensor_interrupt_function;

    pinMode(_amplitude_pin, INPUT);
    pinMode(_zero_crossing_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_zero_crossing_pin), _p_sensor_interrupt_function, CHANGE);
  }

  void measurePolarity(void)
  {
    if(digitalRead(_zero_crossing_pin)) _polarity = POSITIVE;
    else _polarity = NEGATIVE;
  }

   void sample(void)
  {
    _sample_queue.push(analogRead(_amplitude_pin));
  }

  void rmsCalculate()
  {
    _number_of_samples = _sample_queue.size();
    _rms_sum = 0.0;
    uint16_t sample;
    while(!_sample_queue.empty())
    {
      sample = _sample_queue.front();
      _rms_sum += sample * sample;
      _sample_queue.pop();
    }
    _rms_value = sqrt((double)_rms_sum / (double)_number_of_samples);
  }

  uint16_t getNumberOfSamples()
  {
    return _number_of_samples;
  }

  double rmsValue()
  {
    return _rms_value;
  }
};