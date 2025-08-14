#pragma once

#include "Arduino.h"
#include <queue>

//Enums
enum Polarity{POSITIVE, NEGATIVE};

//Classes
class Sensor
{
  private:
  u_int8_t _amplitude_pin = 0;
  u_int8_t _zero_crossing_pin = 0;
  volatile bool _zero_crossing = false;
  Polarity _polarity;

  uint16_t _sample = 0;
  std::queue<uint16_t> _sample_queue;
  volatile uint16_t _number_of_samples = 0;

  volatile uint64_t _rms_sum = 0;
  volatile uint16_t _rms_value = 0;

  volatile uint16_t _average_rms_value = 0;
  volatile uint64_t _average_rms_sum = 0;
  volatile uint16_t _average_rms_counter = 0;

  volatile uint16_t _average_number_of_samples_per_cycle = 0;
  volatile uint64_t _average_number_of_samples_per_cycle_sum = 0;

  public:
  void setup(uint8_t amplitude_pin, uint8_t zero_crossing_pin)
  {
    _amplitude_pin = amplitude_pin;
    _zero_crossing_pin = zero_crossing_pin;

    pinMode(_amplitude_pin, INPUT);
    pinMode(_zero_crossing_pin, INPUT);
  }

  void measurePolarity(void)
  {
    if(digitalRead(_zero_crossing_pin)) _polarity = POSITIVE;
    else _polarity = NEGATIVE;
  }

  uint16_t sample(void)
  {
    _sample = analogRead(_amplitude_pin);
    return _sample;
  }

  void sampleAndHold()
  {
    _sample_queue.push(sample());
  }

  void rmsCalculate()
  {
    _number_of_samples = _sample_queue.size();
    if(_number_of_samples == 0) return; //Avoid division by zero (NaN)

    _rms_sum = 0.0;
    uint16_t sample;
    while(!_sample_queue.empty())
    {
      sample = _sample_queue.front();
      _rms_sum += sample * sample;
      _sample_queue.pop();
    }
    _rms_value = (uint16_t)sqrt(_rms_sum / _number_of_samples);
    _average_rms_sum += _rms_value;
    _average_rms_counter++;
  }

  uint16_t getNumberOfSamples()
  {
    return _number_of_samples;
  }

  uint16_t getRmsValue()
  {
    return _rms_value;
  }

  uint16_t getAverageRmsValue()
  {
    if(_average_rms_counter == 0) return 0; //Avoid division by zero (NaN
    _average_rms_value = _average_rms_sum / _average_rms_counter;
    _average_rms_sum = 0;
    _average_rms_counter = 0;
    return _average_rms_value;
  }
};