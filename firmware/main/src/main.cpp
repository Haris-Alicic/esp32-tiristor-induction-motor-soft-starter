#include <Arduino.h>
#include "sensors.hpp"
#include "timer.hpp"

//Hardware pin names
const uint8_t protection_pin = 25;
const uint8_t triac_1_pin = 19;

const uint8_t voltage_amplitude_pin = 35;
const uint8_t voltage_zero_cross_pin = 17;
const uint8_t current_amplitude_pin = 34;
const uint8_t current_zero_cross_pin = 18;

//TIMERS
//We use timer 2 and 3 because they are not used
const uint8_t timer2 = 2;
const uint8_t timer3 = 3;

//Interrupt declarations
void IRAM_ATTR voltageZeroCrossingInterrupt();
void IRAM_ATTR currentZeroCrossingInterrupt();
void IRAM_ATTR triacInterrupt();
void IRAM_ATTR measurementInterrupt();

//Interrupt flags
volatile bool voltage_zero_crossing_interrupt[2] = {false, false };
volatile bool current_zero_crossing_interrupt = false;
volatile bool triac_interrupt = false;
volatile bool measurement_interrupt = false;

//Tasks
// Define task handles
TaskHandle_t triacTaskHandle;
TaskHandle_t measurementTaskHandle;
TaskHandle_t pidControllerTaskHandle;

// Function prototypes for task functions
void triacTask(void *pvParameters);
void measurementTask(void *pvParameters);
void pidControllerTask(void *pvParameters);

//Task queues
QueueHandle_t voltage_queue;
QueueHandle_t current_queue;

//Objects
Sensor Voltage;
Sensor Current;
//CurrentSensor Current;
Timer Timer2;
Timer Timer3;
SoftwareTimer SoftwareTimer1;
SoftwareTimer SoftwareTimer2;

void setup()
{
  //Serial comunication
  Serial.begin(115200);
  Serial.println("CPU Frequency: " + String(getCpuFrequencyMhz()));

  //Measurement and sensors
  Voltage.setup(voltage_amplitude_pin, voltage_zero_cross_pin, voltageZeroCrossingInterrupt);
  Current.setup(current_amplitude_pin, current_zero_cross_pin, currentZeroCrossingInterrupt);
  
  //Hardware timers
  Timer2.setup(timer2, measurementInterrupt, true);
  Timer2.setDelay(800);
  Timer2.enable();

  Timer3.setup(timer3, triacInterrupt, false);
  Timer3.setDelay(8900);
  Timer3.enable();

  //Software timers
  SoftwareTimer1.setup(1000000, true);
  //SoftwareTimer2.setup(1000000, true);
  
  //Pin setup
  pinMode(protection_pin, OUTPUT);
  pinMode(triac_1_pin, OUTPUT);

  digitalWrite(triac_1_pin, LOW);
  digitalWrite(protection_pin, LOW);

  //Tasks creation
  xTaskCreatePinnedToCore(triacTask, "TriacTask", 10000, NULL, 6, &triacTaskHandle, 0);
  xTaskCreatePinnedToCore(measurementTask, "MeasurementTask", 10000, NULL, 5, &measurementTaskHandle, 0);
  xTaskCreatePinnedToCore(pidControllerTask, "PIDControllerTask", 10000, NULL, 7, &pidControllerTaskHandle, 0);

  //Queue creation
  voltage_queue = xQueueCreate(1, sizeof(double));
  current_queue = xQueueCreate(1, sizeof(double));
}

uint16_t timer_delay = 10000;

void loop()
{
  if(SoftwareTimer1.periodElapsed())
  {
    double rms_voltage = 0.0;
    double rms_current = 0.0;
    xQueueReceive(voltage_queue, &rms_voltage, portMAX_DELAY);
    xQueueReceive(current_queue, &rms_current, portMAX_DELAY);
    Serial.println("Voltage: " + String(rms_voltage));
    Serial.println("Current: " + String(rms_current));
  }
  printLoopFrequency();
}

//*************************************************************************************************
//Interrupt definitions
void IRAM_ATTR voltageZeroCrossingInterrupt(void)
{
  voltage_zero_crossing_interrupt[0] = true;
  voltage_zero_crossing_interrupt[1] = true;
  vTaskResume(triacTaskHandle);
  vTaskResume(measurementTaskHandle);
}

void IRAM_ATTR currentZeroCrossingInterrupt(void)
{
  current_zero_crossing_interrupt = true;
  vTaskResume(measurementTaskHandle);
}

void IRAM_ATTR triacInterrupt()
{
  triac_interrupt = true;
  vTaskResume(triacTaskHandle);
}

void IRAM_ATTR measurementInterrupt()
{
  measurement_interrupt = true;
  vTaskResume(measurementTaskHandle);
}

//*************************************************************************************************
//Task defininitions
// Task function declarations
void triacTask(void *pvParameters)
{
  while(true) 
  {
    if(voltage_zero_crossing_interrupt[0])
    {
      digitalWrite(triac_1_pin, LOW);
      Timer3.restart();
      Timer3.enable();
      voltage_zero_crossing_interrupt[0] = false;
    }

    if(triac_interrupt)
    {
      digitalWrite(triac_1_pin, HIGH);
      Timer3.disable();
      triac_interrupt = false;
    }
    vTaskSuspend(NULL);
  }
}

void measurementTask(void *pvParameters)
{
  double rms_voltage = 0.0;
  double rms_current = 0.0;
  while(true) 
  {
    if(voltage_zero_crossing_interrupt[1])
    {
      Voltage.rmsCalculate();
      rms_voltage = Voltage.rmsValue();
      xQueueOverwrite(voltage_queue, &rms_voltage);
      voltage_zero_crossing_interrupt[1] = false;
    }

    if(current_zero_crossing_interrupt)
    {
      Current.rmsCalculate();
      rms_current = Current.rmsValue();
      xQueueOverwrite(current_queue, &rms_current);
      current_zero_crossing_interrupt = false;
    }

    if(measurement_interrupt)
    {
      Voltage.sample();
      Current.sample();
      measurement_interrupt = false;
    }
    vTaskSuspend(NULL);
  }
}

void pidControllerTask(void *pvParameters)
{
  while(true) 
  {
    
    vTaskSuspend(NULL);
  }
}