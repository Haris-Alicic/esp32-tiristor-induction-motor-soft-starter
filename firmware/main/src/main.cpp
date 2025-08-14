#include <Arduino.h>
#include "tasks.hpp"
#include "interrupts.hpp"
#include "timer.hpp"
#include "pin.hpp"
#include "sensors.hpp"
#include "multitask.hpp"
#include "pid_controller.hpp"

/*************************************************************************************************/
//Global variables
//GPIO pins
const uint8_t voltage_amplitude_pin = 35;
const uint8_t voltage_zero_crossing_pin = 17;
const uint8_t current_amplitude_pin = 34;
const uint8_t current_zero_crossing_pin = 18;

const uint8_t protection_pin = 25;
const uint8_t triac_1_pin = 19;
const uint8_t triac_2_pin = 21;

const uint8_t test_pin = 26;

//TIMERS
//We use timer 2 and 3 because they are not used
const uint8_t timer2 = 2;
const uint8_t timer3 = 3;

/*************************************************************************************************/
/*************************************************************************************************/
//Pin objects
Pin protectionPin;
Pin testPin;

//Interrupt
//Interrupt objects
Interrupt voltageZeroCrossingInterruptObject;
Interrupt currentZeroCrossingInterruptObject;

//Interrupt function prototypes
void IRAM_ATTR voltageZeroCrossingGpioInterruptFunction(void);
void IRAM_ATTR currentZeroCrossingGpioInterruptFunction(void);

/*************************************************************************************************/
//Timers
//Timer objects
HardwareTimer triacTimerInterruptObject;
HardwareTimer measurementTimerInterruptObject;

//Timer function prototypes
void IRAM_ATTR triacTimerInterruptFunction(void);
void IRAM_ATTR measurementTimerInterruptFunction(void);

/*************************************************************************************************/
//Tasks
//Task objects
Tasks triacTaskObject;
Tasks measurmementTaskObject;
Tasks pidTaskObject;

//Task function prototypes
void triacTaskFunction(void* parameter);
void measurementTaskFunction(void* parameter);
void pidTaskFunction(void* parameter);

/*************************************************************************************************/
//Flags
FlagPole voltageZeroCrossingFlagPole;
Flag voltageZeroCrossingInterruptFlag(voltageZeroCrossingFlagPole);

FlagPole currentZeroCrossingFlagPole;
Flag currentZeroCrossingInterruptFlag(currentZeroCrossingFlagPole);

FlagPole triacTimerFlagPole;
Flag triacTimerFlag(triacTimerFlagPole);

FlagPole measurementTimerFlagPole;
Flag measurementTimerFlag(measurementTimerFlagPole);

/*************************************************************************************************/
//Data sharing objects
DataStore<uint16_t> rmsVoltageValueDataStore;
DataReader<uint16_t> rmsVoltageValueDataReader(rmsVoltageValueDataStore);

DataStore<uint16_t> averageRmsVoltageValueDataStore;
DataReader<uint16_t> averageRmsVoltageValueDataReader(averageRmsVoltageValueDataStore);

DataStore<uint16_t> rmsCurrentValueDataStore;
DataReader<uint16_t> rmsCurrentValueDataReader(rmsCurrentValueDataStore);

DataStore<uint16_t> averageRmsCurrentValueDataStore;
DataReader<uint16_t> averageRmsCurrentValueDataReader(averageRmsCurrentValueDataStore);

DataStore<uint16_t> pidOutputDataStore;
DataReader<uint16_t> pidOutputDataReader(pidOutputDataStore);

DataStore<uint16_t> pidInputDataStore;
DataWriter<uint16_t> pidInputDataWriter(pidInputDataStore);

DataStore<uint16_t> pidOutputSetDataStore;
DataWriter<uint16_t> pidOutputSetDataWriter(pidOutputSetDataStore);

FlagPole pidOutputSetFlagPole;
Flag pidOutputSetFlag(pidOutputSetFlagPole);

/*************************************************************************************************/
/*************************************************************************************************/
//Arduino setup and loop functions
void setup()
{
  /*********************************************/
  //Pin setup
  protectionPin.setup(protection_pin, OUTPUT, LOW);
  testPin.setup(test_pin, OUTPUT, LOW);

  //Interrupt setup
  voltageZeroCrossingInterruptObject.setup(voltage_zero_crossing_pin, voltageZeroCrossingGpioInterruptFunction, CHANGE);
  currentZeroCrossingInterruptObject.setup(current_zero_crossing_pin, currentZeroCrossingGpioInterruptFunction, CHANGE);
  //Timer setup
  triacTimerInterruptObject.setup(timer3, triacTimerInterruptFunction, false);
  triacTimerInterruptObject.setDelay(9500);

  measurementTimerInterruptObject.setup(timer2, measurementTimerInterruptFunction, true);
  measurementTimerInterruptObject.setDelay(500);
  measurementTimerInterruptObject.enable();
  //Task setup
  triacTaskObject.setup(triacTaskFunction, "triacTask", 6, 0);
  measurmementTaskObject.setup(measurementTaskFunction, "measurementTask", 5, 0);
  pidTaskObject.setup(pidTaskFunction, "pidTask", 7, 0);
  triacTaskObject.suspend();
  /*********************************************/
  //Serial comunication setup
  Serial.begin(115200);
  Serial.println("Motor soft starter");
  pinMode(triac_1_pin, OUTPUT);
  digitalWrite(triac_1_pin, LOW);
}

void loop()
{
  delay(10000);
  digitalWrite(triac_1_pin, HIGH);
  Serial.println("Motor direct start");
  delay(10000);
  digitalWrite(triac_1_pin, LOW);
  delay(25000);
  pidInputDataWriter.write(1200);
  pidOutputSetFlag.set();
  pidOutputSetDataWriter.write(0);
  triacTaskObject.resume();
  Serial.println("Motor soft start");
  delay(25000);
  triacTaskObject.suspend();
  digitalWrite(triac_1_pin, LOW);
  Serial.println("Motor stop");
}

/*************************************************************************************************/
/*************************************************************************************************/
//GPIO interrupt functions
//Voltage zero crossing interrupt function
void IRAM_ATTR voltageZeroCrossingGpioInterruptFunction(void)
{
  voltageZeroCrossingInterruptFlag.set();
  BaseType_t voltage_zero_crossing_gpio_interrupt_notification = pdFALSE;
  vTaskNotifyGiveFromISR(triacTaskObject.getHandle(), &voltage_zero_crossing_gpio_interrupt_notification);
  vTaskNotifyGiveFromISR(measurmementTaskObject.getHandle(), &voltage_zero_crossing_gpio_interrupt_notification);
  vTaskNotifyGiveFromISR(pidTaskObject.getHandle(), &voltage_zero_crossing_gpio_interrupt_notification);
  portYIELD_FROM_ISR();
}

//Current zero crossing interrupt function
void IRAM_ATTR currentZeroCrossingGpioInterruptFunction(void)
{
  currentZeroCrossingInterruptFlag.set();
  BaseType_t current_zero_crossing_gpio_interrupt_notification = pdFALSE;
  vTaskNotifyGiveFromISR(measurmementTaskObject.getHandle(), &current_zero_crossing_gpio_interrupt_notification);
  portYIELD_FROM_ISR();
}

/*************************************************************************************************/
//Timer interrupt functions
//Triac timer interrupt function
void IRAM_ATTR triacTimerInterruptFunction(void)
{
  triacTimerFlag.set();
  BaseType_t triac_timer_interrupt_notification = pdFALSE;
  vTaskNotifyGiveFromISR(triacTaskObject.getHandle(), &triac_timer_interrupt_notification);
  portYIELD_FROM_ISR();
}

//Measurement timer interrupt function
void IRAM_ATTR measurementTimerInterruptFunction(void)
{
  measurementTimerFlag.set();
  BaseType_t measurement_timer_interrupt_notification = pdFALSE;
  vTaskNotifyGiveFromISR(measurmementTaskObject.getHandle(), &measurement_timer_interrupt_notification);
  portYIELD_FROM_ISR();
}

/*************************************************************************************************/
//Task functions
//Triac task function
void triacTaskFunction(void* parameter)
{
  Pin triac1Pin;
  Pin triac2Pin;

  triac1Pin.setup(triac_1_pin, OUTPUT, LOW);
  triac2Pin.setup(triac_2_pin, OUTPUT, LOW);

  Flag voltageZeroCrossingTaksFlag(voltageZeroCrossingFlagPole);
  Flag triacTimerTaskFlag(triacTimerFlagPole);

  DataReader<uint16_t> pidOutputTaskDataReader(pidOutputDataStore);
  
  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(voltageZeroCrossingTaksFlag.get())
    {
      triac1Pin.setLow();
      triacTimerInterruptObject.setDelay(pidOutputTaskDataReader.read());
      triacTimerInterruptObject.restart();
      triacTimerInterruptObject.enable();
    }
    else if(triacTimerTaskFlag.get())
    {
      triac1Pin.setHigh();
      triacTimerInterruptObject.disable();
    }
  }
}

//Measurement task function
void measurementTaskFunction(void* parameter)
{
  //Sensors
  Sensor voltageSensor;
  Sensor currentSensor;

  //Sensor setup
  voltageSensor.setup(voltage_amplitude_pin, voltage_zero_crossing_pin);
  currentSensor.setup(current_amplitude_pin, current_zero_crossing_pin);

  Flag measurementTimerTaskFlag(measurementTimerFlagPole);
  Flag voltageZeroCrossingTaksFlag(voltageZeroCrossingFlagPole);
  Flag currentZeroCrossingTaksFlag(currentZeroCrossingFlagPole);

  DataWriter<uint16_t> rmsVoltageValueTaskDataWriter(rmsVoltageValueDataStore);
  DataWriter<uint16_t> averageVoltageValueTaskDataWriter(averageRmsVoltageValueDataStore);
  DataWriter<uint16_t> rmsCurrentValueTaskDataWriter(rmsCurrentValueDataStore);
  DataWriter<uint16_t> averageCurrentValueTaskDataWriter(averageRmsCurrentValueDataStore);

  uint16_t voltage_zero_crossing_counter = 0;
  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(measurementTimerTaskFlag.get())
    {
      voltageSensor.sampleAndHold();
      currentSensor.sampleAndHold();
    }
    if(voltageZeroCrossingTaksFlag.get())
    {
      voltageSensor.rmsCalculate();
      rmsVoltageValueTaskDataWriter.write(voltageSensor.getRmsValue());
      voltage_zero_crossing_counter++;

      if(voltage_zero_crossing_counter == 50)
      {
        averageVoltageValueTaskDataWriter.write(voltageSensor.getAverageRmsValue());
        averageCurrentValueTaskDataWriter.write(currentSensor.getAverageRmsValue());
        voltage_zero_crossing_counter = 0;
      }
    }
    if(currentZeroCrossingTaksFlag.get())
    {
      currentSensor.rmsCalculate();
      rmsCurrentValueTaskDataWriter.write(currentSensor.getRmsValue());
    }
  }
}

//PID task function
void pidTaskFunction(void* parameter)
{
  Flag voltageZeroCrossingTaksFlag(voltageZeroCrossingFlagPole);
  Flag pidOutputSetTaskFlag(pidOutputSetFlagPole);

  DataReader<uint16_t> rmsCurrentValueTaskDataReader(rmsCurrentValueDataStore);
  DataReader<uint16_t> pidInputTaskDataReader(pidInputDataStore);
  DataReader<uint16_t> pidOutputSetDataReader(pidOutputSetDataStore);
  DataWriter<uint16_t> pidOutputTaskDataWriter(pidOutputDataStore);


  PID_Controller pidController;

  pidController.setupRange(9500, 0, 9500, true);
  pidController.setupCoefficients(100, 0, 0, 1000);
  
  while(true)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if(voltageZeroCrossingTaksFlag.get())
    {
      if(pidOutputSetFlag.get())
      {
        pidController.setOutputValue(pidOutputSetDataReader.read());
      }
      pidController.setDesiredValue(pidInputTaskDataReader.read());
      pidController.takeMeasuredValue((uint16_t)rmsCurrentValueTaskDataReader.read());
      pidController.calculate();
      pidOutputTaskDataWriter.write(pidController.giveOutputValue());
      //pidTaskDataWriter.write(2500);
    }
  }
}

