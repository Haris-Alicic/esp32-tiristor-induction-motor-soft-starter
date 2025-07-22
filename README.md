\# esp32-tiristor-induction-motor-soft-starter



> \*\*ESP32‑based soft‑starter for three‑phase induction motors\*\*  

> Uses a numeric PID controller to ramp and hold start‑up current via triac phase‑angle firing.



---



\## Overview



This project implements a soft‑starter for single‑phase/three‑phase induction motors using an ESP32.  

By monitoring line voltage and motor current via isolation transformers, the ESP32 computes real‑time RMS current and adjusts triac firing angle through an optocoupler, keeping inrush current at a user‑set limit via a PID loop running under FreeRTOS.



---



\## Features



\- \*\*Zero‑cross detection\*\* on both voltage and current channels  

\- \*\*Ideal rectifier\*\* stages to feed buffered 0–3.3 V analog to ESP32 ADC  

\- \*\*Real‑time RMS calculation\*\* (≈10 samples/s)  

\- \*\*Numeric PID control\*\* for current‑limited startup  

\- \*\*Triac‑phase control\*\* via optocoupler, synchronized to AC mains  

\- \*\*Dual‑core RTOS tasks\*\* with semaphores for edge‑timing and computation  

\- \*\*Over‑voltage protection\*\* on ESP32 pins (Zener + BJT clamp)



---



\## Hardware Architecture



VT1 / CT1: voltage/current isolation transformers



Zero‑cross detectors: polarity comparators→3.3 V digital edges



Ideal rectifiers: UA741 op‑amp circuits → 0–3.3 V analog RMS input



Triac drive: ESP32 GPIO → optocoupler → triac gate



OVP clamp: Zener + BJT protects GPIO from ±12 V op‑amp swings



Electrical Schematics in docs



Software Architecture

FreeRTOS tasks



ZX\_Task: high‑priority interrupt service on zero‑cross GPIO



ADC\_Task: periodic sampling of rectified signals



PID\_Task: compute RMS, update firing angle



Triac\_Task: schedule triac trigger via hardware timer



Inter‑task synchronization via semaphores and queues



PID loop



Setpoint = configured current limit



Feedback = latest RMS current



Output = phase‑angle delay (0…π/2)



PlatformIO project under firmware/



platformio.ini targets ESP32 Dev Module



src/main.cpp, helper modules in src/ \& include/



Configuration

All tunables are in src/main.cpp:



cpp

Copy

Edit

// PID gains (tune per motor / transformer)

float Kp = 0.1;

float Ki = 0.05;

float Kd = 0.01;



// Current setpoint \[A]

float current\_limit = 10.0;

Adjust these values and rebuild to match your motor’s rated start‑up current.
