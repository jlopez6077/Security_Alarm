# Security_Alarm
 
## Goal 
The goal of the project was to expand my knowledge of microcontrollers and it's hardware components. Creating a simple security system with a microcontroller can provide a cost-effective solution for protecting your house. The system can be designed to detect unauthorized entry through doors or windows and sound an alarm. 

## Description
Entering the correct passcode will alarm the system. If the ultrasonic sensor detects movement then it sounds the buzzer. To turn off the buzzer, you need to enter the correct passcode again. 

## Components 
* STM NUCLEO-767ZI
* 4x4 Matrix Membrane Keypad
* 74HC595 IC (Shifting register)
* 4-DIGIT 7-SEG Display
* Ultrasonic Sensor HC-SRO4
* LED x 2
* Active Buzzer 
* 330 ohm resistor x 6
* 1K ohm resistor x 4

## Diagram
![Security Alarm Pinout](https://user-images.githubusercontent.com/116536807/231343331-bf1ff437-98d9-417c-a7eb-0138906a6c54.png)

## Finite State Machine
![FSM](https://user-images.githubusercontent.com/116536807/231346506-0387c68e-f7ab-43f4-898b-066d652bbad4.png)

## How to Set it Up 
I've left you with the project code, diagram, and finite state machine. If you want explaintions on individual components please check out my STM32_Components_Library repository where I go into great detail.

Link: https://github.com/jlopez6077/STM32_Components_Library
 
