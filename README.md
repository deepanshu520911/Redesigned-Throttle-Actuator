# Redesigned Throttle actuator

## Problem
Throttle actuators are manufactured using electromechanical design and because of its electromechanical main sensor module design, there are four contact brushes which are in contact with sensor module plate, made of carbon ceramic resistive material(works similar to a potentiometer, actually 2 pots are coupled with each other and both works opposite to each other) and moving brushes are attached with central moving shaft which directly attached with valve and due to continuosly movement between brush and carbon ceramic resistive coating, its coating starts to erroded and ultimatelty gets failed over time due to wear and tear and thus complete part is failed and lots of money gets wasted. Repairing of this is not possible as no repairing kits. Spare parts are also not available.

## Solution
I converted this electromechanical sensor design to electronic contact less design using a sensor, a microcontroller and a DAC. The sensor is an 14bit magnetic position sensor which gives me actual position of shaft. A diametric magnet is attached to central shaft whose magnetic field is used by sensor to measure its position. Now this angle position is read by a 8bit microcontoller using SPI protocol at a max speed of 10 MHZ which sensor supports as well. Next data is modified accordingly and this data is now ready for a DAC to produce output for ECU to read. A dual channel DAC(Digital to analog convertor) is used as throttle produces two opposite voltage analog signals and as ECU uses a 12bit ADC unit to read throttle signals so, I used 12 bit DAC for this to work smoothly. Intial calibiration is done automatically during first start after code uploading by forcing throttle valve to its complete close positon and sensor angle data is read by microcontroller to save this as a zero position and this calibiration data is stored in EEPROM and used after every new start as a zero angle position. A PCB design is made in PROTEUS with all essential components required like resistors, capacitors, etc and a prototype is made and pasted in place of failed sensor module plate after soldering all componets and checking for any shorts. Also central shaft needs a set of bearings which are replaced before final fitment. 

![alt text](https://github.com/deepanshu520911/Throttle/blob/main/Picture1.png)

## Impact
Due to no repair option was available before above designed product, new throttle costing around **26k** was required on every failure. Now on every failure, fixing a failed part using above design costs **2.1k only**(including bearings, PCB, IC's etc.). Also, no future wear and tear because of its contactless design.

### Components Specification
Magnetic Position Sensor - AS5047D/AS5047P/AS5147P/AS5147

Microcontroller          - ATTINY85/ATTINY13A

DAC                      - DAC7562/DAC7563

## PCB DESIGN 
![Check here](https://github.com/deepanshu520911/Throttle/blob/main/PCB%20THROTTLE.pdf)

## Code
![Check here](https://github.com/deepanshu520911/Throttle/blob/main/AUTO-CALIBIRATION_MOSFET.c)


