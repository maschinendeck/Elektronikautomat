# IR Sensor Board

Sensor board for measuring if there are any items left in a vending machine slot. Uses cheap TCRT5000 sensors. CAD Software used is EAGLE.

The IR LEDs of the sensors are driven by a serial shift register input, the combined analog output signal of all phototransistors is provided as an analog output with an internal pulldown resistor (configurable via pot).