# SousVide-Controller
Used to program a ESP32 microcontroller to act as the control unit of a PID controlled Sous Vide. Built to be compatible with a mobile application.

The code for the microcontroller consists of:
a main file in which the BLE server is built and managed, and peripherals are interfaced (water level sensor, and enabling water pump relays), 
a PID header for adjusting the temperacture predictively, 
a header to interface with a MAX31865 rtd converter using SPI, 
a header to interface with a MCP4822 Digital to Analog Converter using SPI (used to control a relay to enable a heating element)
