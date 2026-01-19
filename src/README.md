# WeatherStation

Weather Shield Example
By: Nathan Seidle
SparkFun Electronics

Date: November 16th, 2013

License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

Much of this is based on Mike Grusin's USB Weather Board code: https://www.sparkfun.com/products/10586

This code reads all the various sensors (wind speed, direction, rain gauge, humidty, pressure, light, batt_lvl) and reports it over the serial comm port. This can be easily routed to an datalogger (such as OpenLog) or a wireless transmitter (such as Electric Imp).

Measurements are reported once a second but windspeed and rain gauge are tied to interrupts that are calcualted at each report.

This example code assumes the GP-735 GPS module is attached.

Updated by Joel Bartlett

03/02/2017

Removed HTU21D code and replaced with Si7021

11/19/2017 - Travis Farmer

added code for DS18B20 for extendid range temp, WiFi, MQQT.

01/18/2026 - Travis Farmer

Modified for use as a Modbus RTU Slave device to report weather data over Modbus RTU protocol.

Removed the GP-735 GPS module code.


#Address Allocation

Modbus = 4

Baud = 115200

Data Bits = 8

Parity = None

Stop Bits = 1

dePin = 23

UART = Serial1

#Coils

0x00 - Not Used

#DiscreteInputs

0x00 - Not Used

#HoldingRegisters

0x00 - Not Used

#InputRegisters

0x00 - Wind Direction (degrees * 100)

0x01 - Wind Speed (mph * 100)

0x02 - Humidity (% * 100)

0x03 - Temperature (F * 100)

0x04 - Rain in last hour (inches * 100)

0x05 - Daily Rain (inches * 100)

0x06 - Battery Level (volts * 100)

0x07 - Light Level (volts * 100)

#Hardware Pin Map

Wind Speed Sensor - D3 (Interrupt 1)

Rain Gauge Sensor - D2 (Interrupt 0)

Wind Direction Sensor - A0

Light Sensor - A1

Battery Level Sensor - A2

3.3V Reference - A3

Status LED 1 - D7

Status LED 2 - D8