Modbus MQTT-RTU Converter
=======
Controlling Modbus device via the internet network.
This firmware uses for ESP8266 Wi-Fi Module such as NodeMCU board.
-----------

[Application Example](http://www.somsakelect.com/2020/05/30/modbus-mqtt/#App)
[Modbus MQTT](http://www.somsakelect.com/2020/05/30/modbus-mqtt/)
[Modbus RTU](https://www.modbustools.com/modbus.html)

How to use firmware
-----------
1. [Download .bin file] to your computer.
2. Download firmware to ESP8266 Wi-Fi module by [NodeMCU Flasher](https://github.com/nodemcu/nodemcu-flasher)
3. Connect module to Modbus RTU device via serial port with TTL.
4. Power ON the module and connect Wi-Fi from ESP8266 Wi-Fi module such as MB_MQTT_XXXX.
5. Open URL http://192.168.4.1
6. Settings a serial communication, MQTT broker and Wi-Fi network.
7. Press 'Save' and waiting success.
8. Download and install [Modbus Online](http://www.somsakelect.com/wp-content/uploads/2020/05/Modbus-Online-Installer-V1.zip)
9. Open Modbus Online and setting MQTT broker
10. Type 'DeviceID' of ESP8266 Wi-Fi module that get from Wi-Fi name in AP mode (MB_MQTT_XXXX)
11. Press 'Start' for running TCP service
12. Download and install [Modbus Poll](https://www.modbustools.com/modbus_poll.html)
13. Open Modbus Poll and set connection to TCP/IP communications by getting IP address and port from Modbus Online software
14. Read/Write register of Modbus RTU device by Modbus Poll
