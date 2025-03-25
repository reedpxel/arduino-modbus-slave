Firmware for Arduino that allows to use an Arduino board as a Modbus RTU slave device and read data from a BMS board manufactured by JiaBaiDa. 
Connection of Arduino board and Modbus RTU master device uses RS485 interface and Modbus RTU protocol, pins 3 - RX, 5 - TX, 7 - RE-DE. 
Connection of Arduino and BMS boards uses RS485 interface and JiaBaiDa BMS interface (https://cdn.shopifycdn.net/s/files/1/0606/5199/5298/files/JDB_RS485-RS232-UART-Bluetooth-Communication_Protocol.pdf?v=1682577935), pins 9 - RX, 11 - TX.
Firmware was tested with Arduino Uno and Nano (ATMEGA168) and JBD BMS 13S60A.
