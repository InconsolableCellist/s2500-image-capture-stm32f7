#!/usr/bin/python3 
import serial

ser = serial.Serial(
    port='/dev/ttyACM2',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

f = open('data.dat', 'wb')
while 1:
    f.write(ser.read(ser.inWaiting()))

f.close()
