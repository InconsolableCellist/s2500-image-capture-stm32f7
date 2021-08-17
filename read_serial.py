#!/usr/bin/python3 
import serial

ser = serial.Serial(
    port='/dev/ttyACM1',
    baudrate=9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

ser.isOpen()

f = open('data.dat', 'wb')
bytecount = 0
i = 0
while 1:
    vals = ser.read(ser.inWaiting())
    i = 0
    for val in vals:
        if vals[i] == 0xFE and vals[i+1] == 0xFE:
            print("sync\n\ttimer overflows: {}, microseconds: {}\n\tscanmode: {}\n\tframe overflows: {}, microseconds: {}\n\tMB: {}".format(
                  int.from_bytes(vals[i+2:i+3], "little"), 
                  int.from_bytes(vals[i+4:i+5], "little"), 
                  int.from_bytes(vals[i+6:i+7], "little"), 
                  int.from_bytes(vals[i+8:i+9], "little"),
                  int.from_bytes(vals[i+10:i+11], "little"), 
                  bytecount/1000/1000))
        i += 1
        bytecount += 1
    f.write(vals)

f.close()
