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
        if (vals[i] == 0xFA and vals[i+1] == 0xFE) or (vals[i] == 0xFB and vals[i+1] == 0xFE):

            print("sync\n\tpulse: {} + {} (s)\n\tscanmode: {}\n\trow: {} + {} (s)\n\tMB: {}".format(
                  (int.from_bytes(vals[i+2:i+4], "little"))/16, 
                  (int.from_bytes(vals[i+4:i+6], "little"))/1000000, 
                  int.from_bytes(vals[i+6:i+8], "little"), 
                  (int.from_bytes(vals[i+8:i+10], "little"))/16,
                  (int.from_bytes(vals[i+10:i+12], "little"))/1000000,
                  bytecount/1000/1000))
        i += 1
        bytecount += 1
    f.write(vals)

f.close()
