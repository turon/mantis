import serial
import sys
from time import localtime,strftime
ser = serial.Serial(sys.argv[1],57600)
fout = open(sys.argv[2],'a')
while (1):
    line = ser.readline()[4:]
    fout.write(strftime("%H:%M:%S  ",localtime()) + line)
    fout.flush()
