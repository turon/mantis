
import serial
import sys
import time
import string

preamble = 'S'  #mos preamble

if len(sys.argv) != 2:
    print "usage:  python firewx.py /dev/tty[]"
    print ""
    sys.exit()
    
ser = serial.Serial(port=sys.argv[1],baudrate=57600,timeout=.5)

print("starting script.. waiting..\n")

ser.write(str(preamble).strip())
ser.write(str(preamble).strip())
ser.write("#")  #pyserial only sends strings, not bytes, so we send # which == 35
ser.write('startstartstartstartstartstartstart') #send a start message
print("sent preamble\n")


while 1:
    hour = time.localtime()[3]
    minute = time.localtime()[4]
    second = time.localtime()[5]

    line = ser.readline()
    if len(line) > 1:
        t = ord(line[0])
        t1 = ord(line[1])
        t2 = ord(line[2])
        t3 = ord(line[3])
        #usually we preamble,size from the node, looks like SSX where x is the size
        while ((t == 83 and t1 == 83) or ((t == 0 or t > 123) and t1 == 83 and t2 == 83)):
            if t == 83 and t1 == 83:
                line = line[3:]
            #sometimes we get a random leading 0.  this handles that case
            elif ((t == 0 or t > 123) and t1 == 83 and t2 == 83):
                line = line[4:]
            t = ord(line[0])
            t1 = ord(line[1])
            t2 = ord(line[2])
            t3 = ord(line[3])
    if len(line) > 1:
        print(time.strftime("%D\t%H:%M:%S  ",time.localtime()) + line + "\n")
       

            
