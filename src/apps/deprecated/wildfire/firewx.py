
import serial
import sys
import time
import string

preamble = 'S'  #mos preamble

if len(sys.argv) != 3:
    print "usage:  python firewx.py /dev/tty[] filename"
    print ""
    sys.exit()
    
ser = serial.Serial(port=sys.argv[1],baudrate=57600,timeout=.5)
fout = open(sys.argv[2],'a')

fout.write("starting script.. waiting..\n")
fout.flush()

while (1):
    interval = 15
    now = time.localtime()[4]
    if now % interval == 0:
        #must send twice to mimic mos preamble
        while (1):
            ser.write(str(preamble).strip())
            ser.write(str(preamble).strip())
            ser.write("#")  #pyserial only sends strings, not bytes, so we send # which == 35
            ser.write('startstartstartstartstartstartstart') #send a start message
            fout.write("sent preamble\n")
            fout.flush()
            line = ser.readline()  # we don't print this line, so just read it.
            #it should just be the response from the node that it's starting
            if len(line) > 1:
                break
            else:
                fout.write("resending preamble\n")
                fout.flush()
            
        while now % interval == 0:
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
                fout.write(time.strftime("%D\t%H:%M:%S  ",time.localtime()) + line)
                fout.flush()
                
            now = time.localtime()[4]
    else:
        line = ser.readline()
 
        #sometimes we just get random carriage returns from the node?  this handles that
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
            fout.write(time.strftime("%D\t%H:%M:%S  ",time.localtime()) + line)
            fout.flush()
            
