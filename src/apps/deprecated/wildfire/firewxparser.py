import re
import sys
import math

######### BEGIN THINGS YOU NEED TO FILL IN#####################
#put the numbers of the nodes that were used in here
nodelist = [11, 12, 21, 50, 52, 13]

#node location
location = "Middle Fork"

#elevations in the format {nodeID: elevation, nodeID2: elevation2, ...}
elevation = {11: 500, 12: 600, 21: 700, 50: 750, 52: 900, 13: 0}

#coordinates in the format {nodeID: lat or long, nodeID2: lat or long, ...}
latitude = {11: 45.2, 12: 25.3, 21: 16.3, 50: 26.3, 52: 26.3, 13: 0}
longitude = {11: 45.2, 12: 25.3, 21: 16.3, 50: 26.3, 52: 26.3, 13: 0}


######### END THINGS YOU NEED TO FILL IN######################


if len(sys.argv) != 3:
    print "usage:  python lastob.py infile outfile"
    print ""
    sys.exit()

fin = open(sys.argv[1],'r')
fout = open(sys.argv[2],'w')

startlist = []

start = 0

start = 0
tempavg = {0: 0}
humidavg = {0: 0}
countlist = {0: 0}
windx = {0: 0}
windy = {0: 0}
windcount = {0: 0}

for i in nodelist:
    tempavg[i] = 0
    humidavg[i] = 0
    windx[i] = 0
    windy[i] = 0

date = ''
time = ''


fout.write("date\t\ttime\tsrc\tlat\tlong\telev\ttemp\thumid\tdwpt\twspd\t\twdir\n")

first = 0;

consta = 17.27
constb = 237.7


for line in fin:
    a = line.strip()
    a = a.replace("  ", "\t")
    a = a.replace(":", "\t")
    a = a.split('\t')
    #print a
    if a[0] == "sent preamble":  #this is the separator between 15 minute intervals
        for i in nodelist:
            countlist[i] = 1  #set counts to 1 for averages
            windcount[i] = 1
        if first == 0:
            first = 1  #don't want to print out on the first go-round
            continue
        start += 1
        #if we're here then we've gone through the list once, so print out the averages for the 15 minutes
        for i in nodelist:
            if windx[i] == 0.0: #have to fudge a little so no divide by zero
                windx[i] = .0001001
            windspeed = math.sqrt(windx[i]**2 + windy[i]**2)
            winddir = math.atan(windy[i]/windx[i]) * 180/3.14
            #to account for polar conversions
            if (windx[i] < 0):
                winddir += 180
            elif (windx[i] > 0 and windy[i] < 0):
                winddir += 360
            #dewpoint calculation (approximation)
            if humidavg[i] == 0:
                humidavg[i] = .0000001 #have to fudge a little because ln(0) is undefined
            tempc = 5.0/9.0*(tempavg[i]-32.0)
            RH = humidavg[i]/100.0
            lam = (consta*tempc/(constb + tempc)) + math.log(RH)            
            dewpoint = constb * lam / (consta - lam)
            dewf = (9/5)*dewpoint + 32
            line = ("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%10s\t%s\t%s\n") % (date, time, i, latitude[i], longitude[i], elevation[i], tempavg[i], humidavg[i], dewf, windspeed, winddir)
            fout.write(line)
            fout.flush()
            #print ("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s") % (date, time, i, latitude[i], longitude[i], elevation[i], tempavg[i], humidavg[i])
            tempavg[i] = 0
            humidavg[i] = 0
            windx[i] = 0
            windy[i] = 0
    if len(a) <= 5 or a[4] == "src":  #remove formatting and other comments
        continue

    node = int(a[4])
    wspeed = int(a[12])
    if int(a[11] == 0):
        wdir = 1
    else:
        wdir = int(a[11])
    if countlist[node] == 1:
        tempavg[node] = a[9]
        humidavg[node] = a[10]
        #grab the date on the first one
        date = a[0]
        time = a[1] + ":" + a[2]
        if wspeed != 0:
            windx[node] = wspeed * math.cos(wdir * 3.14/180)
            windy[node] = wspeed * math.sin(wdir * 3.14/180)

    else:
        #calculate averages
        temp = a[9]
        tempavg[node] = (int(tempavg[node]) * (windcount[node] - 1) + int(temp))/windcount[node]
        humid = a[10]
        humidavg[node] = (int(humidavg[node]) * (windcount[node] - 1) + int(humid))/windcount[node]
        if wspeed != 0:
            windx[node] = ((windx[node] * (windcount[node] - 1)) + (wspeed * math.cos(wdir * 3.14/180)))/windcount[node]
            windy[node] = ((windy[node] * (windcount[node] - 1)) + (wspeed * math.sin(wdir * 3.14/180)))/windcount[node]
            windcount[node] += 1
            
    countlist[node] = countlist[node] + 1



    
  
