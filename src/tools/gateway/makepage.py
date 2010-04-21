#!/usr/bin/python

import getprops
 
class MakePage:
    def __init__(self):
        self.events = {}
        self.nodes = {}
        
    def WritePage(self, nodeslisting, eventslisting):
        self.events = eventslisting
        self.nodes = nodeslisting
        
        global outfile
        outfile = open("nodes.html", "w")

        outfile.write("<html><head><title>Node List</title>")
        outfile.write("<meta http-equiv=refresh content=\"1\">\n")
        outfile.write("</head>\n<body>\n")


        outfile.write("<b>MOS Webview:</b><br>\n")


        idlist = []
        # tabulate the headings/ids/nodes
        for nodes, eventidlist in nodeslisting.iteritems():
            for ids in eventidlist:
                if not ids in idlist:
                    idlist.append(ids)

        idlist.sort()

        for id in idlist:
            self.MakeTable(id)

        outfile.write("</body>\n</html>\n")
        outfile.close()

    def MakeTable(self, id):
        outfile.write("<br>Event ID: " + str(id))
        outfile.write("<br>\n")
        outfile.write("<table border=\"1\" cellpadding=\"5\" cellspacing=\"5\">\n")
        outfile.write(" <tr>\n")

        # first item is always node number
        outfile.write("  <td>Node</td>\n")

        # create headings
        headings = []

        # iterate through events
        for node,eventlist in self.nodes.iteritems():
            # make sure node has data for event
            if not eventlist.has_key(str(id)):
                continue
            for evname in eventlist[str(id)]:
                if not evname in headings:
                    s = str(evname)
                    if not (s[len(s) - 2:len(s)] == "_h"):
                        headings.append(evname)

        if "tv" in headings:
            del headings[headings.index("tv")]
            headings.append("tv")

        for heads in headings:
            outfile.write("<td>" + str(heads) + "</td>\n")

        outfile.write("</tr>")

        # sort by nodeid
        node_id_list = self.nodes.keys()
        node_id_list.sort()

        for node_id in node_id_list:
            node = node_id
            eventlist = self.nodes[node_id]

            # make sure node has data for event
            if not eventlist.has_key(str(id)):
                continue

            outfile.write("<tr><td>" + str(node) + "</td>")

            eventattrs = eventlist[str(id)]
            for heads in headings:
                if not eventattrs.has_key(heads):
                    outfile.write("<td>n/a</td>")
                    continue

                value = eventattrs[heads]
                bgcolor = self.getBackgroundColor(id,heads,value)
                outfile.write("<td bgcolor=\"" + bgcolor + "\">")
                outfile.write(str(value))
                outfile.write("</td>\n")

            outfile.write("</tr>\n")

        # close the table tag
        outfile.write("</table>")
        
    def getBackgroundColor(self, id, event, value):
        try:
            max = self.events.getDataProperty(id, event, "max")
            min = self.events.getDataProperty(id, event, "min")

        except:
            return "#FFFFFF"

        max = int(str(max))
        min = int(str(min))

        reversed = 0
        if max < min:
            nmax = max
            max = min
            min = nmax
            reversed = 1

        if value > max:
            percent = 1
        elif value < min:
            percent = 0
        else:
            range = max - min
            corrected = value - min
            percent = float(float(corrected)/float(range))


        if reversed == 0:
            mycol1 = int(percent * 255)
            mycol2 = 255 - mycol1
            mycol3 = 0
        else:
            mycol2 = int(percent * 255)
            mycol1 = 255 - mycol2
            mycol3 = 0
            

        return self.getHexColor(mycol1,mycol2,mycol3)

    def getHexColor(self, red, green, blue):
        if red > 255 or green > 255 or blue > 255:
            print "Invalid color!"
            return "#FFFFFF"

        hexstrred = self.getTwoHex(red)
        hexstrgreen = self.getTwoHex(green)
        hexstrblue = self.getTwoHex(blue)

        return "#" + hexstrred + hexstrgreen + hexstrblue

    def getTwoHex(self, value):
        hexstr = hex(value)
        hexstr = hexstr[2:len(hexstr)]
        if value < 16:
            hexstr = "0" + hexstr

        return hexstr

        
