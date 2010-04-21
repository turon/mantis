#!/usr/bin/python

import xml.sax
import getprops
import string
import xml.sax.handler
 
class EventListHandler(xml.sax.handler.ContentHandler):
  def __init__(self):
    self.mapping = {}

  def getDataType(self, eventid, name):
    datatypes = self.mapping[str(eventid)]
    datatype = datatypes[name]
    return datatype["type"]

  def getData(self, eventid):
    return self.mapping[str(eventid)]


  def getDataProperty(self, eventid, name, property):
    datatypes = self.mapping[str(eventid)]
    datatype = datatypes[name]
    return datatype[str(property)]

  def getPackString(self, eventid):
    datatypes = self.mapping[str(eventid)]
    packstring = ""

    for name in datatypes["order"]:
      datatype = datatypes[name]
      packstring = packstring + self.getDataType(eventid, name)
    
    return packstring

  def eventIsRegistered(self, eventid):
    if self.mapping.has_key(str(eventid)):
      return 1
    return 0

  def getNames(self, eventid):

    # retrieve the datatypes for this id
    datatypes = self.mapping[str(eventid)]

    # return the ordered list of names
    return datatypes["order"]


  def readTypes(self):
    parser = xml.sax.make_parser(  )
    parser.setContentHandler(self)
    parser.parse("props.xml")

  # this callback occurs when we recieve a new <start element>
  # tag, respond according to the name of the tag
  def startElement(self, name, attributes):
    # new event tag, so start a new event keyed off of ID
    if name == "event":
      self.event = attributes["id"]
      self.event_dict = {}
      self.event_order = []

    # new data attribute, store in current event
    elif name == "data":
      name = attributes["name"]
      datadict = {}
      datadict["type"] = attributes["type"]
      if attributes.has_key("max"):
        datadict["max"] = attributes["max"]
      if attributes.has_key("min"):
        datadict["min"] = attributes["min"]
      self.event_dict[name] = datadict

      # keep a list so we can remember the order
      self.event_order.append(name)

  # this shouldn't occur in our xml file
  def characters(self, data):
    if len(data) == 1:
      return
    if not self.isWhitespace(data):
      print 'unknown characters', len(data), data

 
  def endElement(self, name):  
    if name == "event":
      # store the order so it can be unpacked correctly
      self.event_dict["order"] = self.event_order
      self.mapping[self.event] = self.event_dict

  # determine if a set of characters is whitespace
  def isWhitespace(self, data):
    for chars in data:
      if not chars in string.whitespace:
        return 0
    return 1
      


