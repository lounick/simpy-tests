#!/usr/bin/python
__author__ = 'nick'

import json
import calendar
import time
import queries

class vehicle:
    id = ""
    name = ""
    type = ""
    position3d = ""
    timestamp = 0

    def __init__(self, uid, name, type, position3d, timestamp):
        self.id = uid
        self.name = name
        self.type = type
        self.position3d = position3d
        self.timestamp = timestamp
        # self.timestamp = calendar.timegm(time.gmtime())

    def getJSON(self):
        return json.dumps(self.__dict__)

def main():
    v = vehicle("1", "sauv1", "search", "pos", calendar.timegm(time.gmtime()))
    queries.insert_instance(1,1,1,v)
    print(v.getJSON())

if __name__=="__main__":
    main()
