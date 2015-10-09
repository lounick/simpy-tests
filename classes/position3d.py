#!/usr/bin/python
__author__ = 'nick'

import json
import calendar
import time

class position3d:
    id = ""
    lat = 0
    long = 0
    depth = 0
    timestamp = 0;

    def __init__(self, uid, lat, long, depth, timestamp):
        self.id = uid
        self.lat = lat
        self.long = long
        self.depth = depth
        self.timestamp = timestamp
        # self.timestamp = calendar.timegm(time.gmtime())

    def getJSON(self):
        return json.dumps(self.__dict__)

def main():
    p = position3d("1", 1, 1, 1, calendar.timegm(time.gmtime()))
    print(p.getJSON())

if __name__ == "__main__":
    main()
