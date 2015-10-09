#!/usr/bin/python
__author__ = 'nick'

import json
import calendar
import time

class target:
    id = ""
    position3d = ""
    classification = ""
    timestamp = 0

    def __init__(self, uid, position3d, classification, timestamp):
        self.id = uid
        self.position3d = position3d
        self.classification = classification
        self.timestamp = timestamp
        # self.timestamp = calendar.timegm(time.gmtime())

    def getJSON(self):
        return json.dumps(self.__dict__)

def main():
    t = target("1", "pos", "none", calendar.timegm(time.gmtime()))
    print(t.getJSON())

if __name__=="__main__":
    main()