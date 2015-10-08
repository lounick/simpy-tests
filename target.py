__author__ = 'nick'

# Simple target class.


class Target:
    id = 0
    classification = "None"
    ned_pos = [0, 0, 0]

    def __init__(self, uid, pos):
        self.id = uid
        self.ned_pos = pos
