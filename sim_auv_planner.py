__author__ = 'nick'

from sim_auv import *

class AuvPlanner:
    env = 0
    executor = 0
    def __init__(self, env, auv_executor):
        self.env = env
        self.executor = auv_executor

    def run(self):
        pass

if __name__ == "__main__":
    print("running main function")