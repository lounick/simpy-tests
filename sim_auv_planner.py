__author__ = 'nick'

from sim_auv import *

class AuvPlanner:

    planer_timeout = 10 # Planner rate 0.1 Hz to check.

    env = 0
    plan_request = 0
    plan_feedback = 0
    def __init__(self, env, plan_request, plan_feedback):
        self.env = env
        self.plan_request = plan_request
        self.plan_feedback = plan_feedback
        self.action = env.process(self.run())

    def run(self):
        while True:
            pass



def main():
    pass

if __name__ == "__main__":
    main()