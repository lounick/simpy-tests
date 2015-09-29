__author__ = 'nick'

# Program that simulates the movement of a simple vehicle around a 2d area.

import random
import simpy

class Vehicle:
    env_ = 0
    name_ = ""
    timeout_ = 0

    def __init__(self, env, name):
        self.env_ = env
        self.name_ =  name
        self.timeout_ = random.randint(1,10)
        print("Created instance of vehicle with name {0} and timeout {1}".format(self.name_, self.timeout_))
        # self.honk(self.timeout_)

    def honk(self, time):
        yield self.env_.timeout(time)
        print(env._now)
        print("Vehicle {0} says HONK!!!".format(self.name_))

env = simpy.Environment()
veh1 = Vehicle(env, "Vehicle1")
env.process(veh1.honk(veh1.timeout_))
env.run(until=11)