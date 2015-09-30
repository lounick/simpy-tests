__author__ = 'nick'

# Program that simulates the movement of a simple vehicle around a 2d area.

import random
import simpy

class Vehicle:
    env = 0
    name = ""
    timeout = 0

    def __init__(self, env, name):
        self.env = env
        self.name = name
        # self.timeout_ = random.randint(1,10)
        print("Created instance of vehicle with name {0}".format(self.name))
        self.action = env.process(self.run())
        # yield self.env_.process(self.honk(self.timeout_))

    def run(self):
        # Main vehicle loop.
        while True:
            print('Start parking and charging at %d' % self.env.now)
            charge_duration = 5
            # We yield the process that process() returns
            # to wait for it to finish
            # We may get interrupted while charging the battery
            try:
                yield self.env.process(self.charge(charge_duration))
            except simpy.Interrupt:
                # When we received an interrupt, we stop charing and
                # switch to the "driving" state
                print('Was interrupted. Hope, the battery is full enough ...')

            # The charge process has finished and
            # we can start driving again.
            print('Start driving at %d' % self.env.now)
            trip_duration = 2
            yield self.env.timeout(trip_duration)

    def charge(self, duration):
        yield self.env.timeout(duration)

    def honk(self, time):
        yield self.env_.timeout(time)
        print("Vehicle {0} says HONK!!!".format(self.name))


def driver(env, car):
    yield env.timeout(3)
    car.action.interrupt()

env = simpy.Environment()
veh1 = Vehicle(env, "Vehicle1")
env.process(driver(env, veh1))
env.run(until=15)
