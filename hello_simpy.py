__author__ = 'nick'

# Simple program that will print hello message after a time in simulated seconds set in timeout

import simpy as sp

def hello(env, name, timeout):
    yield env.timeout(timeout)
    print("Hello SimPy from: {0} at time {1}".format(name, timeout))

env = sp.Environment()
env.process(hello(env, "hello1", 10))
env.process(hello(env, "hello2", 20))
env.run(until=30)
