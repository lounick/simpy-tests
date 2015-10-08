__author__ = 'nick'

import simpy


SIM_DURATION = 100


class Pipe(object):
    """This class represents the propagation through a cable."""
    def __init__(self, env, delay):
        self.env = env
        self.delay = delay
        self.store = simpy.Store(env)

    def latency(self, value):
        yield self.env.timeout(self.delay)
        self.store.put(value)

    def put(self, value):
        self.env.process(self.latency(value))

    def get(self):
        return self.store.get()


class Receiver:
    def __init__(self, env, cable):
        self.env = env
        self.cable = cable
        self.action = env.process(self.run())
        self.msg_event = self.cable.get()
        self.msg_event.callbacks.append(self.msg_callback)
        self.msg_rcv = False

    def msg_callback(self, event):
        self.msg_event = self.cable.get()
        self.msg_event.callbacks.append(self.msg_callback)
        print("Got message at {0} saying {1}".format(self.env.now, event.value))
        # self.action.interrupt()
        self.msg_rcv = True

    def run(self):
        while True:
            self.msg_event = self.cable.get()
            self.msg_event.callbacks.append(self.msg_callback)
            self.msg_rcv = False
            try:
                print("Doing normal work at {0}".format(self.env.now))
                yield self.env.timeout(20)
            except simpy.Interrupt:
                print("Interrupted!!!")


def main():
    def __sender(env, cable):
        """A process which randomly generates messages."""
        while True:
            # wait for next transmission
            yield env.timeout(5)
            print("Put message to pipe at {0}".format(env.now))
            cable.put('Sender sent this at %d' % env.now)


    def __msg_callback(event):
        print("Received message at {0} saying {1}".format(env.now, event.value))

    # def __receiver(env, cable):
    #     """A process which consumes messages."""
    #     while True:
    #         # Get event for message pipe
    #         msg_event = cable.get()
    #         msg_event.callbacks.append(__msg_callback)
    #         msg_timeout = env.timeout(1)
    #         yield msg_timeout
    #         # if msg_event.triggered:
    #         #     # print('Received this at %d while %s' % (env.now, msg))
    #         #     print("Received message at {0} saying {1}".format(env.now, msg_event.value))
    #         #     msg_timeout.cancel()
    #         # else:
    #         #     msg_event.cancel()
    #         #     print("Still waiting for message at {0}".format(env.now))
    #         print("Doing stuff at {0}".format(env.now))





    # Setup and start the simulation
    print('Event Latency')
    env = simpy.Environment()

    cable = Pipe(env, 2)
    env.process(__sender(env, cable))
    # env.process(__receiver(env, cable))
    receiver = Receiver(env, cable)
    env.run(until=SIM_DURATION)

if __name__=="__main__":
    main()

