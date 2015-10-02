__author__ = 'nick'

import simpy
import math
import angles

DEBUG = False


def get_euclidean3d(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


class Auv:
    idle = 0
    navigate_to_target = 1
    inspect_target = 2

    linear_vel = 0.4  # In meters per second
    rot_vel = 0.3  # In rad per second
    inspection_duration = 120  # Inspection duration in seconds.

    curr_state = 0
    next_state = 0
    env = 0
    name = ""
    target_list = 0

    curr_pos = [0, 0, 0]  # All positions are NED
    curr_yaw = 0
    next_target_pos = [0, 0, 0]

    def __init__(self, env, name, targets):
        self.env = env
        self.name = name
        self.target_list = targets
        if DEBUG:
            print("Initialised vehicle {0} with targets {1}".format(name,targets))
        self.action = env.process(self.run())

    def run(self):
        while True:
            # State machine
            self.curr_state = self.next_state
            if self.curr_state == self.idle:
                # Do what you have to do when idle
                has_targets = False
                if len(self.target_list) > 0:
                    if DEBUG:
                        print("In state idle, number of targets: {0}".format(len(self.target_list)))
                    has_targets = True
                    self.next_target_pos = self.target_list.pop(0)
                if has_targets:
                    if DEBUG:
                        print("Next target is at: ", self.next_target_pos)
                    self.next_state = self.navigate_to_target
                    if DEBUG:
                        print("Next state: ", self.next_state)
                else:
                    self.next_state = self.idle
                    yield self.env.timeout(1)
            elif self.curr_state == self.navigate_to_target:
                # Do what you have to do when navigating to target
                yield self.env.process(self.send_pilot_req(self.next_target_pos))
                self.next_state = self.inspect_target
            elif self.curr_state == self.inspect_target:
                # Inspection is simulated as waiting at the spot for the moment. Will create an action later
                print("Vehicle {0} starting inspection at point {1} at time {2}".format(self.name, self.curr_pos, self.env.now))
                yield self.env.timeout(self.inspection_duration)
                print("Vehicle {0} finished inspection at {1}. Found a mine!".format(self.name, self.env.now))
                self.next_state = self.idle
            else:
                # You shouldn't be here print error message and default to idle state
                print("[ERROR]: I should not be in this state!!!")
                self.next_state = self.idle

    def send_pilot_req(self, pos):
        # Calculate how much you have to rotate and how much you have to travel
        turn = math.atan2(pos[0] - self.curr_pos[0],pos[1] - self.curr_pos[1])
        turn_deg = angles.r2d(turn)
        yaw_deg = angles.r2d(self.curr_yaw)
        norm_yaw = angles.normalize(-(yaw_deg - 90), -180, 180)
        turn = angles.normalize(norm_yaw - turn_deg, -180, 180)
        turn = angles.d2r(turn)
        yield self.env.process(self.rotate(turn))
        self.curr_yaw += turn
        self.curr_yaw = angles.d2r(angles.normalize(angles.r2d(self.curr_yaw),-180,180))
        if DEBUG:
            print("Vehicle {0} yaw: {1}".format(self.name, self.curr_yaw))
        dist = get_euclidean3d(pos, self.curr_pos)
        yield self.env.process(self.move(dist))
        self.curr_pos = pos
        if DEBUG:
            print("Vehicle {0} position: {1}".format(self.name, self.curr_pos))

    def move(self, distance):
        time = distance // self.linear_vel
        if DEBUG:
            print("Move time: ", time)
        yield self.env.timeout(time)

    def rotate(self, angle):
        time = abs(angle) // self.rot_vel
        if DEBUG:
            print("Rotate time: ", time)
        yield self.env.timeout(time)


env = simpy.Environment()
targets = [[1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [0, 0, 0]]
targets2 = [[2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 0]]
auv1 = Auv(env, "auv1", targets)
auv2 = Auv(env, "auv2", targets2)
env.run(until=3600)

