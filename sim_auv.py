__author__ = 'nick'

import simpy
import math
import angles

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

    curr_pos = [0, 0, 0] # All positions are NED
    curr_yaw = 0
    next_target_pos = [0, 0, 0]

    def __init__(self, env, name, targets):
        self.env = env
        self.name = name
        self.action = env.process(self.run())
        self.target_list = targets

    def run(self):
        # State machine
        self.curr_state = self.next_state
        if self.curr_state == self.idle:
            # Do what you have to do when idle
            targets = 0
            if len(self.target_list) > 0:
                targets = 1
                self.next_target_pos = self.target_list.pop(0)
            if targets:
                self.next_state = self.navigate_to_target
            else:
                self.next_state = self.idle
        elif self.curr_state == self.navigate_to_target:
            # Do what you have to do when navigating to target
            target_pos = [0, 0, 0]
            yield self.env.process(self.send_pilot_req(target_pos))
            self.next_state = self.inspect_target
        elif self.curr_state == self.inspect_target:
            # Inspection is simulated as waiting at the spot for the moment. Will create an action later
            print("Starting inspection at point {0} at time {1}".format(self.curr_pos, self.env.now))
            yield self.env.timeout(self.inspection_duration)
            print("Inspection finished at {0}. Found a mine!".format(self.env.now))
            self.next_state = self.idle
        else:
            # You shouldn't be here print error message and default to idle state
            print("[ERROR]: I should not be in this state!!!")
            self.next_state = self.idle

    def send_pilot_req(self, pos):
        # Calculate how much you have to rotate and how much you have to travel
        turn = atan2(pos[1] - self.curr_pos[1], pos[0] - self.curr_pos[0])
        turn_deg = angles.r2d(turn)
        yaw_deg = angles.r2d(self.curr_yaw)
        norm_yaw = angles.normalize(-(yaw_deg-90), -180, 180)
        turn = angles.d2r(angles.normalize(norm_yaw-turn_deg, -180, 180))
        pass

    def move(self, distance):
        pass

    def rotate(self, angle):
        pass
