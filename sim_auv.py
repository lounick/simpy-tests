__author__ = 'nick'

import simpy
import math
import angles
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import os

from pipe import Pipe
from msgs import *
from target import Target

# DEBUG = os.environ.get('DEBUG', None) == "True"
# PLOT = os.environ.get('PLOT', None) == "True"
DEBUG = False
PLOT = True


def get_euclidean3d(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2 + (p1[2] - p2[2]) ** 2)


class AuvExecutor:
    # States
    idle = 0
    navigate_to_target = 1
    inspect_target = 2

    # Vehicle constants
    linear_vel = 0.8  # In meters per second
    rot_vel = 0.3  # In rad per second
    inspection_duration = 120  # Inspection duration in seconds.

    # Control variables
    curr_state = 0
    next_state = 0
    env = 0
    name = ""
    target_list = []
    target_order = []
    expected_timeout = 0
    action_start = 0
    plan_request = 0
    plan_feedback = 0
    nav_req = 0
    nav_update = 0

    curr_pos = [0, 0, 0]  # All positions are NED
    curr_yaw = 0
    curr_target = 0
    curr_lin_vel = 0
    curr_rot_vel = 0

    plan_msg_event = 0
    nav_msg_event = 0

    def __init__(self, env, name, plan_request, plan_feedback, nav_req, nav_update):
        self.env = env
        self.name = name
        self.plan_request = plan_request
        self.plan_feedback = plan_feedback
        self.plan_msg_event = self.plan_request.get()
        self.plan_msg_event.callbacks.append(self.handle_plan_msg)
        self.nav_req = nav_req
        self.nav_msg_event = self.nav_req.get()
        self.nav_msg_event.callbacks.append(self.handle_nav_req)
        self.nav_update = nav_update
        if DEBUG:
            print("Initialised vehicle {0}".format(name))
        self.action = env.process(self.run())

    def run(self):
        while True:
            # State machine

            self.curr_state = self.next_state
            if self.curr_state == self.idle:
                # Do what you have to do when idle
                has_targets = False
                if len(self.target_order) > 0:
                    if DEBUG:
                        print("In state idle, number of targets: {0}".format(len(self.target_list)))
                    while not has_targets and len(self.target_order) > 0:
                        self.curr_target = self.target_list[self.target_order[0]]
                        has_targets = True
                        if self.curr_target.classification != "None":
                            if DEBUG:
                                print("Target already classified, skipping")
                            self.target_order.pop(0)
                            has_targets = False
                if has_targets:
                    if DEBUG:
                        print("Next target is at: ", self.curr_target.ned_pos)
                    self.next_state = self.navigate_to_target
                    if DEBUG:
                        print("Next state: ", self.next_state)
                else:
                    self.next_state = self.idle
                    if DEBUG:
                        print("Waiting for targets")
                    yield self.env.timeout(1)
            elif self.curr_state == self.navigate_to_target:
                # Do what you have to do when navigating to target
                try:
                    yield self.env.process(self.send_pilot_req(self.curr_target.ned_pos))
                    self.next_state = self.inspect_target
                except simpy.Interrupt:
                    self.next_state = self.idle
            elif self.curr_state == self.inspect_target:
                # Inspection is simulated as waiting at the spot for the moment. Will create an action later
                print("Vehicle {0} starting inspection at point {1} at time {2}".format(self.name, self.curr_pos,
                                                                                        self.env.now))
                yield self.env.timeout(self.inspection_duration)
                print("Vehicle {0} finished inspection at {1}. Found a mine!".format(self.name, self.env.now))
                self.curr_target.classification = "Mine"
                self.plan_feedback.put(PlanFeedbackMsg(self.curr_target.id, self.curr_target.classification))
                self.target_order.pop(0)
                self.next_state = self.idle
            else:
                # You shouldn't be here print error message and default to idle state
                print("[ERROR]: I should not be in this state!!!")
                self.next_state = self.idle

    def send_pilot_req(self, pos):
        # Calculate how much you have to rotate and how much you have to travel
        turn = math.atan2(pos[0] - self.curr_pos[0], pos[1] - self.curr_pos[1])
        turn_deg = angles.r2d(turn)
        yaw_deg = angles.r2d(self.curr_yaw)
        norm_yaw = angles.normalize(-(yaw_deg - 90), -180, 180)
        turn = angles.normalize(norm_yaw - turn_deg, -180, 180)
        turn = angles.d2r(turn)

        yield self.env.process(self.rotate(turn))

        self.curr_yaw += turn
        self.curr_yaw = angles.d2r(angles.normalize(angles.r2d(self.curr_yaw), -180, 180))

        if DEBUG:
            print("Vehicle {0} yaw: {1}".format(self.name, self.curr_yaw))

        dist = get_euclidean3d(pos, self.curr_pos)
        yield self.env.process(self.move(dist))

        self.curr_pos = pos

        if DEBUG:
            print("Vehicle {0} position: {1}".format(self.name, self.curr_pos))

    def move(self, distance):
        time = distance / self.linear_vel
        self.curr_lin_vel = self.linear_vel
        self.action_start = self.env.now
        self.expected_timeout = self.env.now + time
        if DEBUG:
            print("Move time: ", time)
        yield self.env.timeout(time)
        self.curr_lin_vel = 0

    def rotate(self, angle):
        time = abs(angle) / self.rot_vel
        self.curr_rot_vel = self.rot_vel
        self.action_start = self.env.now
        self.expected_timeout = self.env.now + time
        if DEBUG:
            print("Rotate time: ", time)
        yield self.env.timeout(time)
        self.curr_rot_vel = 0

    def get_position(self):
        if self.curr_lin_vel > 0:
            # Vehicle is currently moving so must extrapolate
            return [
                np.interp(self.env.now, [self.action_start, self.expected_timeout], [self.curr_pos[0], self.curr_target.ned_pos[0]]),
                np.interp(self.env.now, [self.action_start, self.expected_timeout], [self.curr_pos[1], self.curr_target.ned_pos[1]]),
                np.interp(self.env.now, [self.action_start, self.expected_timeout], [self.curr_pos[2], self.curr_target.ned_pos[2]])
            ]
        else:
            return self.curr_pos

    def handle_plan_msg(self, event):
        self.plan_msg_event = self.plan_request.get()
        self.plan_msg_event.callbacks.append(self.handle_plan_msg)
        if self.curr_state == self.navigate_to_target:
            self.curr_pos = self.get_position()
            self.action.interrupt()
        self.target_list = event.value.target_list
        self.target_order = event.value.target_order

    def handle_nav_req(self, event):
        self.nav_msg_event = self.nav_req.get()
        self.nav_msg_event.callbacks.append(self.handle_nav_req)
        current_pos = self.get_position()
        self.nav_update.put(NavUpdateMsg(current_pos))


def position_printer(env, veh, ax):
    ax.set_xlabel("East")
    ax.set_ylabel("North")
    ax.set_zlabel("Depth")
    ax.hold(True)
    ax.grid(True)
    first_time = True
    prev_pos = 0
    while True:
        if first_time:
            ax.scatter(veh.get_position()[1], veh.get_position()[0], veh.get_position()[2], 'o', label='inspection points')
            first_time = False
            prev_pos = veh.get_position()
        else:
            if not prev_pos == veh.get_position():
                ax.scatter(veh.get_position()[1], veh.get_position()[0], veh.get_position()[2], 'o', label='inspection points')
                prev_pos = veh.get_position()
                # print(veh.get_position())

        yield env.timeout(10)


class TargetGenerator:
    # targets = np.random.uniform(-1,1,[8,3])
    # targets[:, 2] = np.random.rand(targets.shape[0])
    targets = np.array([[1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [0, 0, 0]])
    targets = targets*100
    targets = targets.tolist()
    targets_list = []
    uid = 0

    def __init__(self, env, plan_req, plan_fb, nav_req, nav_update):
        self.env = env
        self.plan_req = plan_req
        self.plan_fb = plan_fb
        self.plan_msg_event = self.plan_fb.get()
        self.plan_msg_event.callbacks.append(self.handle_plan_feedback)
        self.nav_req = nav_req
        self.nav_update = nav_update
        self.nav_msg_event = self.nav_update.get()
        self.nav_msg_event.callbacks.append(self.handle_nav_update)
        if DEBUG:
            print("Initialised")
        self.action = env.process(self.run())

    def handle_plan_feedback(self, event):
        self.plan_msg_event = self.plan_fb.get()
        self.plan_msg_event.callbacks.append(self.handle_plan_feedback)
        self.targets_list[event.value.target_id][1].classification = event.value.target_class
        print("The AUV has classified target {0} with class {1}".format(event.value.target_id, self.targets_list[event.value.target_id][1].classification))

    def handle_nav_update(self, event):
        self.nav_msg_event = self.nav_update.get()
        self.nav_msg_event.callbacks.append(self.handle_nav_update)
        print("The vehicle is at {0}".format(event.value.pos))

    def run(self):
        while True:
            # self.msg_event = self.plan_fb.get()
            # self.msg_event.callbacks.append(self.handle_plan_feedback)
            self.nav_req.put(NavReqMsg())
            if len(self.targets) > 0:
                # There are more targets to generate
                self.targets_list.append([self.uid, Target(self.uid, self.targets.pop(0))])
                self.uid += 1

                unclassified_targets = []
                for i in range(len(self.targets_list)):
                    if self.targets_list[i][1].classification == "None":
                        unclassified_targets.append([i, self.targets_list[i][1]])
                if len(unclassified_targets) > 0:
                    # Should create a plan
                    targets_order = range(len(unclassified_targets))
                    target_list = []
                    for i in range(len(unclassified_targets)):
                        target_list.append(unclassified_targets[i][1])
                    self.plan_req.put(PlanReqMsg(target_list, targets_order))
            yield self.env.timeout(120)


def main():
    # Simulation setup
    start = time.time()
    env = simpy.Environment()

    # Process cabling
    plan_request = Pipe(env, 0)
    plan_feedback = Pipe(env, 0)
    nav_req = Pipe(env, 0)
    nav_update = Pipe(env, 0)
    fig, ax = plt.subplots()
    ax = fig.add_subplot(111, projection='3d')

    tg = TargetGenerator(env, plan_request, plan_feedback, nav_req, nav_update)
    auv1 = AuvExecutor(env, "auv1", plan_request, plan_feedback, nav_req, nav_update)

    if PLOT:
        env.process(position_printer(env, auv1, ax))
    env.run(until=3600)
    print(time.time()-start)
    if PLOT:
        plt.show()

if __name__ == "__main__":
    main()
