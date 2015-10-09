__author__ = 'nick'

DEBUG = True
PLOT = True

import simpy
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from ovrp_solver import *
from msgs import *
from target import Target
from pipe import Pipe
from sim_auv import AuvExecutor, get_euclidean3d, position_printer

# DEBUG = os.environ.get('DEBUG', None) == "True"
# PLOT = os.environ.get('PLOT', None) == "True"


class AuvPlanner:

    planer_timeout = 100  # Planner rate 0.1 Hz to check.

    env = 0
    plan_request = 0
    plan_feedback = 0

    def __init__(self, env, plan_request, plan_feedback, nav_req, nav_update):
        self.env = env
        self.plan_request = plan_request
        self.plan_feedback = plan_feedback
        self.plan_msg_event = self.plan_feedback.get()
        self.plan_msg_event.callbacks.append(self.handle_plan_feedback)
        self.nav_req = nav_req
        self.nav_update = nav_update
        self.nav_msg_event = self.nav_update.get()
        self.nav_msg_event.callbacks.append(self.handle_nav_update)
        self.vehicle_pos = [0, 0, 0]
        self.got_nav = False
        if DEBUG:
            self.__testing__()
        self.action = env.process(self.run())

    def __testing__(self):
        # self.targets = np.random.uniform(-1,1,[8,3])
        # self.targets[:, 2] = -1*np.random.rand(self.targets.shape[0])
        self.targets = np.array([[1, 0, -1], [1, 1, -1], [0, 1, 0], [0, 0, 0], [-1, 0, 0], [-1, -1, 0], [0, -1, 0], [0, 0, 0]])
        self.targets = self.targets*100
        self.targets = self.targets.tolist()
        self.targets_list = []
        self.uid = 0
        for i in range(len(self.targets)):
            self.targets_list.append([self.uid, Target(self.uid, self.targets.pop(0))])
            self.uid += 1

    def handle_plan_feedback(self, event):
        self.plan_msg_event = self.plan_feedback.get()
        self.plan_msg_event.callbacks.append(self.handle_plan_feedback)
        self.targets_list[event.value.target_id][1].classification = event.value.target_class

    def handle_nav_update(self, event):
        self.got_nav = True
        self.nav_msg_event = self.nav_update.get()
        self.nav_msg_event.callbacks.append(self.handle_nav_update)
        if DEBUG:
            print("Vehicle at {0}".format(event.value.pos))
        self.vehicle_pos = event.value.pos

    def run(self):
        # new_targets = True
        first_time = True
        while True:
            # Check for new targets from db
            # If new targets:
            #   Request nav update
            #   Get unclassified targets
            #   Organize targets with tsp
            #   Send plan
            start = time.time()
            new_targets = False
            if DEBUG:
                if first_time:
                    new_targets = True
                    first_time = False
            else:
                # Write request for new targets from DB
                pass
            if new_targets:
                self.got_nav = False
                self.nav_req.put(NavReqMsg())
                yield self.env.timeout(1)
                unclassified_targets = []
                for i in range(len(self.targets_list)):
                    if self.targets_list[i][1].classification == "None":
                        unclassified_targets.append([i, self.targets_list[i][1]])

                if len(unclassified_targets) > 0:
                    positions = []
                    positions.append(self.vehicle_pos)
                    target_list = []
                    for i in range(len(unclassified_targets)):
                        positions.append(unclassified_targets[i][1].ned_pos)
                        target_list.append(unclassified_targets[i][1])

                    n = len(positions)
                    distances = np.zeros((n, n))
                    positions = np.array(positions)

                    for k in xrange(n):
                        for p in xrange(n):
                            distances[k, p] = np.linalg.norm(positions[k, :] - positions[p, :])

                    tsp_route, total_cost, model = ovrp_solver(distances)

                    if DEBUG:
                        print("OVRP Route: {0}".format(tsp_route))

                    target_order = []
                    for i in range(1, len(tsp_route)):
                        target_order.append(tsp_route[i] - 1)

                    if DEBUG:
                        print(target_order)
                        print(target_list)

                    self.plan_request.put(PlanReqMsg(target_list, target_order))
                    if DEBUG:
                        print("Computation: {0}".format(time.time() - start))
                    yield self.env.timeout(time.time() - start)

            yield self.env.timeout(self.planer_timeout)





def main():
    # Simulation setup
    start = time.time()
    env = simpy.Environment()

    # Process cabling
    plan_request = Pipe(env, 0)
    plan_feedback = Pipe(env, 0)
    nav_req = Pipe(env, 0)
    nav_update = Pipe(env, 0)
    # env.process(__target_generator(env, plan_request, plan_feedback))
    auv1 = AuvExecutor(env, "auv1", plan_request, plan_feedback, nav_req, nav_update)
    vehicle_planner = AuvPlanner(env, plan_request, plan_feedback, nav_req, nav_update)

    fig, ax = plt.subplots()
    ax = fig.add_subplot(111, projection='3d')
    if PLOT:
        env.process(position_printer(env, auv1, ax))
    env.run(until=3600)
    print(time.time()-start)
    if PLOT:
        plt.show()

if __name__ == "__main__":
    main()