__author__ = 'nick'

# Message definitions for process communication


class PlanReqMsg:
    """
    Class that represents a plan request from the planner to the executor.
    """
    target_list = []
    target_order = []

    def __init__(self, targets, order):
        self.target_list = targets
        self.target_order = order


class PlanFeedbackMsg:
    """
    Class that represents a plan feedback for each plan action.
    """
    target_id = 0
    target_class = ""

    def __init__(self, id, classificaton):
        self.target_id = id
        self.target_class = classificaton


class NavReqMsg:
    """
    Dummy request message
    """
    req = True

    def __init__(self):
        pass


class NavUpdateMsg:
    """
    Navigation position update message
    """
    pos = [0, 0, 0]

    def __init__(self, pos):
        self.pos = pos
