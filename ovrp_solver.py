__author__ = 'nick'


# Simple path optimiser that accepts an entry and exit point in literature it is defined as Open Vehicle Routing Problem

import numpy as np
from gurobipy import *

HAS_GUROBI = True

# Euclidean distance between two points


def distance(points, i, j):
    return np.linalg.norm(points[i, :] - points[j, :])


def ovrp_solver(cities, start=None, finish=None):

    """
    Open vehicle routing problem solver for a single vehicle using the Gurobi MILP optimiser.
    :param cities: List of points to optimise the route. Numpy NxM array, where M is in [2, 3] (dimension of the points)
    :param start: Optional starting point for the tour. If none is provided the first point of the array is chosen
    :param finish: Optional ending point of the tour. If none is provided the last point of the array is chosen
    :return: Returns the route the cost and the model.
    """

    # Check for default values
    if start is None:
        start = 0
    if finish is None:
        finish = len(cities)-1

    # Number of points
    n = len(cities)

    m = Model()

    # Create model variables
    vars = {}
    for i in range(n):
        for j in range(n):
            if i == j:
                dist = 0
                vars[i,j] = m.addVar(obj=dist, vtype=GRB.BINARY, name='e'+str(i)+'_'+str(j), ub=0)
            else:
                dist = distance(cities, i, j)
                vars[i,j] = m.addVar(obj=dist, vtype=GRB.BINARY, name='e'+str(i)+'_'+str(j))

    m.update()

    uVars = {}
    for i in range(n):
        uVars[i] = m.addVar(vtype=GRB.INTEGER, name='u'+str(i))
    m.update()

    # None exits the finish point
    m.addConstr(quicksum(vars[finish,j] for j in range(n)) == 0)
    m.update()

    # From all other points someone exits
    for i in range(n):
        if i != finish:
            m.addConstr(quicksum(vars[i,j] for j in range(n)) == 1)
    m.update()

    # None enters the starting point
    m.addConstr(quicksum(vars[j,start] for j in range(n)) == 0)
    m.update()

    # To all other points someone enters
    for i in range(n):
        if i != start:
            m.addConstr(quicksum(vars[j,i] for j in range(n)) == 1)
    m.update()

    # Sub-tour elimination constraint
    for i in range(n):
        for j in range(n):
            if (i != j):
                m.addConstr(uVars[i]-uVars[j]+n*vars[i,j] <= n-1)
    m.update()

    m._vars = vars
    m._uVars = uVars

    m.optimize()
    try:
        solution = m.getAttr('X', vars)
        u = m.getAttr('X', uVars)
        selected = [(i,j) for i in range(n) for j in range(n) if solution[i,j] > 0.5]

        mat = np.zeros((n,n))

        for k,v in solution.iteritems():
            mat[k[0],k[1]] = v

        print(mat)
        print(selected)
        print(u)

        route = np.zeros(n, dtype=np.int)

        for k, v in u.iteritems():
            route[v] = int(k)

        return route, m.objVal, m
    except GurobiError:
        return 0, 0, 0


def main():
    import time
    import matplotlib as mpl
    import matplotlib.pyplot as plt

    mpl.style.use('bmh')
    mpl.rcParams['figure.figsize'] = (12, 6)
    np.random.seed(47)

    def __plot_problem(ips, tsp_route, total_cost):
        idx = tsp_route #[int(city.split('c_')[1]) for city in tsp_route]
        ips_route = ips[idx, :]

        fig, ax = plt.subplots()
        ax.plot(ips[:, 1], ips[:, 0], 'o', label='inspection points')
        ax.plot(ips_route[:, 1], ips_route[:, 0], 'r-', alpha=0.3)

        for n in xrange(len(idx)):
            x, y = ips[n, 1], ips[n, 0]
            xt, yt = x - 0.10 * np.abs(x), y - 0.10 * np.abs(y)

            ax.annotate('#%d' % n, xy=(x, y), xycoords='data', xytext=(xt,yt))

        for k, n in enumerate(idx):
            x, y = ips[n, 1], ips[n, 0]
            xt, yt = x + 0.05 * np.abs(x), y + 0.05 * np.abs(y)

            ax.annotate(str(k), xy=(x, y), xycoords='data', xytext=(xt,yt))

        ax.axis('equal')

        ax.set_xlabel('East (m)')
        ax.set_ylabel('North (m)')
        ax.set_title('TSP Problem')

        return fig, ax

    # generate random problem
    n = 20
    points = np.random.randint(-50, 50, (n, 2))
    cities = ['c_{}'.format(k) for k in xrange(n)]

    # standard cost
    distances = np.zeros((n, n))

    for k in xrange(n):
        for p in xrange(n):
            distances[k, p] = np.linalg.norm(points[k, :] - points[p, :])

    if HAS_GUROBI:
        # solve using the Gurobi solver
        st = time.time()
        tsp_route, total_cost, model = ovrp_solver(points, 5, 6)
        dt = time.time() - st

        print('Gurobi Solver')
        print('Time to Solve: %.2f secs' % dt)
        print('Cost: %.3f' % total_cost)
        print('TSP Route: %s\n' % tsp_route)



        fig, ax = __plot_problem(points, tsp_route, total_cost)
        plt.show()


if __name__ == '__main__':
    main()
