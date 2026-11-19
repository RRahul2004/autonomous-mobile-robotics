import sys
from math import exp

# Type of planner
POINT_PLANNER=0; TRAJECTORY_PLANNER=1



class planner:
    def __init__(self, type_):

        self.type=type_

    
    def plan(self, goalPoint=[-1.0, -1.0], trajectory="parabola"):

        if self.type==POINT_PLANNER:
            return self.point_planner(goalPoint)

        elif self.type==TRAJECTORY_PLANNER:
            return self.trajectory_planner(trajectory)


    def point_planner(self, goalPoint):
        x = goalPoint[0]
        y = goalPoint[1]
        return x, y

    # TODO Part 6: Implement the trajectories here
    def trajectory_planner(self, trajectory="parabola", step=0.05):
        # the return should be a list of trajectory points: [ [x1,y1], ..., [xn,yn]]

        if trajectory == "parabola":
            # y = x^2 for x in [0.0, 1.5]
            x_max = 1.5
            f = lambda x: x**2

        elif trajectory == "sigmoid":
            # y = 2/(1 + e^(-2x)) - 1 for x in [0.0, 2.5]
            x_max = 2.5
            f = lambda x: 2 / (1 + exp(-2 * x)) - 1

        else:
            print("Error! this trajectory doesn't exist", file=sys.stderr)
            return None

        n = round(x_max / step)
        return [[i * step, f(i * step)] for i in range(n + 1)]

