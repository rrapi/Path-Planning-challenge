
import math
from NLinkArm3d import NLinkArm
import numpy as np
import string
import argparse

# Instantiate the parser
parser = argparse.ArgumentParser(description='Simulation test parameters')


# Required arguments
parser.add_argument('--filename', type=str, help='A required string argument for waypoints path input.')
parser.add_argument('--bound_region', type=float,
                    help='A required float bound argument for limiting the robot from ideal path.')
parser.add_argument('--step', type=float, help='A required float step argument for robot moving.')


# Optional arguments
parser.add_argument('--plot', type=bool, default=True,
                    help='An optional boolean argument for plotting the results.')


def main():
    args = parser.parse_args()

    filename = args.filename
    bound_region = args.bound_region
    step = args.step
    plot = args.plot

    print("Argument values:")
    print("Filename= ", args.filename)
    print("Bound limit= ", args.bound_region)
    print("Step move= ", args.step)
    print("Plot= ", args.plot)

    # labels = ["A", "B", "C", "D", "E"]
    # waypoints = [[0.3,-0.2,0.], [0.5,-0.2,0.], [0.6,0.,0.2], [0.6,0.,0.4], [0.2,0.0,0.5]]

    labels = []
    waypoints = []

    # Reading from file the waypoints desired to reach
    f = open(filename, 'r')
    line = f.readline()
    count = 0
    
    while line:
        point = []
        count += 1

        line = line.strip().split(" ")

        assert(len(line) == 3)

        for i in range(3):
            point.append(float(line[i]))

        waypoints.append(point)
        labels.append(str(list(string.ascii_uppercase)[count-1]))

        line = f.readline()
    
    f.close()

    # print(waypoints)
    # print(labels)

    N = len(waypoints)

    print("Start solving Inverse Kinematics for {} waypoints.".format(N))
   

    # init NLinkArm with Denavit-Hartenberg parameters of UR5e robot (Universal Robot official parameters)
    n_link_arm = NLinkArm([[0., math.pi / 2, 0., 0.1625],       # link 1
                           [0., 0., -0.425, 0.],                # link 2
                           [0., 0., -0.3922, 0.],               # link 3
                           [0., math.pi / 2, 0., 0.1333],       # link 4
                           [0., -math.pi / 2, 0., 0.0997],      # link 5
                           [0., 0., 0., 0.0996],                # link 6
                           [0., 0., 0., 0.]])                   # tool 0 (eef)


    n_link_arm.path_planning_trajectory(labels=labels, waypoints=waypoints, bound_region=bound_region, step=step, plot=plot)


if __name__ == "__main__":
    main()