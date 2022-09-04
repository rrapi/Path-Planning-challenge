# Path-Planning-challenge

N-link Robot that reaches M waypoints with Inverse Kinematics

This program could fit different types of manipulators just by editing the DH parameters (in this case I used the UR5e from the Universal Robot)

The goal is to reach the final waypoint (E), starting from the initial point (A) and passing through different intermediate waypoints (B, C, D), moving in a "step by step" basis (in this case each step is a 10 cm distance), by limiting the end-effecor (eef) to pass a certain "bound region". The eef reaches each waypoint with x-axis parallel to the segment from actual pose to the next waypoint. 
Waypoimts are specified in a file where every row is a 3D point (x, y, z) and each coordinate is separated by space.

The result of the simulation tests are shown in the "results" folder.


## Simulation Test

    python3 src/test.py --filename=<input_file> --bound_region=<bound_limit> --step=<step_move> --plot=<True/False>
    
<input_file>:       input filepath of waypoints 

<bound_limit>:      bound limited region for robot moving 

<step_move>:        robot moving step at each time

--plot:             argument for plotting results

# Example

    python3 src/test.py --filename='waypoints.txt' --bound_region=0.15 --step=0.1 --plot=True

    


