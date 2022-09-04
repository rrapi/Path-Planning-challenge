# Path-Planning-challenge
N-link Robot that reaches M waypoints with Inverse Kinematics

This program could fit different types of manipulators just by editing the DH parameters (in this case I used the UR5e from the UNiverdal Robot)

The aim is to reach different waypoints (specified in a document where each line is a 3D point and each coordinate is separated by space) by limiting the end-effecor to pass a certain "bound region" and moving each time of "step" size (for example 10 cm). The eef reach each waypoint with x-axis parallel to the segment from actual pose to the next waypoint.

The reult of the simulation test are shown in the "results" folder.


## Simulation Test

    python3 src/test.py --filename=<input_file> --bound_region=<bound_limit> --step=<step_move> --plot=<True/False>
    
<input_file>:       waypoints input list file path

<bound_limit>:      bound limited region for robot moving 

<step_move>:        robot moving step at each time

--plot:             argument for plotting results

# Example

    python3 src/test.py --filename='waypoints.txt' --bound_region=0.15 --step=0.1 --plot=True

    


