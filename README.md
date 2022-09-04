# Path-Planning-challenge
N-link Robot that reaches M waypoints with Inverse Kinematics



## Simulation Test

    python3 src/test.py --filename=<input_file> --bound_region=<bound_limit> --step=<step_move> --plot=<True/False>
    

    
    python3 src/test.py --filename='waypoints.txt' --bound_region=0.15 --step=0.1 --plot=True

    
<input_file>:       waypoints input list file path

<bound_limit>:      bound limited region for robot moving 

<step_move>:        robot moving step at each time

--plot:             argument for plotting results

