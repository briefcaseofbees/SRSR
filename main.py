import serial
import time
import numpy as np

import OccupancyGrid    # import statement for Occupancy-Grid functions, etc.
import Odometry         # import statement for Odometry-related functions, etc.
import JETBOT_CONSTANTS as JC

# [SETTINGS]
## HARDWARE SETTINGS
PORT = '/dev/ttyUSB0'       # the lower right USB port on the JetBots

OG = OccupancyGrid.OccupancyGrid(starting_edge_dim=10, resolution=8)  # set up occupancy grid for this run

try:
    # setup JETBOT for movement
    JetBot_Movement = Odometry.Odometry(JC._MOTOR_SPEED_MOVEMENT_CONSTANT_LEFT,
                                        JC._MOTOR_SPEED_MOVEMENT_CONSTANT_RIGHT,
                                        JC._ROTATION_PRIMITIVES,
                                        JC._MOTOR_SPEED_ROTATION_CONSTANT,
                                        JC._GYROSCOPE_SCALE_FACTOR,
                                        JC._LINEAR_SPEED_CM_S,
                                        starting_angle=359)

    for i in range(32):

        dist_reading = float(JetBot_Movement.read_dist())

        if dist_reading == 0.0:
                continue
        
        OG.update_square_confidences(distance_reading=dist_reading,
                                     current_position=[JetBot_Movement.get_pose()['y'],
                                                       JetBot_Movement.get_pose()['x']],
                                     current_heading_degs=JetBot_Movement.get_pose()['heading_deg'])
        
        
        JetBot_Movement.turn_degrees(rotation_primitives=[0.3])

        time.sleep(1)
        print(OG.occupancy_grid)

except KeyboardInterrupt:
    print("\nStopping...")

# EOF