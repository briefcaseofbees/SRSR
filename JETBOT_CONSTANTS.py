_MOTOR_SPEED_MOVEMENT_CONSTANT_LEFT = 40        # constant for power of left motor on jetbot (constant/255 gives % of total power)
_MOTOR_SPEED_MOVEMENT_CONSTANT_RIGHT = 37       # constant for power of right motor on jetbot (constant/255 gives % of total power)
_LINEAR_SPEED_CM_S = 17.9                       # distance travelled per second at _MOTOR_SPEED_MOVEMENT_CONSTANT
_GYROSCOPE_SCALE_FACTOR = 1.001                 # constant to correct for gyroscopic weirdness
_MOTOR_SPEED_ROTATION_CONSTANT = 25             # motor speed constant for consistent turning (constant/255 gives % of total power)
_ROTATION_PRIMITIVES = [0.3, 0.55, 1.15, 2.4]     # number of seconds at _MOTOR_SPEED_ROTATION_CONSTANT to turn [22.5deg, 45deg, 90deg, 180deg]