import time
import math
import serial
from Adafruit_MotorHAT import Adafruit_MotorHAT

class Odometry:
    def __init__(self,
                 motor_speed_constant_left,
                 motor_speed_constant_right,
                 rotation_primitives,
                 rotation_speed_constant,
                 gyro_scale_factor,
                 linear_speed,
                 current_x = 0.0,
                 current_y = 0.0,
                 starting_angle:int = 0):

        # SETUP SERIAL CONNECTION
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)

        # COLLECT BOTH MOTORS
        self.mh = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)

        # LEFT MOTOR SETUP
        self.left_motor = self.mh.getMotor(1)
        self.MOTORSPEED_CONSTANT_LEFT = motor_speed_constant_left

        # RIGHT MOTOR SETUP
        self.right_motor = self.mh.getMotor(2)
        self.MOTORSPEED_CONSTANT_RIGHT = motor_speed_constant_right

        # COORDINATES AND HEADING
        self.coordinates = [current_x, current_y]
        self.current_heading_rads = math.radians(starting_angle)
        
        # SET OTHER CONSTANTS FOR THIS PARTICULAR JETBOT
        self.LINEAR_SPEED = linear_speed
        self.GYRO_SCALE = gyro_scale_factor
        self.GYRO_BIAS = self.calibrate_gyro_bias()
        self.ROTATION_CONSTANT = rotation_speed_constant
        self.ROTATION_PRIMITIVES = rotation_primitives

        # SET INITIAL START TIME
        self.last_update_time = time.time()

    def read_dist(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) == 7:
                    return float(values[0])
        except:
            pass
            print("Exception")
        print("0.0 return")
        return 0.0

    def read_gyro(self):
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) == 7:
                    return -float(values[6])  # REVERSE WHICH DIRECTION IS POSITIVE ANGLE TURN
        except:
            pass
            print("Exception")
        print("0.0 return")
        return 0.0

    def calibrate_gyro_bias(self):
        samples = []
        self.ser.reset_input_buffer()  # clean input buffer before collecting samples for bias calculation

        for i in range(50):  # 5 seconds of samples
            samples.append(self.read_gyro())
            time.sleep(0.1)

        bias = sum(samples) / len(samples)  # calc bias
        print(f"Gyro bias: {bias:.2f}°/s")  # display bias value

        return bias

    def update(self, is_moving:bool, gyro_z_raw:float):
        current_time = time.time()
        dt = current_time - self.last_update_time

        self.last_update_time = current_time
        
        # Correct and scale gyro reading
        gyro_z = (gyro_z_raw - self.GYRO_BIAS) * self.GYRO_SCALE
        
        # Update heading (gyro is in deg/s, integrate to get degrees)
        heading_change_deg = gyro_z * dt
        heading_change_rad = math.radians(heading_change_deg)

        self.current_heading_rads += heading_change_rad * 1.25
        self.current_heading_rads
        
        # Update position if moving
        if is_moving:
            distance = self.LINEAR_SPEED * dt  # cm
            self.coordinates[0] += distance * math.cos(self.current_heading_rads)
            self.coordinates[1] += distance * math.sin(self.current_heading_rads)

    def get_pose(self):

        return {
            'x': self.coordinates[0],
            'y': self.coordinates[1],
            'heading_deg': math.degrees(self.current_heading_rads) % 360,
            'heading_rad': self.current_heading_rads
        }
    
    def reset(self, x=0.0, y=0.0, heading_deg=0.0):
        self.coordinates = [x,y]
        self.current_heading_rads = math.radians(heading_deg)
        self.last_update_time = time.time()

    def construct_angle_times(self, desired_angle) -> list:
        print(f"assigning rotation primitives for desired angle {desired_angle}deg...")

        # need to shift desired angle using current heading in degrees

        current_heading_degs = math.degrees(self.current_heading_rads)
        target_angle = (current_heading_degs + desired_angle) % 360
        print(current_heading_degs)

        spin_times = []
        original_angle = target_angle  # hold onto for final report

        while target_angle > 0:
            if target_angle - 180 >= 0:
                spin_times.append(self.ROTATION_PRIMITIVES[3])
                target_angle -= 180
                print(f"added {self.ROTATION_PRIMITIVES[3]} to spin_times")
            elif target_angle - 90 >= 0:
                spin_times.append(self.ROTATION_PRIMITIVES[2])
                target_angle -= 90
                print(f"added {self.ROTATION_PRIMITIVES[2]} to spin_times")
            elif target_angle - 45 >= 0:
                spin_times.append(self.ROTATION_PRIMITIVES[1])
                target_angle -= 45
                print(f"added {self.ROTATION_PRIMITIVES[1]} to spin_times")
            elif int(target_angle) in [x for x in range(6, 36)]:
                spin_times.append(self.ROTATION_PRIMITIVES[0])
                target_angle -= 22.5
                print(f"added {self.ROTATION_PRIMITIVES[0]} to spin_times")
            else:
                print(
                    f"determined spin times {spin_times} (each in seconds) for {original_angle}deg, with remainer {target_angle}")
                break  # angle has been reduced to primitives as much as it can!

        return spin_times

    def turn_degrees(self, rotation_primitives:list =[], desired_angle:int=90, ):
        # determine which is faster: turning left, or turning right
        # compare the delta between a left and right turn

        self.last_update_time = time.time()

        time.sleep(0.5)

        current_heading_degs = int(math.degrees(self.current_heading_rads))
        left_turn_delta = abs(current_heading_degs - desired_angle) % 360

        left_turn_delta = 179

        # set motor rotation speed (same regardless of direction)
        self.left_motor.setSpeed(self.ROTATION_CONSTANT); self.right_motor.setSpeed(self.ROTATION_CONSTANT)

        if left_turn_delta > 180:  # left turn more than 180 degrees, go right instead

            if len(rotation_primitives) == 0:
                # determine rotation primitives for turn...
                rotation_times = self.construct_angle_times(left_turn_delta)
            else:
                rotation_times = rotation_primitives

            while len(rotation_times) > 0:  # don't stop until all rotation times are spent

                # MOTORS TURN RIGHT
                self.left_motor.run(Adafruit_MotorHAT.BACKWARD); self.right_motor.run(Adafruit_MotorHAT.FORWARD)

                start_time = time.time()
                while time.time() - start_time < rotation_times[-1]:
                    gyro_z = self.read_gyro()
                    self.update(is_moving=False, gyro_z_raw=gyro_z)

                # STOP MOTORS FOR THIS PRIMITIVE
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)

                rotation_times.pop()  # pop the rotation time we just used up
                time.sleep(0.5)  # slight pause between rotations

        else:  # go left, or if rotation is 180 exactly (left and right turns don't matter)

            if len(rotation_primitives) == 0:
                # determine rotation primitives for turn...
                rotation_times = self.construct_angle_times(left_turn_delta)
            else:
                rotation_times = rotation_primitives

            while len(rotation_times) > 0:  # don't stop until all rotation times are spent

                # MOTORS TURN LEFT
                self.left_motor.run(Adafruit_MotorHAT.FORWARD); self.right_motor.run(Adafruit_MotorHAT.BACKWARD)

                start_time = time.time()
                while time.time() - start_time < rotation_times[-1]:
                    gyro_z = self.read_gyro()
                    self.update(is_moving=False, gyro_z_raw=gyro_z)

                # STOP MOTORS FOR THIS PRIMITIVE
                self.left_motor.run(Adafruit_MotorHAT.RELEASE)
                self.right_motor.run(Adafruit_MotorHAT.RELEASE)

                rotation_times.pop()  # pop the rotation time we just used up
                time.sleep(0.5)  # slight pause between rotations


    def drive_forward(self, target_distance_cm:int=20):

        start_x = self.coordinates[0]
        start_y = self.coordinates[1]

        # SET MOTOR SPEED (established constants for this jetbot)
        self.left_motor.setSpeed(self.MOTORSPEED_CONSTANT_LEFT)
        self.right_motor.setSpeed(self.MOTORSPEED_CONSTANT_RIGHT)

        distance_traveled = 0  # tracker for how far jetbot has gone

        # MOTORS GO FORWARD (WHICH IS APPARENTLY BACKWARDS?)
        self.left_motor.run(Adafruit_MotorHAT.BACKWARD); self.right_motor.run(Adafruit_MotorHAT.BACKWARD)
        self.last_update_time = time.time()
        while distance_traveled < target_distance_cm:
            gyro_z = self.read_gyro()
            self.update(is_moving=True, gyro_z_raw=gyro_z)

            # Calculate distance from start of this leg
            dx = self.coordinates[0] - start_x
            dy = self.coordinates[1] - start_y
            distance_traveled = math.sqrt(dx ** 2 + dy ** 2)

        # STOP MOTORS
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)

#EOF
