"""
SQUARE_TEST.py
The Square Test verifies that based on the determined constants-- 
that the JetBot is able to go in a 20cm x 20cm square (rotating 90deg 
at each turn) and will end up at the correct heading, and position.
"""

# imports
import time
import math
import serial
from Adafruit_MotorHAT import Adafruit_MotorHAT
from Odometry import Odometry
import JETBOT_CONSTANTS as JC


class SquareTest:
    def __init__(self):
        # Setup hardware
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        time.sleep(2)
        
        self.mh = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
        self.left_motor = self.mh.getMotor(1)
        self.right_motor = self.mh.getMotor(2)
        
        
        self.odo = Odometry(self.ser, JC._MOTOR_SPEED_MOVEMENT_CONSTANT_LEFT, JC._MOTOR_SPEED_MOVEMENT_CONSTANT_RIGHT, JC._GYROSCOPE_SCALE_FACTOR, JC._LINEAR_SPEED_CM_S)
    
    def read_gyro(self):
        """Read gyro from sensor"""
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) == 7:
                    print(f"{float(values[6])}")
                    return float(values[6])
        except:
            pass
            print("Exception")
        print("0.0 return")
        return 0.0
    
    def drive_forward(self, target_distance_cm):
        """Drive forward until target distance reached"""
        print(f"Driving forward {target_distance_cm} cm...")
        
        time.sleep(1)

        start_x = self.odo.coordinates[0]
        start_y = self.odo.coordinates[1]
        
        print(f"starting x and y: {start_x, start_y}")

        # Start motors
        self.left_motor.setSpeed(JC._MOTOR_SPEED_MOVEMENT_CONSTANT_LEFT)
        self.right_motor.setSpeed(JC._MOTOR_SPEED_MOVEMENT_CONSTANT_RIGHT)
        self.left_motor.run(Adafruit_MotorHAT.BACKWARD)
        self.right_motor.run(Adafruit_MotorHAT.BACKWARD)

        print(f"target distance (cm): {target_distance_cm}")
        distance_traveled = 0
        self.odo.last_update_time = time.time()
        while distance_traveled < target_distance_cm:
            gyro_z = self.read_gyro()
            self.odo.update(is_moving=True, gyro_z_raw=gyro_z)
            
            # Calculate distance from start of this leg
            dx = self.odo.coordinates[0] - start_x
            dy = self.odo.coordinates[1] - start_y
            distance_traveled = math.sqrt(dx**2 + dy**2)
            print(f"distance travelled: {distance_traveled}")
            
            time.sleep(0.1)
        
        # Stop motors
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        
        print(f"    Traveled {distance_traveled:.1f} cm")
        pose = self.odo.get_pose()
        print(f"    Position: ({pose['x']:.1f}, {pose['y']:.1f}) cm, "
              f"Heading: {pose['heading_deg']:.1f}°")
    
    def turn_right_90(self):
        """Turn 90 degrees to the right"""
        print(f"  Turning 90° right...")

        time.sleep(1)

        heading_before = self.odo.get_pose()['heading_deg']
        print(f"  Heading before: {heading_before:.1f}°")
        

        self.left_motor.setSpeed(JC._MOTOR_SPEED_ROTATION_CONSTANT)
        self.right_motor.setSpeed(JC._MOTOR_SPEED_ROTATION_CONSTANT)
        start_time = time.time()
        self.left_motor.run(Adafruit_MotorHAT.BACKWARD)
        self.right_motor.run(Adafruit_MotorHAT.FORWARD)

        gyro_z = self.read_gyro()
        while time.time() - start_time < JC._ROTATION_PRIMITIVES[2]:
            gyro_z = self.read_gyro()
            self.odo.update(is_moving=False, gyro_z_raw=gyro_z)
            
       
        # Stop motors
        self.left_motor.run(Adafruit_MotorHAT.RELEASE)
        self.right_motor.run(Adafruit_MotorHAT.RELEASE)
        
        heading_after = self.odo.get_pose()['heading_deg']

        actual_turn = heading_after - heading_before
        
        print(f"  Heading after: {heading_after:.1f}°")
        print(f"  Actual turn: {actual_turn:.1f}°")

    
    def run_square(self, side_length=500):
        """Drive in a square pattern"""
        print("\n" + "="*60)
        print(f"SQUARE TEST: {side_length} cm sides")
        print("="*60)
        
        input("Place robot at starting position. Press Enter to begin...")
        
        # Drive the square
        for side in range(4):
            print(f"\n--- Side {side + 1} ---")
            
            self.drive_forward(side_length)
            time.sleep(1)  # Pause between movements
            
            if side < 3:  # Don't turn after 4th side
                self.turn_right_90()
                time.sleep(1)
        
        self.turn_right_90()

        # Final resultsi
        final_pose = self.odo.get_pose()
        
        # Calculate errors
        position_error = math.sqrt(final_pose['x']**2 + final_pose['y']**2)

        if final_pose['heading_deg'] // 180 > 0:
            # the JetBot is facing LEFT (between 180-360) (undershoot)
            heading_error = 360 - final_pose['heading_deg']
        else:
            # the JetBot is facing RIGHT (between 0-180) (overshoot)
            heading_error = -final_pose['heading_deg']

        
        
        print(f"\n{'='*60}")
        print("SQUARE COMPLETED!")
        print(f"{'='*60}")
        print(f"Final position: ({final_pose['x']:.1f}, {final_pose['y']:.1f}) cm")
        print(f"Distance from origin: {position_error:.1f} cm")
        print(f"Final heading: {final_pose['heading_deg']:.1f}°")
        print(f"Heading error: {heading_error:.1f}°")
        print(f"\nExpected to return to (0, 0) at 0°")
        print(f"Position accuracy: {100 - (position_error/side_length)*100:.1f}%")

if __name__ == "__main__":
    test = SquareTest()
    test.run_square(side_length=20)  # 1 meter square
    
    print("\nTest complete! Check the results above.")
