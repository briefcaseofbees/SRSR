"""
CALIBRATE_DISTANCE.py

Given a distance that the user wants the JetBot to go (at a constant 
motor speed) this script will help to determine what the linear speed
value is-- the user must adjust the linear speed factor until the JetBot 
distance read-out matches the actual distance travelled.
"""

# imports
from Adafruit_MotorHAT import Adafruit_MotorHAT

import serial
import time
import math
from Odometry import Odometry
import JETBOT_CONSTANTS as JC

desired_distance = int(input("How far do you want the JetBot to travel per second? (in cm): "))

print(f"Calibrating linear speed-per-second constant for desired distance={desired_distance}cm; NOTE: edit \"_LINEAR_SPEED_CM_S\" in JETBOT_CONSTANTS.py until actual distance matches desired distance...")
print("="*50)

# Setup Serial, and MotorHAT on JetBot
# serial setup
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # let serial settle

# setup MotorHAT (mh=hardware address and comms)
mh = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
left_motor = mh.getMotor(1)
right_motor = mh.getMotor(2)

# setup Odometry with current constants in JETBOT_CONSTANTS
odo = Odometry(ser, JC._MOTOR_SPEED_MOVEMENT_CONSTANT_LEFT, JC._MOTOR_SPEED_MOVEMENT_CONSTANT_RIGHT, JC._GYROSCOPE_SCALE_FACTOR, JC._LINEAR_SPEED_CM_S)

print("\n" + "="*50)
print(f"TEST: Drive Forward {desired_distance}cm")
print("="*50)

input("Press Enter to start (robot will drive forward)...")

# Start motors
left_motor.setSpeed(odo.MOTORSPEED_CONSTANT_LEFT)
right_motor.setSpeed(odo.MOTORSPEED_CONSTANT_RIGHT)
left_motor.run(Adafruit_MotorHAT.BACKWARD)
right_motor.run(Adafruit_MotorHAT.BACKWARD)

print("\nMoving forward...")
print(f"{'Time':>6} | {'X (cm)':>8} | {'Y (cm)':>8} | {'Heading':>8} | {'Distance':>10}")
print("-"*60)

start_time = time.time()
print("before")
print(odo.coordinates)
odo.reset()
print("after")
print(odo.coordinates)

while True:
    try:
        ser.reset_input_buffer()
        line = ser.readline().decode('utf-8').strip()
    
        if line:
            values = line.split(',')
            if len(values) == 7:
                gyro_z = float(values[6])
                
                # Update odometry
                odo.update(is_moving=True, gyro_z_raw=gyro_z)
                
                pose = odo.get_pose()
                distance = math.sqrt(pose['x']**2 + pose['y']**2)
                elapsed = time.time() - start_time
                
                # Print status every 0.5 seconds
                if int(elapsed * 10) % 5 == 0:
                    print(f"{elapsed:6.1f} | {pose['x']:8.1f} | {pose['y']:8.1f} | "
                            f"{pose['heading_deg']:8.1f} | {distance:10.1f}")
                
                # Stop when target reached
                if distance >= desired_distance:
                    break
    
    except Exception as e:
        print(f"Error: {e}")
    
    time.sleep(0.1)

# Stop motors
left_motor.run(Adafruit_MotorHAT.RELEASE)
right_motor.run(Adafruit_MotorHAT.RELEASE)

final_pose = odo.get_pose()
final_distance = math.sqrt(final_pose['x']**2 + final_pose['y']**2)

print(f"\n{'='*60}")
print("COMPLETED!")
print(f"{'='*60}")
print(f"Target distance: {desired_distance} cm")
print(f"Actual distance: {final_distance:.1f} cm")
print(f"Error: {abs(final_distance - desired_distance):.1f} cm")
print(f"Final position: ({final_pose['x']:.1f}, {final_pose['y']:.1f}) cm")
print(f"Final heading: {final_pose['heading_deg']:.1f}°")
print(f"Heading drift: {abs(final_pose['heading_deg']):.1f}° from straight")

print(f"\n{'='*60}")
print(f"Adjust \"_LINEAR_SPEED_CM_S\" in Constants file now...")
