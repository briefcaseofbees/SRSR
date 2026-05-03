# rotation_primitives.py
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time
import JETBOT_CONSTANTS as JC

# Setup motors
motorhat = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
left_motor = motorhat.getMotor(1)
right_motor = motorhat.getMotor(2)

def spin(desired_angle:int=90, motor_speed=JC._MOTOR_SPEED_ROTATION_CONSTANT, rotation_primitives=JC._ROTATION_PRIMITIVES):
    print("\n" + "=" * 50)
    print(f"DESIRED ANGLE {desired_angle} ROTATION TEST")
    print("=" * 50)

    left_motor.setSpeed(motor_speed)
    right_motor.setSpeed(motor_speed)

    print(f"assigning rotation primitives for desired angle {desired_angle}deg...")

    spin_times = []
    original_angle = desired_angle  # hold onto for final report

    while desired_angle > 0:
        if desired_angle - 180 >= 0:
            spin_times.append(rotation_primitives[3])
            desired_angle -= 180
            print(f"added {rotation_primitives[3]} to spin_times")
        elif desired_angle - 90 >= 0:
            spin_times.append(rotation_primitives[2])
            desired_angle -= 90
            print(f"added {rotation_primitives[2]} to spin_times")
        elif desired_angle - 45 >= 0:
            spin_times.append(rotation_primitives[1])
            desired_angle -= 45
            print(f"added {rotation_primitives[1]} to spin_times")
        elif int(desired_angle) in [x for x in range(6,36)]:
            spin_times.append(rotation_primitives[0])
            desired_angle -= 22.5
            print(f"added {rotation_primitives[0]} to spin_times")
        else:
            print(f"determined spin times {spin_times} (each in seconds) for {original_angle}deg, with remainer {desired_angle}")
            break  # angle has been reduced to primitives as much as it can!

    input("Press Enter when ready...")

    while spin_times:
        print(f"waiting for a second...")
        time.sleep(1.5)
        print(f"executing {spin_times[-1]} rotation primitive")
        left_motor.run(Adafruit_MotorHAT.FORWARD)
        right_motor.run(Adafruit_MotorHAT.BACKWARD)
        print("Motors spinning...")
        time.sleep(spin_times[-1])  # run motors for duration given
        # Stop motors
        left_motor.run(Adafruit_MotorHAT.RELEASE)
        right_motor.run(Adafruit_MotorHAT.RELEASE)
        print("\nSTOPPED!")
        spin_times.pop()  # remove last item in spin_times


# Run calibration
print("JetBot Rotation Primitive Angles Test")
print("This will test how effective the primitive angle times I've derived translate to desired angles...")

# Option to do multiple tests
user_desired_angle = int(input("\nWhat is the desired angle? (in degrees): "))

print(f"\n--- Test for {user_desired_angle} ---")
spin(user_desired_angle)

