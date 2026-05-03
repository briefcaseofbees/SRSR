# rotation_primitives.py
from Adafruit_MotorHAT import Adafruit_MotorHAT
import time

# Setup motors
motorhat = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
left_motor = motorhat.getMotor(1)
right_motor = motorhat.getMotor(2)

def spin(duration=1.0, spin_speed=51):
    print("\n" + "=" * 50)
    print(f"PRIMITIVE ANGLE DETERMINATION-- for {duration} seconds")
    print("=" * 50)

    input("Press Enter when ready...")

    print("Starting in 3...")
    time.sleep(1)
    print("2...")
    time.sleep(1)
    print("1...")
    time.sleep(1)
    print("SPINNING!")

    # Start motors
    left_motor.setSpeed(spin_speed)
    right_motor.setSpeed(spin_speed)
    left_motor.run(Adafruit_MotorHAT.FORWARD)
    right_motor.run(Adafruit_MotorHAT.BACKWARD)

    # *** LET MOTORS STABILIZE ***
    print("Motors spinning...")
    time.sleep(duration)  # run motors for duration given

    # Stop motors
    left_motor.run(Adafruit_MotorHAT.RELEASE)
    right_motor.run(Adafruit_MotorHAT.RELEASE)

    print("\nSTOPPED!")

    actual = float(input("\nActual rotation (degrees): "))

    return actual


# Run calibration
print("JetBot Rotation Primitive Derivation")
print("This will allow primitive angles to be found based on duration of motor running over a duration of time")

# Option to do multiple tests
rotations = []
num_tests = int(input("\nHow many tests? (recommend 10): "))
duration_seconds = float(input("\nHow long? (in seconds): "))
motor_strength = int(input(f"\nMotor Strength? (30/255=0.12; 51/255=0.2; 100/255=0.4): "))

for i in range(num_tests):
    print(f"\n--- Test {i + 1}/{num_tests} ---")
    rotation_actual = spin(duration=duration_seconds, spin_speed=motor_strength)
    rotations.append(rotation_actual)

    if i < num_tests - 1:
        input("\nPress Enter for next test...")

# Average results
avg_rotation = sum(rotations) / len(rotations)

print(f"rotation primitive for motor strength {motor_strength} at {duration_seconds}: {avg_rotation:.3f}")