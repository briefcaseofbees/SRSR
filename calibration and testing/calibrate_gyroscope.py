# gyro_calibration.py
from Adafruit_MotorHAT import Adafruit_MotorHAT
import serial
import time

# Setup motors
motorhat = Adafruit_MotorHAT(addr=0x60, i2c_bus=1)
left_motor = motorhat.getMotor(1)
right_motor = motorhat.getMotor(2)

# Setup serial to Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)
ser.reset_input_buffer()

def calibrate_gyro_bias(ser):
    """Measure gyro zero offset"""
    print("Calibrating gyro bias (keep robot still)...")
    samples = []
    
    for i in range(50):  # 5 seconds of samples
        try:
            line = ser.readline().decode('utf-8').strip()
            if line:
                values = line.split(',')
                if len(values) == 7:
                    gyro_z = float(values[6])
                    samples.append(gyro_z)
        except:
            pass
        time.sleep(0.1)
    
    bias = sum(samples) / len(samples)
    print(f"Gyro bias: {bias:.2f}°/s")
    return bias

GYRO_BIAS = calibrate_gyro_bias(ser)

def read_gyro():
    """Read gyro_z from Arduino"""
    try:
        line = ser.readline().decode('utf-8').strip()
        if line:
            values = line.split(',')
            if len(values) == 7:
                gyro_z = float(values[6])  # Last value is gyro_z
                return gyro_z - GYRO_BIAS
    except:
        pass
    return 0.0



def spin_test(duration=2.0, spin_speed=51):
    print("\n" + "="*50)
    print("GYRO CALIBRATION - SPIN TEST")
    print("="*50)
    
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
    print("Motors spinning up...")
    time.sleep(1.0)  # Wait for steady rotation
    
    # Clear any old readings from buffer
    ser.reset_input_buffer()
    
    print("Recording data NOW!")

    # NOW RECORD

    total_rotation = 0.0
    start_time = time.time()
    last_time = start_time
    reading_count = 0
    
    while time.time() - start_time < duration:
        try:
            line = ser.readline().decode('utf-8').strip()
            
            if line:
                csv = line.split(',')
                if len(csv) == 7:
                    current_time = time.time()
                    dt = current_time - last_time
                    last_time = current_time
                    
                    gyro_z_raw = float(csv[6])
                    gyro_z = gyro_z_raw - GYRO_BIAS
                    
                    total_rotation += gyro_z * dt
                    reading_count += 1
        except:
            pass
    
    # Stop motors
    left_motor.run(Adafruit_MotorHAT.RELEASE)
    right_motor.run(Adafruit_MotorHAT.RELEASE)
    
    print("\nSTOPPED!")
    print(f"Readings collected: {reading_count}")
    print(f"Gyro reported: {total_rotation:.1f}°")
    
    actual = float(input("\nActual rotation (degrees): "))
    
    scale_factor = actual / total_rotation
    
    print(f"\n{'='*50}")
    print("RESULTS:")
    print(f"{'='*50}")
    print(f"Gyro reported: {total_rotation:.1f}°")
    print(f"Actual rotation: {actual:.1f}°")
    print(f"Scale factor: {scale_factor:.3f}")
    
    return scale_factor


# Run calibration
print("JetBot Gyroscope Calibration")
print("This will spin the robot to test gyro accuracy")

# Option to do multiple tests
scales = []
num_tests = int(input("\nHow many tests? (recommend 2-3): "))

for i in range(num_tests):
    print(f"\n--- Test {i + 1}/{num_tests} ---")
    scale = spin_test(duration=5.0, spin_speed=30)
    scales.append(scale)

    if i < num_tests - 1:
        input("\nPress Enter for next test...")

# Average results
avg_scale = sum(scales) / len(scales)

print(f"\n{'=' * 50}")
print("FINAL CALIBRATION")
print(f"{'=' * 50}")
print(f"Individual tests: {[f'{s:.3f}' for s in scales]}")
print(f"Average scale factor: {avg_scale:.3f}")
print(f"\n✓ Use this in your config:")
print(f'  "gyro_scale_factor": {avg_scale:.3f}')