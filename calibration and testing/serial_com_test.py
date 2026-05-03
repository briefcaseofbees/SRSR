import serial
import time

PORT = '/dev/ttyUSB0'  # may need to be configured on a bot-by-bot basis

try:
    # attempt to connect to the arduino nano using baudrate of 9600
    ser = serial.Serial(PORT, 9600, timeout=1)
    print(f"Opened: {PORT}") 
    print("Attempting to read distance from Arduino...")
    
    time.sleep(2)
    ser.reset_input_buffer()

    while True:
        if ser.in_waiting > 0:
            print(f"\n{ser.in_waiting} bytes available...")
            # Read line from Arduino
            line = ser.readline().decode('utf-8').rstrip()  # grab the serial output from the arduino nano
            print(f"received {line}...")
        
        else:
            print(".", end="", flush=True)

        time.sleep(0.01)  # small delay to prevent CPU hogging

except serial.SerialException as e:
    print(f"Error opening serial port: {e}")

except KeyboardInterrupt:
    print("\nStopping...")
    ser.close()

# EOF
