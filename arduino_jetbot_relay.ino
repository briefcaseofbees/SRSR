// functionality libraries
#include <Wire.h>
#include <math.h>

// sensor libraries
#include <MPU6050.h>
#include <HCSR04.h>

// sensor variables
MPU6050 mpu; // defaults to using 4,5 on Nano == SCK,SDA
UltraSonicDistanceSensor distanceSensor(6,7);  // pins 6,7 on Nano == Trig,Echo

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Initializing MPU6050...");
  
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connected successfully!");
  } else {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  
  delay(1000);
}

void loop() {

  // MPU work
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Ultrasonic work
    float current_distance = round(distanceSensor.measureDistanceCm());
    String distance_reading = "DIST: " + String(current_distance);  // 'DIST' for indentifying data comes from HC-SR04

  // Convert to g's and degrees/second
  float accel_x = ax / 16384.0;
  float accel_y = ay / 16384.0;
  float accel_z = az / 16384.0;
  float gyro_x = gx / 131.0;
  float gyro_y = gy / 131.0;
  float gyro_z = gz / 131.0;
  
  // build the CSV line for interpretation by the jetbot...
  // serial line format is the following:
  // < distance, accelX, accelY, accelZ, gyroX, gyroY, gyroZ >

  Serial.print(current_distance, 1);
  Serial.print(",");
  Serial.print(accel_x, 2);
  Serial.print(",");
  Serial.print(accel_y, 2);
  Serial.print(",");
  Serial.print(accel_z, 2);
  Serial.print(",");
  Serial.print(gyro_x, 1);
  Serial.print(",");
  Serial.print(gyro_y, 1);
  Serial.print(",");
  Serial.println(gyro_z, 1);
  
  delay(100); // delay should reflect what the HCSR04 needs in terms of serial updates (10mHz updates); the IMU will transfer at that time interval as a result
}
