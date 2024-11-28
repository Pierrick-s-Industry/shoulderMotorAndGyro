#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>
#define PIN_SG90 33
#define PIN_SG89 32

Servo sg90;
Servo sg89;

Adafruit_MPU6050 mpu;

// Speed variables
float speedX = 0, speedY = 0, speedZ = 0;
unsigned long prevTime = 0; // To track time intervals

// Thresholds to ignore small accelerations
const float ACCEL_THRESHOLD = 0.1; // Minimum significant acceleration in m/s^2


void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 Initialized");
  prevTime = millis(); // Initialize time tracking

  sg90.setPeriodHertz(50); // Fréquence PWM pour le SG90
  sg90.attach(PIN_SG90, 500, 2400); // Largeur minimale et maximale de l'impulsion (en µs) pour aller de 0° à 180°

  sg89.setPeriodHertz(50); // Fréquence PWM pour le SG90
  sg89.attach(PIN_SG89, 500, 2400); // Largeur minimale et maximale de l'impulsion (en µs) pour aller de 0° à 180°
}

void loop() {

  // Rotation de 0 à 180°
  for (int pos = 0; pos <= 180; pos += 1) {
    sg90.write(pos);
    sg89.write(pos);
    delay(10);
  }
 // Rotation de 180° à 0°
  for (int pos = 180; pos >= 0; pos -= 1) {
    sg89.write(pos);
    delay(10);
  }

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Get acceleration data
  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  // Apply threshold to ignore noise
  if (fabs(accelX) < ACCEL_THRESHOLD) accelX = 0;
  if (fabs(accelY) < ACCEL_THRESHOLD) accelY = 0;
  if (fabs(accelZ) < ACCEL_THRESHOLD) accelZ = 0;

  // Calculate time difference (in seconds)
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0; // Convert to seconds
  prevTime = currentTime;

  // Update speeds only if acceleration is significant
  speedX = accelX * deltaTime;
  speedY = accelY * deltaTime;
  speedZ = accelZ * deltaTime;

  // Reset speed if the device is stationary (optional)
  if (accelX == 0 && accelY == 0 && accelZ == 0) {
    speedX = 0;
    speedY = 0;
    speedZ = 0;
  }

  // Print results
  Serial.print("Speed X: ");
  Serial.print(speedX);
  Serial.print(" m/s, Speed Y: ");
  Serial.print(speedY);
  Serial.print(" m/s, Speed Z: ");
  Serial.print(speedZ);
  Serial.println(" m/s");

  // Optional: Calculate resultant speed
  float resultantSpeed = sqrt(speedX * speedX + speedY * speedY + speedZ * speedZ);
  Serial.print("Resultant Speed: ");
  Serial.print(resultantSpeed);
  Serial.println(" m/s");

  delay(1000);
}
