#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_1000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(a.acceleration.x*1.05);
  Serial.print(',');
  Serial.print(a.acceleration.y*0.98);
  Serial.print(',');
  Serial.print(a.acceleration.z*1.283);
  Serial.print(',');
  Serial.print(g.gyro.x-0.76);
  Serial.print(',');
  Serial.print(g.gyro.y+0.07);
  Serial.print(',');
  Serial.print(g.gyro.z);
  Serial.print(';');

  delay(10);
  }