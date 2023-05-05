#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>

Adafruit_MPU6050 imu;
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

void setup() {
    Serial.begin(115200);

    if (!imu.begin()) {
        Serial.println("No IMU found...");
        while (true);
    }

    pixels.begin();
    pixels.setPixelColor(0, 255, 0, 255);
    pixels.show();
}

void loop() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    float gx = gyro.gyro.x;

    Serial.println(gx);
}