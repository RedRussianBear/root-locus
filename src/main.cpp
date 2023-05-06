#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <ServoInput.h>
#include <Servo.h>

#define CH_ZERO1 1485
#define CH_ZERO2 1485
#define US_TO_RADPS 0.02
#define N_SAMPLES 400
#define FAILSAFE_TIMEOUT_MS 1000

Adafruit_MPU6050 imu;
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

ServoInputPin<A3> ch1;
ServoInputPin<A6> ch2;

int ch1v = CH_ZERO1;
int ch2v = CH_ZERO2;
double g_zero = 0;
double heading = 0;
unsigned long last_time_us;
Servo motor_left, motor_right;

unsigned long last_pulse_ms;

void setup() {
//    Serial.begin(115200);

    if (!imu.begin()) {
        Serial.println("No IMU found...");
        while (true);
    }

    // Set IMU operating characteristics
    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_250_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_10_HZ);

    motor_left.attach(A0);
    motor_right.attach(A1);

    pixels.begin();
    pixels.setPixelColor(0, 255, 255, 0);
    pixels.show();

    motor_left.writeMicroseconds(ch1v);
    motor_right.writeMicroseconds(ch2v);

    sensors_event_t accel, gyro, temp;
    for (int i = 0; i < N_SAMPLES; i++) {
        imu.getEvent(&accel, &gyro, &temp);
        g_zero += gyro.gyro.x;
        delay(10);
    }
    g_zero /= N_SAMPLES;

    pixels.setPixelColor(0, 255, 0, 255);
    pixels.show();
    last_time_us = micros();
    last_pulse_ms = millis();
}


void loop() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    double gx = gyro.gyro.x - g_zero;
    double dt = (micros() - last_time_us) / 1e6;
    last_time_us = micros();
    heading += gx * dt;
    double K = 60;

    pixels.setPixelColor(0, 127 + heading * 50, 0, 127 - heading * 50);
    pixels.show();

    if (ch1.available() || ch2.available())
        last_pulse_ms = millis();

    ch1v = ch1.getPulse();
    ch2v = ch2.getPulse();
    int rotation_setpoint = ch1v + ch2v - CH_ZERO1 - CH_ZERO2;

    if (rotation_setpoint > 40)
        heading = 0;

    int correction = (int) (-K * heading);

    if (millis() - last_pulse_ms > FAILSAFE_TIMEOUT_MS){
        motor_left.writeMicroseconds(0);
        motor_right.writeMicroseconds(0);
    } else {
        motor_left.writeMicroseconds(ch1v - correction);
        motor_right.writeMicroseconds(ch2v - correction);
    }

    delay(1);
//        Serial.print(gx);
//        Serial.print('\t');
//        Serial.print(heading);
//        Serial.print('\t');
//        Serial.println(correction);
    //    Serial.print(ch1v);
    //    Serial.print('\t');
    //    Serial.print(ch2v);
    //    Serial.print('\t');
    //    Serial.print(ch1v + ch2v - CH_ZERO1 - CH_ZERO2);
    //    Serial.print('\t');
    //    Serial.print(rotation_setpoint);
    //    Serial.print('\t');
    //    Serial.println(corr);
    // Gyro positive = turning right
    // motor left minus normal
    // motor right plus normal
    // straight: ch1v + ch2v = 0
    // right ch1v + ch2v < 0
    // left ch1v + ch2v > 0
}