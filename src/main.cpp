#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>
#include <ServoInput.h>
#include <Servo.h>

#define CH_ZERO1 1490
#define CH_ZERO2 1480
#define US_TO_RADPS 0.02

Adafruit_MPU6050 imu;
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

ServoInputPin<A3> ch1;
ServoInputPin<A6> ch2;

int ch1v = CH_ZERO1;
int ch2v = CH_ZERO2;
float g_zero = 0;
Servo motor_left, motor_right;

void setup() {
//    Serial.begin(115200);

    if (!imu.begin()) {
        Serial.println("No IMU found...");
        while (true);
    }

    // Set IMU operating characteristics
    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_5_HZ);

    motor_left.attach(A0);
    motor_right.attach(A1);

    pixels.begin();
    pixels.setPixelColor(0, 255, 255, 0);
    pixels.show();

    motor_left.writeMicroseconds(ch1v);
    motor_right.writeMicroseconds(ch2v);
    delay(3000);
    sensors_event_t accel, gyro, temp;

    for (int i = 0; i < 300; i++) {
        imu.getEvent(&accel, &gyro, &temp);
        g_zero += gyro.gyro.x;
        delay(10);
    }
    g_zero /= 300;

    pixels.setPixelColor(0, 255, 0, 255);
    pixels.show();
}

double corr = 0;
void loop() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    double gx = gyro.gyro.x - g_zero;
    double K = 0.2;


    pixels.setPixelColor(0, 120 + gx * 50, 0, 120 - gx * 50);
    pixels.show();

    ch1v = ch1.getPulse();
    ch2v = ch2.getPulse();
    double rotation_setpoint = (ch1v + ch2v - CH_ZERO1 - CH_ZERO2) * US_TO_RADPS;

    if (abs(ch1v + ch2v - CH_ZERO1 - CH_ZERO2) < 40 && ch2v > CH_ZERO2 + 56) {
        corr += (K * gx / US_TO_RADPS);
    } else {
        corr = 0;
    }
//    Serial.print(gx);
//    Serial.print('\t');
//    Serial.print(ch1v);
//    Serial.print('\t');
//    Serial.print(ch2v);
//    Serial.print('\t');
//    Serial.print(ch1v + ch2v - CH_ZERO1 - CH_ZERO2);
//    Serial.print('\t');
//    Serial.print(rotation_setpoint);
//    Serial.print('\t');
//    Serial.println(corr);


    motor_left.writeMicroseconds(ch1v + corr);
    motor_right.writeMicroseconds(ch2v + corr);

    delay(1);
    // Gyro positive = turning right
    // motor left minus normal
    // motor right plus normal
    // straight: ch1v + ch2v = 0
    // right ch1v + ch2v < 0
    // left ch1v + ch2v > 0
}