#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Math.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include <Servo.h>

Servo servo1;
Servo servo2;

struct Angles {
  double x;
  double y;

  Angles() {};

  Angles(double _x, double _y) {
    x = _x;
    y = _y;
  }

  void print() {
    Serial.print("Angle X: ");
    Serial.print(x);
    Serial.print(", Angle Y: ");
    Serial.println(y);
  }
};


#define LENGTH 2

Angles average[LENGTH];
int averageLength = 0;

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

void setup(void) {
  Serial.begin(9600);
  //  Wire.begin();'

  servo1.attach(7);
  servo2.attach(8);

  if (!accel.begin()) {
    Serial.println("No ADXL345 sensor detected.");
    while (1)
      ;
  }
}

void loop(void) {
  sensors_event_t event;
  accel.getEvent(&event);
  Angles accAngles = getAnglesFromAcc(event.acceleration.x, event.acceleration.y, event.acceleration.z);

  fillAverage(average, averageLength, accAngles);
  if (averageLength < LENGTH) {
    averageLength ++;
  }

  Angles averageAngle = getAverage(average, averageLength);

  servo1.write(map(averageAngle.x, -90, 90, 0, 180));
  servo2.write(map(averageAngle.y, -90, 90, 0, 180));

  // delay(300);
}

Angles getAnglesFromAcc(double accX, double accY, double accZ) {

  double angleX = atan(accX / accZ) * 180 / PI;
  double angleY = atan(accY / accZ) * 180 / PI;

  return Angles(angleX, angleY);
}

Angles getAverage(Angles average[], int length) {
  float x = 0;
  float y = 0;
  
  for (int i = 0; i < length; i++) {
    x += average[i].x;
    y += average[i].y;
    // Serial.print("[" + String(i) + "]");
    // average[i]->print();
  }

  Angles angle(x / length, y / length);
  // Serial.print("[A]");
  // angle.print();
  return angle;
}

void shiftAverage(Angles average[], int length) {
  // Move second element to pos 1, move third element to pos 2, move fourth element to pos 3, etc.
  for (int i = 1; i < length; i++) {
    average[i - 1].x = average[i].x;
    average[i - 1].y = average[i].y;
    Serial.print("[" + String(i) + "]");
    average[i].print();
  }
}

void fillAverage(Angles *average, int length,  Angles angle) {
  
  if (length < LENGTH) {
    average[length].x = angle.x;
    average[length].y = angle.y;
    Serial.print("Next angle: ");
    angle.print();
    Serial.print("Average: ");
    getAverage(average, length + 1).print();
    return;
  }


  shiftAverage(average, LENGTH);

  average[LENGTH - 1] = angle;

  Serial.print("Average: ");
  getAverage(average, LENGTH).print();
}
