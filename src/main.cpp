#include <Arduino.h>
#include <Servo.h>

#define kp 0.1
#define kd 0.011
#define ki 1

#define servo_x 12
#define servo_y 11

#define THRESHOLD 35
#define deadband 50

#define SERVO_MIN 550
#define SERVO_MAX 2450

Servo servo1;
Servo servo2;

unsigned long reference;
unsigned long time_elapsed;
unsigned long t0;

float x_integrator = 0;
float y_integrator = 0;
float x_differentiator = 0;
float y_differentiator = 0;

int faultcheck = 1;

float x_error = 0;
float y_error = 0;

int firstEnter = 1;

// remember current servo positions
float posX = 1500;
float posY = 1500;

enum state {
  off = 0,
  initialize = 1,
  search = 2,
  sleep = 3,
  track = 4,
  hold = 5,
};

state current, next;

float error_x(int TR, int LR, int TL, int LL);
float error_y(int TR, int LR, int TL, int LL);

void setup() {
  Serial.begin(115200);
  t0 = millis();

  pinMode(10, INPUT_PULLUP);
  pinMode(servo_x, OUTPUT);
  pinMode(servo_y, OUTPUT);

  servo1.attach(servo_x);
  servo2.attach(servo_y);

  current = off;
  next = off;
}

void loop() {
  float x_errorPrev = x_error;
  float y_errorPrev = y_error;
  float x_integratorPrev = x_integrator;
  float y_integratorPrev = y_integrator;

  // Correct LDR mapping:
  // A3 = TL, A4 = TR, A5 = LL, A6 = LR
  int TL = analogRead(A6);
  int TR = analogRead(A3);
  int LL = analogRead(A5);
  int LR = analogRead(A4);

  int on = 1; //attatch to digital pin if a switch is available

  x_error = error_x(TR, LR, TL, LL); //
  y_error = error_y(TR, LR, TL, LL); //

  int threshold = (TR > THRESHOLD || LR > THRESHOLD || TL > THRESHOLD || LL > THRESHOLD);
  int aligned = (abs(x_error) <= deadband) && (abs(y_error) <= deadband);

  if (next == search && current != search) {
    reference = millis();
  }

  if (next == track && current != track) {
    firstEnter = 1;
    if (current != hold) {
      x_integrator = 0;
      y_integrator = 0;
    }
  }

  current = next;

  faultcheck = 1;

  Serial.print("TL A3=");
  Serial.print(TL);
  Serial.print(" TR A4=");
  Serial.print(TR);
  Serial.print(" LL A5=");
  Serial.print(LL);
  Serial.print(" LR A6=");
  Serial.print(LR);
  Serial.print(" x_error=");
  Serial.print(x_error);
  Serial.print(" y_error=");
  Serial.print(y_error);
  Serial.print(" current=");
  Serial.println(current);

  if (!on) {
    next = off;
  } else {
    switch (current) {
      case off: {
        next = (on) ? initialize : off;
        time_elapsed = 0;
        break;
      }

      case initialize: {
        posX = 1500;
        posY = 1500;

        servo1.writeMicroseconds((int)posX);
        servo2.writeMicroseconds((int)posY);

        x_integrator = 0;
        y_integrator = 0;
        x_differentiator = 0;
        y_differentiator = 0;

        next = search;
        break;
      }

      case search: {
        time_elapsed = millis() - reference;
        next = (threshold) ? track : (!threshold && (time_elapsed > 5000)) ? sleep : search;
        break;
      }

      case sleep: {
        time_elapsed = 0;
        next = (threshold) ? search : sleep;
        break;
      }

      case track: {
        time_elapsed = 0;
        next = (!threshold) ? search : (aligned) ? hold : track;

        if (!firstEnter) {
          x_integrator = x_integratorPrev + x_error * (millis() - t0) * 0.001;
          x_integrator = constrain(x_integrator, -2000, 2000);

          y_integrator = y_integratorPrev + y_error * (millis() - t0) * 0.001;
          y_integrator = constrain(y_integrator, -2000, 2000);

          x_differentiator = (x_error - x_errorPrev) / ((millis() - t0) * 0.001);
          y_differentiator = (y_error - y_errorPrev) / ((millis() - t0) * 0.001);
        }

        t0 = millis();
        firstEnter = 0;

        float ux = kp * x_error + kd * x_differentiator + ki * x_integrator;
        float uy = kp * y_error + kd * y_differentiator + ki * y_integrator;

        posX = constrain(1500 + ux, SERVO_MIN, SERVO_MAX);
        posY = constrain(1500 + uy, SERVO_MIN, SERVO_MAX);

        servo1.writeMicroseconds((int)posX);
        servo2.writeMicroseconds((int)posY);

        break;
      }

      case hold: {
        next = (!threshold) ? search : (!aligned) ? track : hold;
        break;
      }

      default: {
        next = off;
        break;
      }
    }
  }

  delay(20);
}

float error_x(int TR, int LR, int TL, int LL) {
  // bottom motor = left/right yaw
  // positive x_error means right side is brighter
  int left_side = TL + LL;
  int right_side = TR + LR;
  return float(right_side - left_side);
}

float error_y(int TR, int LR, int TL, int LL) {
  // top motor = up/down pitch
  // positive y_error means top side is brighter
  int top_side = TL + TR;
  int lower_side = LL + LR;
  return float(top_side - lower_side);
}
