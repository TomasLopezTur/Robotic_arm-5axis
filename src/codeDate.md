# Código 1

```cpp
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
void setup() {
  servo1.attach(6);
  servo2.attach(7);
  servo3.attach(8);
  servo4.attach(9);
  servo5.attach(10);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
}
void moveServo(Servo &servo, int startPos, int endPos) {
  int step = startPos < endPos ? 1 : -1;
  for (int pos = startPos; pos != endPos; pos += step) {
    servo.write(pos);
    delay(15);
  }
}
void loop() {
  moveServo(servo1, 70, 120);
  moveServo(servo1, 120, 70);
  moveServo(servo2, 70, 120);
  moveServo(servo2, 120, 70);
  moveServo(servo3, 70, 120);
  moveServo(servo3, 120, 70);
  moveServo(servo4, 70, 120);
  moveServo(servo4, 120, 70);
  moveServo(servo5, 70, 120);
  moveServo(servo5, 120, 70);
}
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;
int pos5 = 0;

void setup() {
  servo1.attach(6);
  servo2.attach(7);
  servo3.attach(8);
  servo4.attach(9);
  servo5.attach(10);

  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
}
void loop() {

  for (pos1 = 70; pos1 <= 120; pos1 += 1) {
    servo1.write(pos1);
    delay(15);
  }
  for (pos1 = 120; pos1 >= 70; pos1 -= 1) {
    servo1.write(pos1);
    delay(15);
  }
  for (pos2 = 70; pos2 <= 120; pos2 += 1) {
    servo2.write(pos2);
    delay(15);
  }
  for (pos2 = 120; pos2 >= 70; pos2 -= 1) {
    servo2.write(pos2);
    delay(15);
  }
    for (pos3 = 70; pos3 <= 120; pos3 += 1) {
    servo1.write(pos3);
    delay(15);
  }
  for (pos3 = 120; pos3 >= 70; pos3 -= 1) {
    servo1.write(pos3);
    delay(15);
  }
    for (pos4 = 70; pos4 <= 120; pos4 += 1) {
    servo1.write(pos4);
    delay(15);
  }
  for (pos4 = 120; pos4 >= 70; pos4 -= 1) {
    servo1.write(pos4);
    delay(15);
  }
    for (pos5 = 70; pos5 <= 120; pos5 += 1) {
    servo1.write(pos5);
    delay(15);
  }
  for (pos5 = 120; pos5 >= 70; pos5 -= 1) {
    servo1.write(pos5);
    delay(15);
  }
} 

# Código 3

```cpp
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h> // Incluir la librería SPI.h

#define PCA9685_ADDRESS 0x40
#define SERVO_PIN_1 0
#define SERVO_PIN_2 1
#define SERVO_MIN_ANGLE 10
#define SERVO_MAX_ANGLE 180
#define LED_PIN 2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia de 50 Hz para servos
  pinMode(LED_PIN, OUTPUT);
}

void moveServo(int servoPin, int startAngle, int endAngle, int speed) {
  startAngle = constrain(startAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  endAngle = constrain(endAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  int step = (startAngle < endAngle) ? 1 : -1;

  for (int pos = startAngle; pos != endAngle; pos += step) {
    pwm.setPWM(servoPin, 0, map(pos, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 100, 600));
    delay(speed);
  }
  pwm.setPWM(servoPin, 0, map(endAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 100, 600));
}

int readServoAngle(int servoPin) {
  int pulseWidth = pwm.getPWM(servoPin);
  return map(pulseWidth, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 0, 180);
}

void loop() {
  int currentAngle1 = readServoAngle(SERVO_PIN_1);
  int currentAngle2 = readServoAngle(SERVO_PIN_2);

  // Movimiento servo 1: de 10° a 60° a 80 ms
  moveServo(SERVO_PIN_1, currentAngle1, 60, 100);

  // Movimiento servo 2: de 10° a 50° a 80 ms
  moveServo(SERVO_PIN_2, currentAngle2, 50, 100);

  // Retorno a 10°
  moveServo(SERVO_PIN_1, 60, 10, 80);
  moveServo(SERVO_PIN_2, 50, 10, 80);

  // Encender LED
  digitalWrite(LED_PIN, HIGH);
  delay(2000); // Encendido por 2 segundos

  // Apagar LED y esperar antes de reiniciar el loop
  digitalWrite(LED_PIN, LOW);
  delay(2000); // Apagado por 2 segundos
}
