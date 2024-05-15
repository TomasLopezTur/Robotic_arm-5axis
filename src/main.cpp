#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

#define PCA9685_ADDRESS 0x40
#define SERVO_PIN_1 2
#define SERVO_PIN_2 1
#define SERVO_MIN_ANGLE 10
#define SERVO_MAX_ANGLE 180
#define LED_PIN 2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

// Prototipos de función
int readServoAngle(int servoPin);
void moveServo(int servoPin, int startAngle, int endAngle, int speed);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);
  pinMode(LED_PIN, OUTPUT);

  int currentAngle1 = readServoAngle(SERVO_PIN_1);
  if (currentAngle1 != 10) {
    moveServo(SERVO_PIN_1, currentAngle1, 10, 100);
  }

  int currentAngle2 = readServoAngle(SERVO_PIN_2);
  if (currentAngle2 != 10) {
    moveServo(SERVO_PIN_2, currentAngle2, 10, 100);
  }
}

// Definiciones de funciones
int readServoAngle(int servoPin) {
  int pulseWidth = pwm.getPWM(servoPin);
  return map(pulseWidth, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 0, 180);
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

void loop() {
  // Establecer la posición inicial de los servos en 10° si no están ya en esa posición
  int currentAngle1 = readServoAngle(SERVO_PIN_1);
  if (currentAngle1 != 10) {
    moveServo(SERVO_PIN_1, currentAngle1, 10, 80);
  }

  int currentAngle2 = readServoAngle(SERVO_PIN_2);
  if (currentAngle2 != 10) {
    moveServo(SERVO_PIN_2, currentAngle2, 10, 80);
  }

  // Movimiento servo 1: de 10° a 60° a 80 ms
  moveServo(SERVO_PIN_1, 10, 60, 100);

  // Movimiento servo 2: de 10° a 50° a 80 ms
  moveServo(SERVO_PIN_2, 10, 50, 100);

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