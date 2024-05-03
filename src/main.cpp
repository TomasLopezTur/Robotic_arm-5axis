#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h> // Incluir la librería SPI.h

// Dirección I2C del PCA9685
#define PCA9685_ADDRESS 0x40

// Definir el pin del servo
#define SERVO_PIN 0

// Definir el ángulo de inicio y fin del servo
#define SERVO_HOME_ANGLE 90
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180

// Crear una instancia del controlador PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDRESS);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50); // Frecuencia de 50 Hz para servos
}

void moveServoSmooth(int targetAngle, int servoSpeed) {
  int currentAngle = SERVO_HOME_ANGLE;
  int angleStep = 1; // Incremento/decremento de 1 grado por paso
  int delayTime = 1000 / servoSpeed; // Tiempo de retardo entre pasos (ms)

  // Asegurarse de que el ángulo objetivo esté dentro del rango válido
  if (targetAngle < SERVO_MIN_ANGLE) {
    targetAngle = SERVO_MIN_ANGLE;
  } else if (targetAngle > SERVO_MAX_ANGLE) {
    targetAngle = SERVO_MAX_ANGLE;
  }

  // Mover el servo de la posición actual al ángulo objetivo
  if (targetAngle > currentAngle) {
    // Mover el servo hacia arriba
    for (int angle = currentAngle; angle <= targetAngle; angle += angleStep) {
      pwm.setPWM(SERVO_PIN, 0, map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 10, 600));
      delay(delayTime);
    }
  } else {
    // Mover el servo hacia abajo
    for (int angle = currentAngle; angle >= targetAngle; angle -= angleStep) {
      pwm.setPWM(SERVO_PIN, 0, map(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE, 10, 600));
      delay(delayTime);
    }
  }

  // Actualizar la posición actual del servo
  currentAngle = targetAngle;
}

void loop() {
  // Mueve el servo a 45 grados a una velocidad de 30 grados/segundo
  moveServoSmooth(45, 30);
  delay(2000); // Espera 2 segundos

  // Mueve el servo a 135 grados a una velocidad de 45 grados/segundo
  moveServoSmooth(135, 45);
  delay(2000); // Espera 2 segundos
}
