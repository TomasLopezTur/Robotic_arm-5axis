#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>

// Definición de pines para los motores
const int microServoPin = 13;
const int servo2Pin = 12;
const int servo3Pin = 14;
const int stepper1Pin1 = 27;
const int stepper1Pin2 = 26;
const int stepper2Pin1 = 25;
const int stepper2Pin2 = 33;

//Servos y steppers
Servo microServo1;
Stepper stepper1(200, stepper1Pin1, stepper1Pin2);
Stepper stepper2(200, stepper2Pin1, stepper2Pin2);
Servo servo2;
Servo servo3;

void setup() {
  // put your setup code here, to run once:
  
  microServo1.attach(6);
  servo2.attach(7);
  servo3.attach(8);

  microServo1.write(90);
  servo2.write(90);
  servo3.write(90);

  // Mueve el stepper motor a 90 grados (en sentido horario)
  int stepsToMove1 = 90 * 200 / 360;  // Calcula la cantidad de pasos para 90 grados
  stepper1.step(stepsToMove1); 
  int stepsToMove2 = 90 * 200 / 360;  // Calcula la cantidad de pasos para 90 grados
  stepper2.step(stepsToMove2);
}

void loop() {
  // put your main code here, to run repeatedly:
}

