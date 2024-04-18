# Date Code Colection

- 13
- 12
- 14
- 27
- 26
- 25
- 33

``` cpp
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
```

``` cpp
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
```
