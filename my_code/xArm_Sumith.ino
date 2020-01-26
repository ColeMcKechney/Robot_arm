#include <LobotServoController.h>

LobotServoController myse(Serial);

const int GRIP = 1;
const int NECK = 6;
const int THIRD = 4; // 4
const int FORTH = 3; // 3
const int FIFTH = 5; // 5
const int BOTTOM = 2; // 2

const int GRIP_CLOSE = 750; //750;
const int GRIP_OPEN = 0;

const int NECK_LEFT = 0;
const int NECK_RITE = 1500;

const int THIRD_LEFT = 0; //tested fail
const int THIRD_RITE = 150; //tested fail

const int FORTH_LEFT = 0; //tested success
const int FORTH_RITE = 750; //tested success

const int FIFTH_LEFT = 200; //tested success
const int FIFTH_RITE = 800; //tested success

const int BOTTOM_LEFT = 0;
const int BOTTOM_RITE = 1500;

int MAX_ITER = 5;

int iteration = 0;
int exec_dely = 2000;
int norm_dely = exec_dely + 300;

void move_forth();

//----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);
  stop_servos();
}
//----------------------------------------------------------------------
void loop() {
  if (iteration < MAX_ITER)  {
    //move_grip();
    //rotate_neck();
    //move_third();
    //move_forth();
    //move_fifth();
    rotate_base();
  }
  else  {
    stop_servos();
  }
  iteration = iteration + 1;
  delay(500);
  Serial.flush();
}
//----------------------------------------------------------------------
void move_grip() {
  myse.moveServo(GRIP, GRIP_CLOSE, exec_dely);    delay(norm_dely);
  myse.moveServo(GRIP, GRIP_OPEN, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
void rotate_neck() {
  myse.moveServo(NECK, NECK_LEFT, exec_dely);    delay(norm_dely);
  myse.moveServo(NECK, NECK_RITE, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
void move_third() {
  myse.moveServo(THIRD, THIRD_LEFT, exec_dely);    delay(norm_dely);
  myse.moveServo(THIRD, THIRD_RITE, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
void move_forth() {
  myse.moveServo(FORTH, FORTH_LEFT, exec_dely);    delay(norm_dely);
  myse.moveServo(FORTH, FORTH_RITE, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
void move_fifth() {
  myse.moveServo(FIFTH, FIFTH_LEFT, exec_dely);    delay(norm_dely);
  myse.moveServo(FIFTH, FIFTH_RITE, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
void rotate_base() {
  myse.moveServo(BOTTOM, BOTTOM_LEFT, exec_dely);    delay(norm_dely);
  myse.moveServo(BOTTOM, BOTTOM_RITE, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
void stop_servos() {
  myse.moveServo(GRIP, (GRIP_CLOSE + GRIP_OPEN) / 2, exec_dely);    delay(norm_dely);
  myse.moveServo(NECK, (NECK_LEFT + NECK_RITE) / 2, exec_dely);    delay(norm_dely);
  myse.moveServo(THIRD, (THIRD_LEFT + THIRD_RITE ) / 2, exec_dely);    delay(norm_dely);
  myse.moveServo(FORTH, (FORTH_LEFT + FORTH_RITE) / 2, exec_dely);    delay(norm_dely);
  myse.moveServo(FIFTH, (FIFTH_LEFT + FIFTH_RITE) / 2, exec_dely);    delay(norm_dely);
  myse.moveServo(BOTTOM, (BOTTOM_LEFT + BOTTOM_RITE) / 2, exec_dely);    delay(norm_dely);
}
//----------------------------------------------------------------------
