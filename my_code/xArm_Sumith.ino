#include <LobotServoController.h>

LobotServoController myse(Serial);

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define FRAME_HEADER 0x55
#define CMD_SERVO_MOVE 0x03
#define CMD_MULT_SERVO_POS_READ 0x15

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

uint8_t send_buf[15];
uint8_t rece_buf[25];
int prev_val = 30000;
int cur_val;
int lengt;
uint8_t dummy_val;
int servo_pos[7]; // 1 to 6

//----------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  while (!Serial);
  stop_servos();
}
//----------------------------------------------------------------------
void loop() {

  check_positions();

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
void check_positions() {
  send_buf[0] = FRAME_HEADER;
  send_buf[1] = FRAME_HEADER;
  send_buf[2] = 0x09; // length
  send_buf[3] = CMD_MULT_SERVO_POS_READ; // Command
  send_buf[4] = 0x06; // No: of servos to be read
  send_buf[5] = 0x01; // Id of the servo
  send_buf[6] = 0x02; // Id of the servo
  send_buf[7] = 0x03; // Id of the servo
  send_buf[8] = 0x04; // Id of the servo
  send_buf[9] = 0x05; // Id of the servo
  send_buf[10] = 0x06; // Id of the servo

  Serial.write(send_buf, 11); //Write the serial data
  Serial.flush();
  delay(1500); // send and wait to receive the data

  if (Serial.available() ) {
    while (Serial.available() > 0) {
      prev_val = Serial.read();
      cur_val = Serial.read();

      if (cur_val == prev_val) {
        Serial.println("Starts transmission");
        lengt = Serial.read();
        dummy_val = Serial.read(); // command
        dummy_val = Serial.read(); // no: of servos

        for (int i = 0; i < 18; i++) { // 18 for 6 servos - num, lowbit, highbit
          rece_buf[i] = Serial.read(); // read 8bit angles of servos
        }
        servo_pos[1] = rece_buf[2]*256 + rece_buf[1];
        servo_pos[2] = rece_buf[5]*256 + rece_buf[4];
        servo_pos[3] = rece_buf[8]*256 + rece_buf[7];
        servo_pos[4] = rece_buf[11]*256 + rece_buf[10];
        servo_pos[5] = rece_buf[14]*256 + rece_buf[13];
        servo_pos[6] = rece_buf[17]*256 + rece_buf[16];
        
        for (int i = 1; i < 7; i++) {
          Serial.print(servo_pos[i]);
          Serial.print(",");
        }
        Serial.println();
      }
      else
      {
        Serial.flush();
      }
      //blinkled();
      prev_val = cur_val;

      Serial.flush();
      delay(1000);
    }
    Serial.flush();
    delay(1000);
  }
  else {
    Serial.println("nothing from serial comm");
  }
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
