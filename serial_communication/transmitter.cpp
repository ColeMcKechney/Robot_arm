#include <Stream.h>

#define GET_LOW_BYTE(A) (uint8_t)((A))
#define GET_HIGH_BYTE(A) (uint8_t)((A) >> 8)
#define FRAME_HEADER            0x55
#define CMD_SERVO_MOVE          0x03

uint8_t buf[11];
int Time = 100;
int Position = 750;
int servoID = 2;

void setup() {
  Serial.begin(9600);
  pinMode(3,OUTPUT);

  buf[0] = FRAME_HEADER;             
  buf[1] = FRAME_HEADER;
  buf[2] = 8;                             
  buf[3] = CMD_SERVO_MOVE;              
  buf[4] = 1;                         
  buf[5] = GET_LOW_BYTE(Time);        
  buf[6] = GET_HIGH_BYTE(Time);        
  buf[7] = servoID;             
  buf[8] = GET_LOW_BYTE(Position);        
  buf[9] = GET_HIGH_BYTE(Position); 
}

void loop() {
  Serial.write(buf,10); //Write the serial data
  
  //Serial.println(buf[0],DEC);
  Serial.flush();
  //blinkled();
}

void blinkled(){
  digitalWrite(3,HIGH);
  delay(500);
  digitalWrite(3,LOW);
  delay(500);
}
