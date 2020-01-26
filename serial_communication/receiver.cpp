#include <Stream.h>

int rec_buf[20]; //Initialized variable to store recieved data
int prev_val = 30000;
int cur_val;
int lengt;

void setup() {
  // Begin the Serial at 9600 Baud
  Serial.begin(9600);
  pinMode(3, OUTPUT);
}

void loop() {
  if (Serial.available() ) {
    while (Serial.available() > 0) {
      prev_val = Serial.read();
      cur_val = Serial.read();
      // Serial.println(cur_val, DEC); //Print data on Serial Monitor

      if (cur_val == prev_val) {
        Serial.println("Starts transmission");
        lengt = Serial.read();

        for (int i = 0; i < lengt - 1; i++) {
          rec_buf[i] = Serial.read();
        }
        Serial.print("cur_val = ");
        Serial.print(cur_val, DEC);
        Serial.print(", prev_val = ");
        Serial.print(prev_val, DEC);
        Serial.print("\n");
        for (int i = 0; i < lengt - 1; i++) {
          Serial.print(rec_buf[i]);
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
    }
    Serial.flush();
    delay(1000);
  }
  Serial.flush();
  delay(1000);
}

void blinkled() {
  digitalWrite(3, HIGH);
  delay(500);
  digitalWrite(3, LOW);
  delay(500);
}
