char ch_var = 'a';
char ch_var1[10] = "an";
int integ = 45;


void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.write(integ);
  delay(50);
}
