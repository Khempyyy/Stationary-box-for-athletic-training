int latchPin = D8; // ST_CP
int clockPin = D9; // SH_CP
int dataPin = D10; // DS
int bitout = D7; // DS

int num[] = { 0x01, 0x02, 0x04, 0x08 ,0x10};

void setup() {
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  pinMode(bitout, OUTPUT);

  digitalWrite(latchPin, HIGH);
  digitalWrite(bitout, HIGH);
}

void loop() {
  for (int i=0;i<5;i++) {
    DataOut(num[i]);
    delay(500);
  }
}

void DataOut(byte data) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, data);
  digitalWrite(latchPin, HIGH);
}