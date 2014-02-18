/* void vibr(byte _state = 0) {
  if (_state == 1) {
    pinMode(PIN_VIBR, INPUT); // vibration motor
  }
  if (_state == 0) {
    pinMode(PIN_VIBR, OUTPUT); // vibration motor
    digitalWrite(PIN_VIBR, LOW); // pull low and disable motor for now
  }
  if (_state == 2){
  vibr(1);
  delay(32);
  vibr(0);
  }
}
*/ 

// below new version with n-chan enhanced LL mosfet:

void vibr(byte _state = 0) {
  if (_state == 1) {
    digitalWrite(PIN_VIBR, HIGH);
  }
  if (_state == 0) {
    digitalWrite(PIN_VIBR, LOW);
  }
  if (_state == 2){
  vibr(1);
  delay(32);
  vibr(0);
  }
  if (_state == 3)
   analogWrite(PIN_VIBR, 64);
}
