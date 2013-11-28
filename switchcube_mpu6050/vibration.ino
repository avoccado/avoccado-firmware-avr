void vibr(byte _state = 0) {
  if (_state == 1) {
    pinMode(9, INPUT); // vibration motor
  }
  if (_state == 0) {
    pinMode(9, OUTPUT); // vibration motor
    digitalWrite(9, LOW); // pull low and disable motor for now
  }
  if (_state == 2){
  vibr(1);
  delay(32);
  vibr(0);
  }
}
