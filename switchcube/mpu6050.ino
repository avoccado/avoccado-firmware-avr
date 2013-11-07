void mpucheck() {
  // read raw accel/gyro measurements from device
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // these methods (and a few others) are also available
  //mpu.getAcceleration(&ax, &ay, &az);
  //mpu.getRotation(&gx, &gy, &gz);

#ifdef OUTPUT_READABLE_mpu
  // display tab-separated accel/gyro x/y/z values
  Serial.print(F("a/g:\t"));
  Serial.print(ax); 
  Serial.print(F("\t"));
  Serial.print(ay); 
  Serial.print(F("\t"));
  Serial.print(az); 
  Serial.print(F("\t"));
  Serial.print(gx); 
  Serial.print(F("\t"));
  Serial.print(gy); 
  Serial.print(F("\t"));
  Serial.print(gz); 
  Serial.print(F("\t"));
  double dT = ( (double) mpu.getTemperature() + 12412.0) / 340.0;
  Serial.print(dT, 3);
  Serial.print(F("C \t"));
  Serial.println();
#endif

#ifdef OUTPUT_BINARY_mpu
  Serial.write((uint8_t)(ax >> 8)); 
  Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); 
  Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); 
  Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); 
  Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); 
  Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); 
  Serial.write((uint8_t)(gz & 0xFF));
#endif
}
