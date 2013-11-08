

void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}


// The sensor should be motionless on a horizontal surface while calibration is happening
void calibrate_sensors() {
  int                   num_readings = 10; // 10 times 100ms = 1sec
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  
  Serial.println(F("Starting Calibration"));

  // Discard the first set of values read from the IMU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // delay(500);
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    x_accel += ax;
    y_accel += ay;
    z_accel += az;
    x_gyro += gx;
    y_gyro += gy;
    z_gyro += gz;
    delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  Serial.println(F("Finishing Calibration"));
}

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

  float angle_x;
  float angle_y;
  float angle_z;

void getangle()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Read the raw values.
  double dT;
  unsigned long t_now = millis();   // Get the time of reading for rotation computations

  // Convert gyro values to degrees/sec
  //  float FS_SEL = 131; 250deg/s
  float FS_SEL = 16; // 2000 deg/s
  float gyro_x = (gx - base_x_gyro)/FS_SEL;
  float gyro_y = (gy - base_y_gyro)/FS_SEL;
  float gyro_z = (gz - base_z_gyro)/FS_SEL;
  
  float accel_x = ax;
  float accel_y = ay;
  float accel_z = az;
  
  // Get angle values from accelerometer
  float RADIANS_TO_DEGREES = 180/3.14159;
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  
  // Compute the (filtered) gyro angles
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
  // Compute the drifting gyro angles
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
  // Apply the complementary filter to figure out the change in angle - choice of alpha is
  // estimated now.  Alpha depends on the sampling rate...
float alpha = 0.87;
angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle
  
  // Update the saved data with the latest values
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  if (DEBUG) {
  // Send the data to the serial port
  /* Serial.print(F("DEL:"));              //Delta T
  Serial.print(dt, DEC);
  Serial.print(F("#ACC:"));              //Accelerometer angle
  Serial.print(accel_angle_x, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_y, 2);
  Serial.print(F(","));
  Serial.print(accel_angle_z, 2);
  Serial.print(F("#GYR:"));
  Serial.print(unfiltered_gyro_angle_x, 2);        //Gyroscope angle
  Serial.print(F(","));
  Serial.print(unfiltered_gyro_angle_y, 2);
  Serial.print(F(","));
  Serial.print(unfiltered_gyro_angle_z, 2);

  Serial.print(F("\t#FIL:\t"));             //Filtered angle
  Serial.print(angle_x, 2);
  Serial.print(F("\t"));
  Serial.print(angle_y, 2);
  Serial.print(F("\t"));
  Serial.print(angle_z, 2);
  Serial.println(F(""));
  */
 } 
}
