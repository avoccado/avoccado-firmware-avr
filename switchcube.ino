
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of the project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

int16_t ax, ay, az; // accel values
int16_t gx, gy, gz; // gyro values

// uncomment "OUTPUT_READABLE_mpu" if you want to see a tab-separated
// list of the accel X/Y/Z and then gyro X/Y/Z values in decimal. Easy to read,
// not so easy to parse, and slow(er) over UART.
#define OUTPUT_READABLE_mpu

// uncomment "OUTPUT_BINARY_mpu" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
// #define OUTPUT_BINARY_mpu

#define STATUS_LED_PIN 5
bool blinkState = false;

void setup() {
  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  pinMode(3,OUTPUT);
  digitalWrite(3, HIGH); // Vcc for MPU6050
  Serial.begin(115200);
  delay(128);
  // initialize device
  Serial.println(F("Initializing devices..."));
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Serial.println(F("I2CSetup"));
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Serial.println(F("FastwireSetup"));
  Fastwire::setup(400, true);
#endif
  Serial.println(F("mpu.initialize"));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setDLPFMode(6); // 5Hz low pass
  mpu.setDHPFMode(4); // 0.625Hz high pass
  mpu.setFullScaleAccelRange(2);
  mpu.setFullScaleGyroRange(3);
  //    Wire.write(0x16);            // register DLPF_CFG - low pass filter configuration & sample rate
  //    Wire.write(0x1D);            //   10Hz Low Pass Filter Bandwidth - Internal Sample Rate 1kHz

  Serial.println(F("Reading/Updating internal sensor offsets..."));
  // -76	-2359	1688	0	0	0
  //-596	-585	664	0	0	0
  Serial.print(mpu.getXAccelOffset()); 
  Serial.print(F("\t")); // -76
  Serial.print(mpu.getYAccelOffset()); 
  Serial.print(F("\t")); // -2359
  Serial.print(mpu.getZAccelOffset()); 
  Serial.print(F("\t")); // 1688
  Serial.print(mpu.getXGyroOffset()); 
  Serial.print(F("\t")); // 0
  Serial.print(mpu.getYGyroOffset()); 
  Serial.print(F("\t")); // 0
  Serial.print(mpu.getZGyroOffset()); 
  Serial.print(F("\t")); // 0
  Serial.print(F("\n"));

  // change accel/gyro offset values
  mpu.setXGyroOffset(28);
  mpu.setYGyroOffset(6);
  mpu.setZGyroOffset(13);
  
  Serial.print(mpu.getXAccelOffset()); 
  Serial.print(F("\t")); // -76
  Serial.print(mpu.getYAccelOffset()); 
  Serial.print(F("\t")); // -2359
  Serial.print(mpu.getZAccelOffset()); 
  Serial.print(F("\t")); // 1688
  Serial.print(mpu.getXGyroOffset()); 
  Serial.print(F("\t")); // 0
  Serial.print(mpu.getYGyroOffset()); 
  Serial.print(F("\t")); // 0
  Serial.print(mpu.getZGyroOffset()); 
  Serial.print(F("\t")); // 0
  Serial.print(F("\n"));

  // configure Arduino LED for output
  pinMode(STATUS_LED_PIN, OUTPUT);
}

void loop() {
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

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(STATUS_LED_PIN, blinkState);
  delay(100);
}

