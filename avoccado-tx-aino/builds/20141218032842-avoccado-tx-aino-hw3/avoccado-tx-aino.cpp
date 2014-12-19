#line 1 "avoccado-tx-aino.ino"
#if 1                                   
#define HW 3                                                                                                                                       
#include "Arduino.h"
#include "Arduino.h"
void setup();
bool checkTouch(unsigned int _sensitivity);
void loop();
void handle_K();
void handle_T();
void handle_B();
void pulse(unsigned int _length);
#line 4
inline unsigned long get_last_time();

void active();
void ledst(int sta);
void colorWipe(uint32_t c, uint8_t wait);
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro);
void calibrate_sensors();
void getangle();
void send_K(unsigned int to);
boolean send_L(uint16_t to, byte* ledmap);
void processPacket();
#line 2
__asm volatile ("nop");                                   
#endif

                                     
                                               

                                                                               
                                                              
#include "I2Cdev.h"
#include "MPU6050.h"
#include <avr/sleep.h>
#include <avr/power.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE                                                                                                     
#include "Wire.h"
#endif
#include <RF24.h>
#include <SPI.h>
#include <stdarg.h>
#include <EEPROM.h>
#ifdef USE_TOUCH
#include <CapacitiveSensor.h>
#endif
#define DEBUG 1                                                            
#define USE_EEPROM                                                                                      
#define LEDPIN 4
#define KEEPALIVE 1                                                         
                                     
                                               
#define TIMEOUT_HIBERNATE 512
#define TIMEOUT_MEMS 5000
#define TIMEOUT_RADIO 8000
#define PIN_VIBR 3                                                                

#ifdef USE_LEDS
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel leds = Adafruit_NeoPixel(8, LEDPIN, NEO_GRB + NEO_KHZ800);                                                           
#endif
RF24 radio(9, 10);                                       

#ifdef USE_TOUCH
CapacitiveSensor cs = CapacitiveSensor(8, 7);                                            
#endif

const unsigned long interval_filter = 32;
const unsigned long interval = 24;                              
unsigned long last_time_filtered;
byte sweep = 0;
byte nodeID = 1;                                                                                                           
uint16_t this_node = 00;                                                                            
                                                               
unsigned long iterations = 0;
unsigned long errors = 0;
unsigned int loss = 0;
unsigned long p_sent = 0;
unsigned long p_recv = 0;

const short max_active_nodes = 32;                                           
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;
unsigned long last_time_sent;
unsigned long updates = 0;                                                                  
boolean send_T(uint16_t to);
void send_L1(int to, int _b);
void handle_K(char header);
void handle_L(char header);
void handle_T(char header);
void handle_B(char header);
void ledupdate(byte* ledmap);
void p(char *fmt, ... );
void ledst(int sta = 127);
void mpucheck();
bool strobe = 1;

                                    
                                                           
                           
                  
MPU6050 mpu;
                                                                   

int16_t ax, ay, az;                
int16_t gx, gy, gz;               

#define OUTPUT_READABLE_mpu                                                                                
                                                                

unsigned long last_read_time;
float         last_x_angle;                                  
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;                                           
float         last_gyro_y_angle;
float         last_gyro_z_angle;

inline unsigned long get_last_time() {
  return last_read_time;
}
inline float get_last_x_angle() {
  return last_x_angle;
}
inline float get_last_y_angle() {
  return last_y_angle;
}
inline float get_last_z_angle() {
  return last_z_angle;
}
inline float get_last_gyro_x_angle() {
  return last_gyro_x_angle;
}
inline float get_last_gyro_y_angle() {
  return last_gyro_y_angle;
}
inline float get_last_gyro_z_angle() {
  return last_gyro_z_angle;
}

                                                           
                                        
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

                     
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;
void setup_watchdog(uint8_t prescalar);
void do_sleep(void);
const short sleep_cycles_per_transmission = 10;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;
void powerMode(byte _mode);

void vibr(byte _state);
void pulse(unsigned int _length = 50);
bool checkTouch(unsigned int _sensitivity = 96);
byte mode = 0;
unsigned long lastActive = 0;

                                             
typedef enum { sender = 1, receiver } role_e;

                                          
const char* role_friendly_name[] = { "invalid", "sender", "receiver"};

                                         
role_e role;

const int role_pin = 1;

                                                       
const uint64_t pipes[2] = { 0xAEAEAEAEA0LL, 0xAEAEAEAEA1LL };


void setup() {
#if HW == 2                               
  pinMode(A1, OUTPUT);                
  digitalWrite(A1, LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);                   
  pinMode(8, OUTPUT);                            
  digitalWrite(8, HIGH);                                                                                                                          
#endif
  pinMode(PIN_VIBR, OUTPUT);                   
  vibr(0);                                      
  vibr(1);                         
#ifdef USE_LEDS
  leds.begin();              
  leds.show();                                  
  ledst(5);                                                   
#endif
#ifdef USE_TOUCH
  cs.set_CS_AutocaL_Millis(10000);
#endif
  Serial.begin(115200);                                   
  delay(64);
  vibr(0);                  
                       
  Serial.println(F("avoccado aino 0.201411262359"));
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Serial.println(F("I2C setup"));
  Wire.begin();                                                               
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Serial.println(F("FastwireSetup"));
  Fastwire::setup(400, true);
#endif
  Serial.println(F("mpu.initialize"));
  mpu.initialize();
  ledst(4);
#ifdef USE_EEPROM
  nodeID = EEPROM.read(0);
  Serial.print(F("EEPROM, "));
#endif
  Serial.println(nodeID);
  Serial.print(F("Network ID (oct): "));
#ifdef USE_EEPROM
  this_node = ((int) EEPROM.read(15) * 256) + ((int) (EEPROM.read(16)));
  Serial.print(F("EEPROM, "));
#endif
  Serial.print(this_node, OCT);
  Serial.print(F(", (dec): "));
  Serial.print(this_node, DEC);
  Serial.println();
  SPI.begin();                     
  radio.begin();                     
                                                                                                                      
  radio.setPALevel(RF24_PA_MAX);                                      
  radio.enableDynamicPayloads();                             
  radio.setRetries(4, 8);                                                                  
  radio.setChannel(42);
  Serial.print(F("\t datarate 250K: "));
  Serial.println(radio.getDataRate());
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  Serial.print(F("PA-level: "));
  Serial.print(radio.getPALevel());
  Serial.print(F("\t CRC: "));
  Serial.print(radio.getCRCLength());
  Serial.print(F("\t datarate 250K: "));
  Serial.println(radio.getDataRate());
  Serial.println();

  if ( role_pin == 1 )
    role = sender;
  else
    role = receiver;

  p("\t role: %s\n\r", role_friendly_name[role]);
  if ( role == sender )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
  }

  radio.startListening();
  radio.printDetails();
  Serial.println();
  Serial.print(F("UID: "));
  Serial.print(F("(N/A)\n"));
  p("%010ld: Starting up\n", millis());
#ifdef USE_LEDS
  colorWipe(leds.Color(50, 0, 0), 50);       
  colorWipe(leds.Color(0, 50, 0), 50);         
  colorWipe(leds.Color(0, 0, 50), 50);        
  colorWipe(leds.Color(0, 0, 0), 0);         
  ledst(5);
#endif

                      
  Serial.println(F("init I2C..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connected." : "MPU6050 error.");
  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
#ifdef USE_LEDS
  ledst(1);
#endif
  setup_watchdog(wdt_1s);                                   
  radio.powerUp();
}

void powerMode(byte _mode = mode) {
  switch (_mode)                                               
  {
    case 0:
      break;                           
    case 1:
      mpu.setSleepEnabled(0);                            
      Serial.println(F("powerMode(1), MEMS up"));
      mode = 0;
      break;
    case 2:
      radio.powerUp();                      
      Serial.println(F("powerMode(2), radio up"));
      mode = 0;
      break;
    case 5:
      Serial.println(F("powerMode(5), hibernate"));
      radio.powerDown();
      mpu.setSleepEnabled(1);
      mpu.setStandbyXGyroEnabled(1);
      mpu.setStandbyYGyroEnabled(1);
      mpu.setStandbyZGyroEnabled(1);
      Serial.print(F("Sleeping."));
      vibr(1);
      delay(64);
      vibr(0);
      delay(196);
      vibr(1);
      delay(64);
      vibr(0);
      setup_watchdog(wdt_250ms);                               
      while ( ((ay < (-10000)) || ay > (10000)) ) {                                                      
        do_sleep();                    
        Serial.print(F("."));
        mpu.setSleepEnabled(0);                                                        
                             
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                                                               
                                 
      }
      for (int _i = 0; _i < 2; _i++) {                                                        
        vibr(1);
        delay(128);
        vibr(0);
        delay(56);
      }
      Serial.println();
      powerMode(1);                 
      mpu.setStandbyXGyroEnabled(0);
      mpu.setStandbyYGyroEnabled(0);
      mpu.setStandbyZGyroEnabled(0);
      powerMode(2);                  
      active();
      break;
    case 6:                                  
      Serial.println(F("powerMode(6), shutdown"));
      radio.powerDown();
      mpu.setSleepEnabled(1);
      vibr(4);                                
      delay(512);
      vibr(4);
      delay(700);
      vibr(4);
      delay(900);
      vibr(4);
      delay(1100);
      vibr(5);
      delay(1280);
      vibr(5);
      delay(1650);
      pulse(32);
      setup_watchdog(wdt_8s);                             
      while (1) do_sleep();                 
      break;
    default:                           
      return;
  };
}

bool checkTouch(unsigned int _sensitivity)
{
#ifdef USE_TOUCH
  long csVal = cs.capacitiveSensor(24);
  if (csVal > _sensitivity) {
                       
                            
    return 1;
  }
#endif
  return 0;
}

void active() {
  lastActive = millis();
}

void loop() {

                           
                                                                                   
                           

  if (millis() - lastActive > TIMEOUT_HIBERNATE && ((ay < (-10000)) || ay > (10000))) powerMode(5);
                                                                    
                                                                
  updates++;

                                                                          
   
                    
   
    
  unsigned long now = millis();
  unsigned long nowM = micros();
  if ( now - last_time_filtered >= interval_filter ) {                                                                  
    getangle();
    last_time_filtered = now;
  }

  if ( now - last_time_sent >= interval )                                                                  
  {
               
                   
                                                                 
        
    updates = 0;
    last_time_sent = now;
    radio.powerUp();
    send_K(1);                                           
    radio.powerDown();
  }

}


                                                
                                    
void setup_watchdog(uint8_t prescalar)
{
  prescalar = min(9, prescalar);
  uint8_t wdtcsr = prescalar & 7;
  if ( prescalar & 8 )
    wdtcsr |= _BV(WDP3);
  MCUSR &= ~_BV(WDRF);
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDCE) | wdtcsr | _BV(WDIE);
}

ISR(WDT_vect)
{
  --sleep_cycles_remaining;
}

void do_sleep(void)
{
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                          
  sleep_enable();
  sleep_mode();                                             
  sleep_disable();                                                                               
}


                                                 
void p(char *fmt, ... ) {
  char tmp[128];                                         
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}
#line 1 "led.ino"
void ledst(int sta){
  #ifdef USE_LEDS
  uint32_t c=0;
  switch (sta) {
  case 0:
    c=leds.Color(0, 0, 0);
    break;
  case 1:
    c=leds.Color(50, 0, 0);
    break;
  case 2:
    c=leds.Color(0, 50, 0);
    break;
  case 3:
    c=leds.Color(0, 0, 50);
    break;
  case 4:
    c=leds.Color(25, 0, 25);
    break;
  case 5:
    c=leds.Color(25, 25, 0);
    break;
  case 6:
    c=leds.Color(255, 0, 0);
    break;
  case 10:
    c=leds.Color(255, 255, 255);
    break;      
  case 11:
    c=leds.Color(255, 0, 0);
    break;
  case 12:
    c=leds.Color(0, 255, 0);
    break;
  case 13:
    c=leds.Color(0, 0, 255);
    break;
  default:
    c=leds.Color(20, 2, 2);
    break;
  }
  leds.setPixelColor(0, c);
  leds.show();
  #endif
}

PROGMEM const char red[]=
{
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0};
PROGMEM const char blue[]=
{
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255};

PROGMEM const byte pat1[]={
  10,0,0,
  10,10,0,
  0,10,0,
  0,10,10,
  0,0,10,
  10,0,10,
  10,5,0,
  5,16,5};

void colorWipe(uint32_t c, uint8_t wait) {                                               
  #ifdef USE_LED
  for(uint16_t i=0; i<leds.numPixels(); i++) {
    leds.setPixelColor(i, c);
    leds.show();
    delay(wait);
  }
  #endif
}
#line 1 "mpu6050.ino"
inline float get_last_x_angle();
inline float get_last_y_angle();
inline float get_last_z_angle();
inline float get_last_gyro_x_angle();
inline float get_last_gyro_y_angle();
inline float get_last_gyro_z_angle();
float angle_x;
float angle_y;
float angle_z;


void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}

                                                                                         
void calibrate_sensors() {
  mpu.setDLPFMode(2);                               
  mpu.setDHPFMode(4);                                          
  mpu.setFullScaleAccelRange(0);   
  mpu.setFullScaleGyroRange(2);                 

  Serial.println(F("Reading/Updating internal sensor offsets..."));
                         
                       
                                             
  Serial.print(mpu.getXAccelOffset());
  Serial.print(F("\t"));       
  Serial.print(mpu.getYAccelOffset());
  Serial.print(F("\t"));         
  Serial.print(mpu.getZAccelOffset());
  Serial.print(F("\t"));        
  Serial.print(mpu.getXGyroOffset());
  Serial.print(F("\t"));     
  Serial.print(mpu.getYGyroOffset());
  Serial.print(F("\t"));     
  Serial.print(mpu.getZGyroOffset());
  Serial.print(F("\t"));     
  Serial.print(F("\n"));
                  
                                    
  mpu.setXGyroOffset(-10);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(68);

  Serial.print(mpu.getXAccelOffset());
  Serial.print(F("\t"));       
  Serial.print(mpu.getYAccelOffset());
  Serial.print(F("\t"));         
  Serial.print(mpu.getZAccelOffset());
  Serial.print(F("\t"));        
  Serial.print(mpu.getXGyroOffset());
  Serial.print(F("\t"));     
  Serial.print(mpu.getYGyroOffset());
  Serial.print(F("\t"));     
  Serial.print(mpu.getZGyroOffset());
  Serial.print(F("\t"));     
  Serial.print(F("\n"));
  
  
                              
  
  int                   num_readings = 10;                         
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  
  Serial.println(F("Starting calibration"));

                                                      
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                
                                                 
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
  
                                              
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
  
  Serial.println(F("Calibration finished"));
}

void mpucheck() {
                                                 
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                                                        
                                       
                                   

#ifdef OUTPUT_READABLE_mpu
if (DEBUG) {
                                                  
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
}
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

void getangle()
{
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                        
  double dT;
  unsigned long t_now = millis();                                                       

                                       
                                  
  float FS_SEL = 16;              
  float gyro_x = (gx - base_x_gyro)/FS_SEL;
  float gyro_y = (gy - base_y_gyro)/FS_SEL;
  float gyro_z = (gz - base_z_gyro)/FS_SEL;
  
  float accel_x = ax;
  float accel_y = ay;
  float accel_z = az;
  
                                        
  float RADIANS_TO_DEGREES = 180/3.14159;
                                                                                         
  float accel_angle_y = atan(-1*accel_x/sqrt(pow(accel_y,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_x = atan(accel_y/sqrt(pow(accel_x,2) + pow(accel_z,2)))*RADIANS_TO_DEGREES;
  float accel_angle_z = 0;
  
                                       
  float dt =(t_now - get_last_time())/1000.0;
  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
  float gyro_angle_z = gyro_z*dt + get_last_z_angle();
  
                                     
  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();
  
                                                                                          
                                                          
float alpha = 0.87;
angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
angle_z = gyro_angle_z;                                      
  
                                                 
  set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);
  if (DEBUG) {
                                     
                                                    
                        
                                                              
                                 
                       
                                 
                       
                                 
                           
                                                                    
                       
                                           
                       
                                           

                                                            
                           
                        
                           
                        
                           
                        
    
 } 
}
#line 1 "packet_types.ino"
                 
                                          
                                          
                            
                                             
  
                                      
                                       
          
                     
                                       
                                                
   

char receive_payload[32 + 1];                                                

byte br = 50;                             
byte _r, _g, _b, _c1, _c2, _c3 = 50;
const int PROGMEM breakpoint = 32;
void send_K(unsigned int to) {
  byte _actions = 0;
  unsigned int GMAX = 32000;
  mpucheck();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
                                                                                                     

  if ( ay > (+15000) ) {
    Serial.println(F("RIGHT"));
    int _cd = abs(gy) - breakpoint;                                                           
    _cd = (max(_cd, breakpoint)) - breakpoint;
    _cd = map(_cd, 0, GMAX, 0, 255);
    if (gy > 0) {
      if (((int)_c1 - _cd) < 0) {
        _c1 = 0;
        vibr(2);
        _actions++;
      } else {
        _c1 -= _cd;
      }
    }
    if (gy <= 0) {
      if (((int)_c1 + _cd) > 255) {
        _c1 = 255;
        vibr(2);
        _actions++;
      } else {
        _c1 += _cd;
      }
    }
  }

  if ( ay < (-10000) ) {                                
    Serial.println(F("LEFT/TOP"));

                                               
                                
                                 
                                      
                                      
  }

  if ( ax > (+15000) ) {
    Serial.println(F("BACK"));
    int _cd = abs(gx) - breakpoint;
    _cd = (max(_cd, breakpoint)) - breakpoint;
    _cd = map(_cd, 0, GMAX, 0, 255);
    if (gx > 0) {
      if (((int)_c3 - _cd) < 0) {
        _c3 = 0;
        vibr(2);
        _actions++;
      } else {
        _c3 -= _cd;
      }
    }
    if (gx <= 0) {
      if (((int)_c3 + _cd) > 255) {
        _c3 = 255;
        vibr(2);
        _actions++;
      } else {
        _c3 += _cd;
      }
    }
  }

  if ( ax < (-15000) ) {
    Serial.println(F("FRONT"));
    int _cd = abs(gx) - breakpoint;
    _cd = (max(_cd, breakpoint)) - breakpoint;
    _cd = map(_cd, 0, GMAX, 0, 255);
    if (gx > 0) {
      if (((int)_c2 + _cd) > 255) {
        _c2 = 255;
        vibr(2);
        _actions++;
      } else {
        _c2 += _cd;
      }
    }
    if (gx <= 0) {
      if (((int)_c2 - _cd) < 0) {
        _c2 = 0;
        vibr(2);
        _actions++;
      } else {
        _c2 -= _cd;
      }
    }
  }

                                   

  if ( az > 15000 ) {
    Serial.println(F("TOP"));
    int _cd = abs(gz) - breakpoint;
    _cd = (max(_cd, breakpoint)) - breakpoint;
    _cd = map(_cd, 0, GMAX, 0, 255);
    if (gz > 0) {
      if (((int)br - _cd) < 0) {
        br = 0;
        vibr(2);
        _actions++;
      } else {
        br -= _cd;
      }
    }
    if (gz <= 0) {
      if (((int)br + _cd) > 255) {
        br = 255;
        vibr(2);
        _actions++;
      } else {
        br += _cd;
      }
    }
  }

  if ( az < (-15000) ) {
    Serial.println(F("BOT"));

    if (strobe) br = 255;
    if (!strobe) br = 0;
    _actions++;
  }

  Serial.print(_actions, DEC);
  Serial.println(F(" ACT"));
  if (_actions > 0) active();

  if ( (abs(angle_x) < 10) & (abs(angle_y) < 10) ) {
  }

                            
  if (DEBUG) {
  Serial.print(br);
  }
                                                                 
                                                                 
  
  _r = (unsigned int) _c1 * br / 255;
  _g = (unsigned int) _c2 * br / 255;
  _b = (unsigned int) _c3 * br / 255;
  if (DEBUG) {
  Serial.print(F("\t _r: "));
  Serial.print(_r);
  Serial.print(F("\t _g: "));
  Serial.print(_g);
  Serial.print(F("\t _b: "));
  Serial.println(_b);
  }
                                          
                                        
                                        
                                          
                                          
                                          
      
                     
                                     
                                               
                                 
                                 
                                 
                  
                  
                    
      
  
  byte kmap[24] = {
    _r, _g, _b,                   
    _r, _g, _b,
    _r, _g, _b,
    _r, _g, _b,
    _r, _g, _b,
    _r, _g, _b,
    _r, _g, _b,
    _r, _g, _b
  };


  unsigned long now = millis();
  
                               
  
  bool ok = radio.write(kmap, sizeof(kmap));
  
  if (DEBUG) {
    if (ok) Serial.println(F("send_K okay"));
    if (!ok) Serial.println(F("send_K timout"));                      
  }
}

void send_L1(int to, int _b = 0) {
  if ( to != this_node) {
    byte ledmap[24] = {
      1, 2, 3,
      0, 0, _b,
      0, 0, _b,
      0, 0, _b,
      0, 0, _b,
      0, 0, _b,
      0, 0, _b,
      0, (((byte) millis() & 0xFF) / 10), _b
    };

    unsigned long now = millis();
    bool ok = 1;                      
    if (DEBUG) {
      p(" in %ld ms.\n", (millis() - now) );
      if (ok) {}
      if (!ok) p("%010ld: send_L1 timout.\n", millis());                      
    }
    ledst();
  }
}

boolean send_T(uint16_t to)                                          
{
  if (DEBUG) p("%010ld: Sent 'T' to   %05o", millis(), to);
                                     
  unsigned long time = micros();
  return radio.write(&time, sizeof(time));
}

boolean send_L(uint16_t to, byte* ledmap)                       
{
  if (DEBUG) p("%010ld: 'L' to   %05o", millis(), to);
  return radio.write(ledmap, sizeof(ledmap));
}

#ifdef USE_LEDS
void ledupdate(byte* ledmap) {
  for (uint8_t i = 0; i < leds.numPixels(); i++) {
    uint32_t c = leds.Color(ledmap[i * 3], ledmap[(i * 3) + 1], ledmap[(i * 3) + 2]);
    leds.setPixelColor(i, c);
  }
  leds.show();
}
#endif

void handle_K()
{
  byte kmap[24];

  if (DEBUG) p("%010ld: 'K' from %05o\n", millis(), receive_payload[0]);
  byte ledmap[24] = {
    000, 000, 000,                   
    kmap[0], kmap[1], kmap[2],              
    kmap[3], kmap[3], kmap[3],          
    kmap[4], kmap[4], kmap[4],             
    kmap[5], kmap[5], kmap[5],
    000, 000, 00,
    000, 000, 00,
    000, 000, 0
  };
#ifdef USE_LEDS
  ledupdate(ledmap);
#endif

  if (DEBUG) {
    for (uint16_t i = 0; i < sizeof(kmap); i++) {                                            
      Serial.print(kmap[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void handle_T()
{
  unsigned long time;
                                            
  if (DEBUG) {
                                                                              
  }
                               
                                      
  {
                                                                            
    unsigned long nowM = micros();
                                                     
    if (DEBUG) {
                                                                         
                                            
                                       
    }
  }
}

void handle_B()
{
  p_recv++;
  unsigned long ref_time;
                                                    
  if (DEBUG) {
                                                                                                             
  }
}

void processPacket() {
  ledst(3);                                       
  uint8_t len = 0;
  bool done = 0;
  while (!done)
  {
                                                          
    len = radio.getDynamicPayloadSize();
    done = radio.available();
                                              
    receive_payload[len] = 0;
                   
    p("  Recv size=%i value=%s\n\r", len, receive_payload);
  }
  switch (receive_payload[0])                                       
  {
    case 'K':
      handle_K();
      break;
    case 'L':
                   
      break;
    case 'T':
      handle_T();
      break;
    case 'B':
      handle_B();
      break;
    default:
                                                                                 
      if (DEBUG) {
        Serial.print(F("undefined packet type: "));                                      
        Serial.print(receive_payload[0]);
        Serial.println();
      }
      break;
  };
  ledst();                                               
}
#line 1 "vibration.ino"
                               
                    
                                                
   
                    
                                                 
                                                                      
   
                   
          
            
          
   
 
   

                                                    
void pulse(unsigned int _length) {
  vibr(1);
  delay(_length);
  vibr(0);
  }
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
  if (_state == 4){                          
  pulse(70);
  delay(170);
  pulse(38);
  }
  if (_state == 5){                          
  pulse(80);
  delay(290);
  pulse(50);
  }

  
  if (_state == 3){
                                                                                 
}
}

