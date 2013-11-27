#if 1 // BOF preprocessor bug workaround
__asm volatile ("nop");
#endif

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of the project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <avr/sleep.h>
#include <avr/power.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <stdarg.h>
#include <EEPROM.h>
#include <CapacitiveSensor.h>
#define DEBUG 1 // debug mode with verbose output over serial at 115200 bps
#define USE_EEPROM // read nodeID and network settings from EEPROM at bootup, overwrites nodeID and MAC.
#define LEDPIN 6
#define KEEPALIVE 0 // keep connections alive with regular polling to node 0
#define USE_LEDS // LED stripe used
//#define USE_TOUCH // capacitive touch sensing

#ifdef USE_LEDS
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel leds = Adafruit_NeoPixel(8, LEDPIN, NEO_GRB + NEO_KHZ800); // number of pixels in strip, pin number, pixel type flags
#endif
RF24 radio(A0, 10); // CE, CS. CE at pin A0, CSN at pin 10
RF24Network network(radio); // mesh network layer

#ifdef USE_TOUCH
CapacitiveSensor cs = CapacitiveSensor(8,7);        // capacitive sensing at pins 7 and 8
#endif

const unsigned long interval_filter = 10;
const unsigned long interval = 100; // KEEPALIVE interval in [ms]
unsigned long last_time_filtered;
byte sweep = 0;
byte nodeID = 1; // Unique Node Identifier (2...254) - also the last byte of the IPv4 adress, not used if USE_EEPROM is set
uint16_t this_node = 00; // always begin with 0 for octal declaration, not used if USE_EEPROM is set
// Debug variables, TODO: don't initialize if DEBUG is set to 0
unsigned long iterations = 0;
unsigned long errors = 0;
unsigned int loss = 0;
unsigned long p_sent = 0;
unsigned long p_recv = 0;
// Variables for the 32bit unsigned long Microsecond rollover handling
static unsigned long microRollovers = 0; // variable that permanently holds the number of rollovers since startup
static unsigned long halfwayMicros = 2147483647; // this is halfway to the max unsigned long value of 4294967296
static boolean readyToRoll = false; // tracks whether we've made it halfway to rollover

const short max_active_nodes = 32; // the size of the array with active nodes
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;
unsigned long last_time_sent;
unsigned long updates = 0; // has to be changed to unsigned long if the interval is too long
void add_node(uint16_t node);
boolean send_T(uint16_t to);
void send_L1(int to, int _b);
void handle_K(RF24NetworkHeader& header);
void handle_L(RF24NetworkHeader& header);
void handle_T(RF24NetworkHeader& header);
void handle_B(RF24NetworkHeader& header);
void ledupdate(byte* ledmap);
void p(char *fmt, ... );
void ledst(int sta = 127);
void mpucheck();
bool strobe = 1;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // alternative I2C address, if AD0 is set high

int16_t ax, ay, az; // accel values
int16_t gx, gy, gz; // gyro values

#define OUTPUT_READABLE_mpu // tab-separated list of the accel X/Y/Z and then gyro X/Y/Z values in decimal.
// #define OUTPUT_BINARY_mpu //send all 6 axes of data as 16-bit

unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
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

//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;
void setup_watchdog(uint8_t prescalar);
void do_sleep(void);
const short sleep_cycles_per_transmission = 10;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;
void mode(byte _mode);
void vibr(byte _state);

void setup() {
// ONLY for C3POW prototype 2:
/*  pinMode(7,OUTPUT);
  digitalWrite(7, HIGH); // Vcc for MPU6050
  pinMode(8, OUTPUT); // Vcc for the NRF24 module, 3.5-5V output to an LDO supplying 3.3V
  digitalWrite(8, HIGH); // Vcc for the NRF24 module activated. Shutdown with LOW.
  */

  vibr(0); // pull low and disable motor for now
#ifdef USE_LEDS
  leds.begin(); // the 8 LEDs
  leds.show(); // Initialize all pixels to 'off'
  ledst(5); // set initial status to 5, ledst() sets the first
#endif
  #ifdef USE_TOUCH
  cs.set_CS_AutocaL_Millis(64000);
  #endif
  Serial.begin(115200); // initialize serial communication
  delay(64);
  // initialize devices
  Serial.println(F("C3POW 0.20131127"));
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Serial.println(F("I2CSetup"));
  Wire.begin(); // join I2C bus (I2Cdev library doesn't do this automatically)
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
  SPI.begin(); // SPI for the NRF24
  radio.begin(); // init of the NRF24
  // The amplifier gain can be set to RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_MAX=0dBm.
  radio.setPALevel(RF24_PA_MAX); // transmitter gain value (see above)
  radio.printDetails();
  network.begin( 1, this_node ); // fixed radio channel, node ID
  Serial.print(F("UID: "));
  Serial.print(F("(undefined)\n"));
  p("%010ld: Starting up\n", millis());
#ifdef USE_LEDS
  colorWipe(leds.Color(50, 0, 0), 50); // Red
  colorWipe(leds.Color(0, 50, 0), 50); // Green
  colorWipe(leds.Color(0, 0, 50), 50); // Blue
  colorWipe(leds.Color(0, 0, 0), 0); // clear
  ledst(5);
#endif

  // verify connection
  Serial.println(F("Init I2C..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connected." : "MPU6050 error.");
  mpu.setDLPFMode(2); // 2 = 100Hz, 6 = 5Hz low pass
  mpu.setDHPFMode(4); // 0.625Hz high pass for motion detection
  mpu.setFullScaleAccelRange(0); //
  mpu.setFullScaleGyroRange(3); // 2000deg/s
  //    Wire.write(0x16);            // register DLPF_CFG - low pass filter configuration & sample rate
  //    Wire.write(0x1D);            //   10Hz Low Pass Filter Bandwidth - Internal Sample Rate 1kHz

  Serial.println(F("Reading/Updating internal sensor offsets..."));
  // -76	-2359	1688	0	0	0
  //-596	-585	664	0	0	0
  //-2648	823	808	-10	9	68	switchcube alpha 2
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
  // -596	-585	664
  // change accel/gyro offset values
  mpu.setXGyroOffset(-10);
  mpu.setYGyroOffset(9);
  mpu.setZGyroOffset(68);

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

  calibrate_sensors();
  set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);
#ifdef USE_LEDS
  ledst(1);
#endif
  setup_watchdog(wdt_1s); // Set the watchdog timer interval
}

void mode(byte _mode) {
  if (_mode == 0) {
    Serial.println(F("mode(0)"));
    radio.powerDown();
    mpu.setSleepEnabled(1);
    vibr(1);
    delay(256);
    int _j = 3;
    for (int _i = 0; _i < _j; _i++) {
      vibr(1);
      delay(128);
      vibr(0);
      delay(256);
    }
    setup_watchdog(wdt_8s);
    while (1) do_sleep(); // sleep forever
  }
}

void loop() {
//  mode(0);
  wilssen();
  network.update();
  send_K(01);
  updates++;
  while ( network.available() ) // while there are packets in the FIFO buffer
  {
#ifdef USE_LEDS
    ledst(3); // light up status LED with pattern #3
#endif
    RF24NetworkHeader header; // initialize header
    network.peek(header); // preview the header, but don't advance nor flush the packet
    switch (header.type) // check which packet type we received
    {
      case 'K':
        handle_K(header);
        break;
      case 'L':
        handle_L(header);
        break;
      case 'T':
        handle_T(header);
        break;
      case 'B':
        handle_B(header);
        break;
      default:
        network.read(header, 0, 0); // if none of the above packet types matched, read out and flush the buffer
        if (DEBUG) {
          Serial.print(F("            undefined packet type: ")); // print the unrecognized packet type
          Serial.print(header.type);
          Serial.println();
        }
        break;
    };
    ledst(); // reset the status LED to the default pattern
  }
  unsigned long now = millis();
  unsigned long nowM = micros();
  if ( now - last_time_filtered >= interval_filter ) { // non-blocking check for start of debug service routine interval
    getangle();
    last_time_filtered = now;
  }

  if ( now - last_time_sent >= interval ) // non-blocking check for start of debug service routine interval
  {
    ledst(2);
    /* // unsigned long int rollover checking:
     Serial.print(microsRollover()); // how many times has the unsigned long micros() wrapped?
     Serial.print(":"); //separator
     Serial.print(nowM); //micros();
     Serial.print("\n"); //new line
     */
    if (DEBUG) {
      p("%010ld: %ld Hz\n", millis(), updates * 1000 / interval);
    }
    updates = 0;
    last_time_sent = now;
    if (KEEPALIVE) {
      uint16_t to = 000;
      bool ok = 0;
      if ( to != this_node)
      {
        unsigned long nowM = micros();
        ok = send_T(to);
        if (DEBUG) p(" in %ld us.\n", (micros() - nowM) );
        if (ok) p_sent++;
        if (!ok)
        {
          errors++;
          if (DEBUG) {
            p("%010ld: ACK timeout.\n", millis()); // An error occured, need to stahp!
          }
        }
        iterations++;
      }
    }
    sweep += 1;
    if (sweep > 254) sweep = 0;
    strobe = !strobe;
    if ( this_node == 00) {
      for (short _i = 0; _i < num_active_nodes; _i++) {
        send_K(active_nodes[_i]);
      }
    }
  }
 
  /*  
  long csVal = cs.capacitiveSensor(30);
  Serial.println(csVal);
  delay(50);
  */
  /*
  radio.powerDown();
  Serial.print(F("MotionDetectionDuration: "));
  Serial.print(mpu.getMotionDetectionDuration());
  Serial.print(F("\t Threshold: "));
  Serial.print(mpu.getMotionDetectionThreshold());
  Serial.print(F("\t gyroRange: "));
  Serial.print(mpu.getFullScaleGyroRange());
  Serial.print(F("\t getClockSource: "));
  Serial.print(mpu.getClockSource());
  Serial.print(F("\n"));
  Serial.print(F("Radio down.\n"));
  delay(1000);

  Serial.print(F("+MCU into sleep mode for 10s.\n"));
  delay(50);
  while ( sleep_cycles_remaining ) do_sleep(); // Sleep the MCU, the watchdog timer will awaken in a short while.
  sleep_cycles_remaining = sleep_cycles_per_transmission;

  Serial.print(F("+Xgyro now sleeping for 10s.\n"));
  mpu.setStandbyXGyroEnabled(1);
  delay(50);
  while ( sleep_cycles_remaining ) do_sleep(); // Sleep the MCU, the watchdog timer will awaken in a short while.
  sleep_cycles_remaining = sleep_cycles_per_transmission;

  Serial.print(F("+Ygyro now sleeping for 10s.\n"));
  mpu.setStandbyYGyroEnabled(1);
  delay(50);
  while ( sleep_cycles_remaining ) do_sleep(); // Sleep the MCU, the watchdog timer will awaken in a short while.
  sleep_cycles_remaining = sleep_cycles_per_transmission;

  Serial.print(F("+Zgyro now sleeping for 10s.\n"));
  mpu.setStandbyZGyroEnabled(1);
  delay(50);
  while ( sleep_cycles_remaining ) do_sleep(); // Sleep the MCU, the watchdog timer will awaken in a short while.
  sleep_cycles_remaining = sleep_cycles_per_transmission;

  Serial.print(F("+setSleepEnabled(1) for 10s.\n"));
  mpu.setSleepEnabled(1);
  delay(50);
  while ( sleep_cycles_remaining ) do_sleep(); // Sleep the MCU, the watchdog timer will awaken in a short while.
  sleep_cycles_remaining = sleep_cycles_per_transmission;

  Serial.print(F("MCU now sleeping for 10s with power to radio cut off.\n"));
  delay(127);
  while ( sleep_cycles_remaining ) do_sleep(); // Sleep the MCU, the watchdog timer will awaken in a short while.
  Serial.print(F("MCU awake. 1s delay to radio powerup.\n"));
  digitalWrite(8, HIGH);
  digitalWrite(7, HIGH);
  delay(1000);
  mpu.setSleepEnabled(0);
  radio.powerUp();
  Serial.print(F("Radio up.\n"));
  sleep_cycles_remaining = sleep_cycles_per_transmission;
  */
}

// 0=16ms, 1=32ms,2=64ms,3=125ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out
}


void wilssen() {

}

// Arduino version of the printf()-funcition in C
void p(char *fmt, ... ) {
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}

void add_node(uint16_t node) //TODO: remove_node, after a certain timeout...
{
  short i = num_active_nodes;
  while (i--)
    if ( active_nodes[i] == node ) break; // Do we already know about this node?
  if ( i == -1 && num_active_nodes < max_active_nodes )  // If not and there is enough place, add it to the table
  {
    active_nodes[num_active_nodes++] = node;
    if (DEBUG) {
      p("%010ld: New node: %05o\n", millis(), node);
    }
  }
}

unsigned long microsRollover() { //based on Rob Faludi's (rob.faludi.com) milli wrapper

  // This would work even if the function were only run once every 35 minutes, though typically,
  // the function should be called as frequently as possible to capture the actual moment of rollover.
  // The rollover counter is good for over 584000 years of runtime.
  //  --Alex Shure

  unsigned long nowMicros = micros(); // the time right now

  if (nowMicros > halfwayMicros) { // as long as the value is greater than halfway to the max
    readyToRoll = true; // we are in the second half of the current period and ready to roll over
  }

  if (readyToRoll == true && nowMicros < halfwayMicros) {
    // if we've previously made it to halfway
    // and the current millis() value is now _less_ than the halfway mark
    // then we have rolled over
    microRollovers++; // add one to the count of rollovers (approx 71 minutes)
    readyToRoll = false; // we're no longer past halfway, reset!
  }
  return microRollovers;
}



