#include "Arduino.h"
inline unsigned long get_last_time();

void active();
void ledst(int sta);
void colorWipe(uint32_t c, uint8_t wait);
void send_K(unsigned int to);
boolean send_L(uint16_t to, byte* ledmap);
void processPacket();
#line 2
__asm volatile ("nop"); // BOF preprocessor bug workaround

//#define USE_LEDS // LED stripe used
//#define USE_TOUCH // capacitive touch sensing

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of the project
#include "I2Cdev.h"
#include <avr/sleep.h>
#include <avr/power.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation is used in I2Cdev.h
#include "Wire.h"
#endif
#include <RF24.h>
#include <SPI.h>
#include <stdarg.h>
#include <EEPROM.h>
#define DEBUG 1 // debug mode with verbose output over serial at 115200 bps
#define USE_EEPROM // read nodeID and network settings from EEPROM at bootup, overwrites nodeID and MAC.
#define LEDPIN 6
#define KEEPALIVE 1 // keep connections alive with regular polling to node 0
//#define USE_LEDS // LED stripe used
#define TIMEOUT_HIBERNATE 512
#define TIMEOUT_MEMS 5000
#define TIMEOUT_RADIO 8000
#define PIN_VIBR 5 // pin for vibration motor, 9 for switchcube1, 5 for prototype2

#ifdef USE_LEDS
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel leds = Adafruit_NeoPixel(8, LEDPIN, NEO_GRB + NEO_KHZ800); // number of pixels in strip, pin number, pixel type flags
#endif
RF24 radio(A0, 10); // CE, CS. CE at pin A0, CSN at pin 10

const unsigned long interval_filter = 32;
const unsigned long interval = 24; // main loop interval in [ms]
unsigned long last_time_filtered;
byte sweep = 0;
byte nodeID = 2; // Unique Node Identifier (2...254) - also the last byte of the IPv4 adress, not used if USE_EEPROM is set
uint16_t this_node = 00; // always begin with 0 for octal declaration, not used if USE_EEPROM is set
// Debug variables, TODO: don't initialize if DEBUG is set to 0
unsigned long iterations = 0;
unsigned long errors = 0;
unsigned int loss = 0;
unsigned long p_sent = 0;
unsigned long p_recv = 0;

const short max_active_nodes = 32; // the size of the array with active nodes
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;
unsigned long last_time_sent;
unsigned long updates = 0; // has to be changed to unsigned long if the interval is too long
boolean send_T(uint16_t to);
void send_L1(int to, int _b);
void handle_K(char header);
void handle_L(char header);
void handle_T(char header);
void handle_B(char header);
void ledupdate(byte* ledmap);
void p(char *fmt, ... );
void ledst(int sta = 127);
bool strobe = 1;

// Sleep declarations
typedef enum { wdt_16ms = 0, wdt_32ms, wdt_64ms, wdt_128ms, wdt_250ms, wdt_500ms, wdt_1s, wdt_2s, wdt_4s, wdt_8s } wdt_prescalar_e;
void setup_watchdog(uint8_t prescalar);
void do_sleep(void);
const short sleep_cycles_per_transmission = 10;
volatile short sleep_cycles_remaining = sleep_cycles_per_transmission;

void powerMode(byte _mode);
void vibr(byte _state);
void pulse(unsigned int _length = 50);
byte mode = 0;
unsigned long lastActive = 0;

// The various roles supported by this sketch
typedef enum { tx = 1, rx } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "unassigned", "tx", "rx"};

// The role of the current running sketch
role_e role;

const int role_pin = 0; // TODO: hardware role switch

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xAEAEAEAEA0LL, 0xAEAEAEAEA1LL };


void setup() {
  pinMode(PIN_VIBR, OUTPUT); // vibration motor
  vibr(0); // pull low and disable motor for now
  vibr(1); // vibrate during bootup
#ifdef USE_LEDS
  leds.begin(); // the 8 LEDs
  leds.show(); // Initialize all pixels to 'off'
  ledst(5); // set initial status to 5, ledst() sets the first
#endif
  Serial.begin(115200); // initialize serial communication
  delay(64);
  vibr(0); // stop vibrating
  // initialize devices
  Serial.println(F("avoccado-tx \'holz\' 0.201412152310"));
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
  radio.enableDynamicPayloads();   // enable dynamic payloads
  radio.setRetries(4, 8);  // optionally, increase the delay between retries & # of retries
  radio.setChannel(42);
  Serial.print(F("\t rate: "));
  Serial.println(radio.getDataRate());
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  Serial.print(F("PA-level: "));
  Serial.print(radio.getPALevel());
  Serial.print(F("\t CRC: "));
  Serial.print(radio.getCRCLength());
  Serial.print(F("\t set rate to 250K: "));
  Serial.println(radio.getDataRate());
  Serial.println();

  if ( role_pin == 1 )
    role = tx;
  else
    role = rx;

  p("\t role: %s\n\r", role_friendly_name[role]);
  if ( role == tx )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1, pipes[0]);
  }
  radio.powerUp();
  radio.startListening();
  radio.printDetails();
  Serial.println();
  Serial.print(F("UID: "));
  Serial.print(F("unassigned prototype\n"));
  p("%010ld: Starting up\n", millis());
#ifdef USE_LEDS
  colorWipe(leds.Color(50, 0, 0), 50); // Red
  colorWipe(leds.Color(0, 50, 0), 50); // Green
  colorWipe(leds.Color(0, 0, 50), 50); // Blue
  colorWipe(leds.Color(0, 0, 0), 0); // clear
  ledst(5);
#endif
  setup_watchdog(wdt_1s); // Set the watchdog timer interval
}

void powerMode(byte _mode = mode) {
  switch (_mode) // check which power mode should be set active
  {
    case 0:
      break; // no change in power mode
    case 1:
      Serial.println(F("powerMode(1), MEMS up"));
      mode = 0;
      break;
    case 2:
      radio.powerUp(); // activate the radio
      Serial.println(F("powerMode(2), radio up"));
      mode = 0;
      break;
    case 5:
      Serial.println(F("powerMode(5), hibernate"));
      radio.powerDown();
      Serial.print(F("Sleeping."));
      vibr(1);
      delay(64);
      vibr(0);
      delay(196);
      vibr(1);
      delay(64);
      vibr(0);
      setup_watchdog(wdt_250ms); // set activity check interval
      while ( 1 ) { // if the Avoccado device is resting in this position
        do_sleep(); // keep on sleeping
        Serial.print(F("."));
      }
      for (int _i = 0; _i < 2; _i++) { // vibrate briefly when waking up from hibernation mode
        vibr(1);
        delay(128);
        vibr(0);
        delay(56);
      }
      Serial.println();
      powerMode(1); // power up MEMS
      powerMode(2); // power up radio
      active();
      break;
    case 6: // shutdown until next hard reset
      Serial.println(F("powerMode(6), shutdown"));
      radio.powerDown();
      vibr(4); // descending heartbeat pattern
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
      setup_watchdog(wdt_8s); // longest watchdog interval
      while (1) do_sleep(); // sleep forever
      break;
    default: // no change in power mode
      return;
  };
}


void active() {
  lastActive = millis();
}

void loop() {

  /////////////////////////
  //powerMode(6); // WIP - shutdown forever, disable hardware until next hard reset
  /////////////////////////

//  if (millis() - lastActive > TIMEOUT_HIBERNATE && ((ay < (-10000)) || ay > (10000))) powerMode(5);
  //  if (millis()-lastActive> TIMEOUT_MEMS) mpu.setSleepEnabled(1);
  //  if (millis()-lastActive> TIMEOUT_RADIO) radio.powerDown();
  updates++;

if ( radio.available() ) // while there are packets in the FIFO buffer
  {
    processPacket();
  }

  unsigned long now = millis();
  unsigned long nowM = micros();

  if ( now - last_time_sent >= interval ) // non-blocking check for start of debug service routine interval
  {
    //ledst(2);
    /* if (DEBUG) {
      p("%010ld: %ld Hz\n", millis(), updates * 1000 / interval);
    } */
    updates = 0;
    last_time_sent = now;
//    radio.powerUp();
//    send_K(1); // send out a new packet to a remote node.
//    radio.powerDown();
  }

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


// Arduino version of the printf()-funcition in C
void p(char *fmt, ... ) {
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}
