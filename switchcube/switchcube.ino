
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of the project
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <stdarg.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#define DEBUG 1 // debug mode with verbose output over serial at 115200 bps
#define USE_EEPROM // read nodeID and network settings from EEPROM at bootup, overwrites nodeID and MAC.
#define LEDPIN 6
#define KEEPALIVE 0 // keep connections alive with regular polling to node 0

Adafruit_NeoPixel leds = Adafruit_NeoPixel(8, LEDPIN, NEO_GRB + NEO_KHZ800); // number of pixels in strip, pin number, pixel type flags

RF24 radio(A0,10); // CE, CS. CE at pin A0, CSN at pin 10
RF24Network network(radio); // mesh network layer 

const unsigned long interval = 10000; // KEEPALIVE interval in [ms]
byte sweep=0;
byte nodeID = 1; // Unique Node Identifier (2...254) - also the last byte of the IPv4 adress, not used if USE_EEPROM is set
uint16_t this_node = 00; // always begin with 0 for octal declaration, not used if USE_EEPROM is set
// Debug variables, TODO: don't initialize if DEBUG is set to 0
unsigned long iterations=0;
unsigned long errors=0;
unsigned int loss=0;
unsigned long p_sent=0;
unsigned long p_recv=0;
// Variables for the 32bit unsigned long Microsecond rollover handling
static unsigned long microRollovers=0; // variable that permanently holds the number of rollovers since startup
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
void ledst(int sta=127);
void mpucheck();
bool strobe=1; 

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

void setup() {
  pinMode(7,OUTPUT);
  digitalWrite(7, HIGH); // Vcc for MPU6050  
  pinMode(A6, INPUT); // some nodes have a sense wire to the 3v3 for the NRF24 module via external LDO
  pinMode(A7, INPUT); // see above
  leds.begin(); // the 8 LEDs
  leds.show(); // Initialize all pixels to 'off'
  ledst(5);
  pinMode(A1, OUTPUT); // GND for the NRF24 module
  digitalWrite(A1, LOW); // GND for the NRF24 module
  pinMode(5, OUTPUT); // Vcc for the NRF24 module, 3.5-5V output to an LDO supplying 3.3V
  digitalWrite(5, HIGH); // Vcc for the NRF24 module activated. Shutdown with LOW.
  Serial.begin(115200); // initialize serial communication
  delay(128);
  // initialize devices
  Serial.println(F("Initializing devices..."));
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
  nodeID=EEPROM.read(0);
  Serial.print(F("EEPROM, "));
#endif
  Serial.println(nodeID);
  Serial.print(F("Network ID (oct): "));
#ifdef USE_EEPROM
  this_node = ((int) EEPROM.read(15)*256) + ((int) (EEPROM.read(16)));
  Serial.print(F("EEPROM, "));
#endif
  Serial.print(this_node,OCT);
  Serial.print(F(", (dec): "));
  Serial.print(this_node,DEC);
  Serial.println();
  SPI.begin(); // SPI for the NRF24
  radio.begin(); // init of the NRF24
  // The amplifier gain can be set to RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_MAX=0dBm.
  radio.setPALevel(RF24_PA_MAX); // transmitter gain value (see above)
  network.begin( 1, this_node ); // fixed radio channel, node ID
  Serial.print(F("UID: \n"));
  p("%010ld: Starting up\n", millis());
  colorWipe(leds.Color(100, 0, 0), 50); // Red
  colorWipe(leds.Color(0, 100, 0), 50); // Green
  colorWipe(leds.Color(0, 0, 100), 50); // Blue
  colorWipe(leds.Color(0, 0, 0), 0); // clear
  ledst(5);
  
  // verify connection
  Serial.println(F("testing I2C device connections..."));
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setDLPFMode(6); // 5Hz low pass
  // mpu.setDHPFMode(4); // 0.625Hz high pass
  mpu.setFullScaleAccelRange(0); //
  mpu.setFullScaleGyroRange(3); // 2000deg/s
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
  mpu.setXGyroOffset(21);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(9);
  
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
  ledst(1);
}

void loop() {
  network.update();
  updates++;
  while ( network.available() ) // while there are packets in the FIFO buffer
  {
    ledst(3); // light up status LED with pattern #3
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
      network.read(header,0,0); // if none of the above packet types matched, read out and flush the buffer
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
      p("%010ld: %ld Hz\n",millis(),updates*1000/interval);
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
        if (DEBUG) p(" in %ld us.\n", (micros()-nowM) );
        if (ok) p_sent++;
        if (!ok)
        {
          errors++;
          if (DEBUG) {
            p("%010ld: No ACK timeout.\n", millis()); // An error occured, need to stahp!
          }
        }
        iterations++;
        /*
      Serial.print("loop: \t\t");
         Serial.println(iterations);
         Serial.print("errors: \t\t");
         Serial.println(errors);     
         Serial.print("send error in %: \t");
         Serial.println(errors*100/iterations);
         Serial.print("pkts sent    : \t");
         Serial.println(p_sent);
         Serial.print("pkts received: \t");
         Serial.println(p_recv);
         Serial.print("replies in %: \t");
         Serial.println(p_recv*100/(p_sent-1));
         */
      }
    }
    sweep+=100;
    if (sweep>254) sweep=0;
    /*
    if ( this_node == 00){
      for (short _i=0; _i<num_active_nodes; _i++) {
        send_L1(active_nodes[_i],(byte) sweep&0xFF);
      }
      strobe=!strobe;
    }
    */
    if ( this_node == 00){
      send_K(01);
      //send_L1(00,(byte) sweep&0xFF);
      strobe=!strobe;
    }
  }
}

// Arduino version of the printf()-funcition in C 
void p(char *fmt, ... ){
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

