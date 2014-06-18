/*
 Avoccado
 Open Source Haptic Input Device.
 WIP Adhoc networking. No meshing.
 Licensing information in the repository, see file LICENSE.
 */

#if 1 // BOF preprocessor bug workaround
#define HW 1 // define hardware revision, 1=switchcube, 2=C3POWproto2, ... for different pin assignments and peripheral hardware
#include "Arduino.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#line 2
__asm volatile ("nop"); // BOF preprocessor bug workaround
#endif

//
// Prototypes
//

inline unsigned long get_last_time();
inline float get_last_x_angle();
inline float get_last_y_angle();
inline float get_last_z_angle();
inline float get_last_gyro_x_angle();
inline float get_last_gyro_y_angle();
inline float get_last_gyro_z_angle();

//
// Hardware configuration
//

RF24 radio(A0, 10); // setting up the NRF24 radio: CE, CS. CE at pin A0, CSN at pin 10

// sets the role of this unit in hardware.  Connect to GND to be the 'pong' receiver
// Leave open to be the 'ping' transmitter
const int role_pin = 0;

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xAEAEAEAEA0LL, 0xAEAEAEAEA1LL };

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  The hardware itself specifies
// which node it is.
//
// This is done through the role_pin
//

// The various roles supported by this sketch
typedef enum { sender = 1, receiver } role_e;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "sender", "receiver"};

// The role of the current running sketch
role_e role;

//
// Payload
//

const int min_payload_size = 4;
const int max_payload_size = 32;
const int payload_size_increments_by = 2;
int next_payload_size = min_payload_size; // initial payload set to minimum

char receive_payload[max_payload_size+1]; // +1 to allow room for a terminating NULL char

void setup(void)
{
  delay(20); // Just to get a solid reading on the role pin

  // read the address pin, establish our role
  if ( role_pin==1 )
    role = sender;
  else
    role = receiver;

  //
  // Print preamble
  //
  pinMode(A1,OUTPUT);
  digitalWrite(A1, LOW); // Vcc for MPU6050
  pinMode(5,OUTPUT);
  digitalWrite(5, HIGH); // Vcc for MPU6050
  pinMode(7,OUTPUT);
  digitalWrite(7, HIGH); // Vcc for MPU6050
    
  Serial.begin(115200);
  printf_begin();
  printf("\n\r  \n\r");
  printf("ROLE: %s\n\r",role_friendly_name[role]);

  radio.begin();
  radio.enableDynamicPayloads();   // enable dynamic payloads
  radio.setRetries(4,8);   // optionally, increase the delay between retries & # of retries
  radio.setChannel(42);
   // The amplifier gain can be set to RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_MAX=0dBm.
  radio.setPALevel(RF24_PA_MAX); // transmitter gain value (see above)
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_16);
  
  Serial.print(F("PA-level: "));
  Serial.print(radio.getPALevel());
  Serial.print(F("\t CRC: "));
  Serial.print(radio.getCRCLength());
  Serial.print(F("\t datarate: "));
  Serial.println(radio.getDataRate());
  //
  // Open pipes to other nodes for communication
  //

  // This opens two pipes for these two nodes to communicate
  // back and forth.
  // Open 'our' pipe for writing
  // Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

  if ( role == sender )
  {
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1,pipes[1]);
  }
  else
  {
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(1,pipes[0]);
  }

  radio.startListening();

  radio.printDetails();
}

void loop(void)
{
  //
  // Ping out role.  Repeatedly send the current time
  //

  if (role == sender)
  {
    // The payload will always be the same, what will change is how much of it we send.
    static char send_payload[] = "LBCDEFGHIJKLMNOPQRSTUVWXYZ789012"; //32 byte max
    radio.stopListening(); // stop listening so we can talk.
    printf("Now sending length %i...",next_payload_size);
    radio.write( send_payload, next_payload_size ); // This will block until complete

    // Now, continue listening
    radio.startListening();

    // Wait here until we get a response, or timeout
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout )
      if (millis() - started_waiting_at > 128 ) // timeout set to 128ms
        timeout = true;

    // Describe the results
    if ( timeout )
    {
      printf("response timed out.\n\r");
    }
    else
    {
      // Grab the response, compare, and send to debugging spew
      uint8_t len = radio.getDynamicPayloadSize();
      radio.read( receive_payload, len );

      // Put a zero at the end for easy printing
      receive_payload[len] = 0;

      // Spew it
      printf("Got response size=%i value=%s\n\r",len,receive_payload);
    }
    
    // Update size for next time.
    next_payload_size += payload_size_increments_by;
    if ( next_payload_size > max_payload_size )
      next_payload_size = min_payload_size;

    // Try again 1s later
    delay(1000);
  }

  //
  // Pong back role.  Receive each packet, dump it out, and send it back
  //

  if ( role == receiver )
  {
    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      uint8_t len;
      bool done = false;
      while (!done)
      {
        // Fetch the payload, and see if this was the last one.
	len = radio.getDynamicPayloadSize();
	done = radio.read( receive_payload, len );

	// Put a zero at the end for easy printing
	receive_payload[len] = 0;
	// Spew it
	printf("Got payload size=%i value=%s\n\r",len,receive_payload);
      }

      // First, stop listening so we can talk
      radio.stopListening();

      // Send the final one back.
      radio.write( receive_payload, len );
      printf("Sent response.\n\r");

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }
  }
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
