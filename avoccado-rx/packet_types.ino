/* WIP DRAFT, TBD
 C voltage (1-24 bytes) fixed point values
 D current (1-24 bytes) fixed point values
 E battery voltage (2 bytes)
 F error code (1 byte) + error value (1 byte)
 G
 H reserved for home automation packet
 K LED map based on 6-dimensional input
 L LED map
 T send out timestamp
 V software version, UID, wID, location
 B reply with the just received timestamp (T->B)
 */

char receive_payload[32 + 1]; // +1 to allow room for a terminating NULL char

void processPacket() {
  uint8_t len = 0;
  while (radio.available()) // this is blocking until the full packet came through
  {
    // Fetch the payload and see if this was the last one.
    uint8_t len = radio.getDynamicPayloadSize();
    radio.read( receive_payload, len ); // read the FIFO buffer from the NRF24 radio and fill our local global public buffer
    if (DEBUG) {Serial.print("R: ");} // print an R as long as we receive
  }
  receive_payload[len] = 0; // Put a zero at the end for easy printing, make it a string
  switch (receive_payload[0]) // check which packet type we received
  {
    case 'K':
      handle_K();
      break;
    case 'L':
      //handle_L();
      break;
    case 'T':
      handle_T();
      break;
    case 'B':
      handle_B();
      break;
    default:
      // if none of the above packet types matched, read out and flush the buffer
      if (DEBUG) {
//        Serial.print(F("packet type: ")); // print the unrecognized packet type
//        Serial.print(receive_payload[0],DEC);
//        Serial.println();
      }
      handle_K(); // WIP handling K even when no packet type determined
      break;
  };
}

boolean send_T(uint16_t to) // Send out this nodes' time -> Timesync!
{
  if (DEBUG) p("%010ld: Sent 'T' to   %05o", millis(), to);
  //RF24NetworkHeader header(to,'T');
  unsigned long time = micros();
  return radio.write(&time, sizeof(time));
}

boolean send_L(uint16_t to, byte* ledmap) // Send out an LED map
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
  byte kmap[33]={0};
  Serial.print("K ");
  for (uint8_t i = 0; i < 32; i++) {
  kmap[i]=receive_payload[i];
  }
  
  byte ledmap[24] = {
    000, 000, 000, // status LED at 0
    kmap[0], kmap[1], kmap[2], // acc values
    kmap[3], kmap[3], kmap[3], // gyro x
    kmap[4], kmap[4], kmap[4], // gyro y...
    kmap[5], kmap[5], kmap[5],
    000, 000, 00,
    000, 000, 00,
    000, 000, 0
  };
  
#ifdef USE_LAMP
//  for (uint8_t _i = 2; _i <= 9; _i++) {
//  digitalWrite(_i, !(receive_payload[1]>>(_i-2)& 1));
//  Serial.print("\t");
//  Serial.print(!((receive_payload[1]>>(_i-2) & 1)),BIN);
//  }
  byte load=kmap[3]/28;
  Serial.print("load: ");
  Serial.print(load,DEC);
  Serial.println();
  for (uint8_t _i = 0; _i <= 7; _i++) {
  uint8_t _j=_i+2;
  digitalWrite(_j, ((_i+1)/load));
  Serial.print((_i/load));
  Serial.print("\t");
  delay(30); // hardcoded delay to reduce power surges
  }  
  Serial.println();
#endif
delay(1); // hardcoded delay to reduce power surges

#ifdef USE_LEDS
  ledupdate(ledmap);
#endif

  if (DEBUG) {
    for (uint16_t i = 0; i < sizeof(receive_payload); i++) { // print out the received packet via serial
      Serial.print((char)receive_payload[i],DEC);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void handle_T()
{
  unsigned long time;
  //network.read(header,&time,sizeof(time));
  if (DEBUG) {
    //  p("%010ld: 'T' from %05o:%010ld\n", millis(), header.from_node, time);
  }
  //add_node(header.from_node);
  // if(header.from_node != this_node)
  {
    //  RF24NetworkHeader header2(header.from_node/*header.from_node*/,'B');
    unsigned long nowM = micros();
    //  if(network.write(header2,&time,sizeof(time)))
    if (DEBUG) {
      //     p("%010ld: ->'B' to   %05o in ", millis(),header.from_node);
      //     Serial.print(micros()-nowM-16);
      //     Serial.print(F(" us.\n"));
    }
  }
}

void handle_B()
{
  p_recv++;
  unsigned long ref_time;
  //network.read(header,&ref_time,sizeof(ref_time));
  if (DEBUG) {
    //  p("%010ld: Recv 'B' from %05o -> %ldus round trip\n", millis(), header.from_node, micros()-ref_time);
  }
}
