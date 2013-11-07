
/* WIP DRAFT, TBD
 C voltage (1-24 byte) fixed point values
 D current (1-24 byte) fixed point values
 E battery voltage (2 byte)
 F error code + error value
 G 
 K draft for home automation packet
 L LED map
 T send out timestamp
 V software version, UID, wID, location
 B reply with the just received timestamp (T->B)
 */
 
void send_K(int to){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  byte axl=map(abs(ax),0,17000,0,255);
  byte ayl=map(abs(ay),0,17000,0,255);
  byte azl=map(abs(az),0,17000,0,255);
  byte gxl=map(2*abs(gx),0,32767,0,255);
  byte gyl=map(2*abs(gy),0,32767,0,255);
  byte gzl=map(2*abs(gz),0,32767,0,255);
  byte kmap[24]={
    000,000,000, // status LED at 0
    axl&0xFF,ayl&0xFF,azl&0xFF, // acc values
    axl&0xFF,ayl&0xFF,azl&0xFF,
    axl&0xFF,ayl&0xFF,azl&0xFF,
    axl&0xFF,ayl&0xFF,azl&0xFF,    
    gxl,gxl,gxl,
    gyl,gyl,gyl,
    gzl,gzl,gzl };

    unsigned long now = millis();
    bool ok = send_L(to, kmap);
    if (DEBUG) {
      p(" in %ld ms.\n", (millis()-now) );
      if (ok){}
      if (!ok) p("%010ld: send_K timout.\n", millis()); // An error occured..
    }
  }
 
void send_L1(int to, int _b = 0){
  if ( to != this_node) {      
    byte ledmap[24]={
      1,2,3,
      0,0,_b,
      0,0,_b,
      0,0,_b,
      0,0,_b,
      0,0,_b,
      0,0,_b,
      0,(((byte) millis()&0xFF)/5),_b        };

    unsigned long now = millis();
    bool ok = send_L(to, ledmap);
    if (DEBUG) {
      p(" in %ld ms.\n", (millis()-now) );
      if (ok){}
      if (!ok) p("%010ld: send_L timout.\n", millis()); // An error occured..
    }
    ledst();
  }
}

boolean send_T(uint16_t to) // Send out this nodes' time -> Timesync!
{
  if (DEBUG) p("%010ld: Sent 'T' to   %05o", millis(),to);
  RF24NetworkHeader header(to,'T');
  unsigned long time = micros();
  return network.write(header,&time,sizeof(time));
}

boolean send_L(uint16_t to, byte* ledmap) // Send out an LED map
{
  if (DEBUG) p("%010ld: Sent 'L' to   %05o", millis(),to);
  RF24NetworkHeader header(to,'L');
  return network.write(header,ledmap,24);
}

void ledupdate(byte* ledmap){
  for(uint8_t i=0; i<leds.numPixels(); i++) {
    uint32_t c = leds.Color(ledmap[i*3],ledmap[(i*3)+1],ledmap[(i*3)+2]);
    leds.setPixelColor(i, c);
  }
  leds.show();
}

void handle_K(RF24NetworkHeader& header)
{
  byte kmap[24];
  network.read(header,kmap,sizeof(kmap));
  if (DEBUG) p("%010ld: Recv 'K' from %05o\n", millis(), header.from_node);
  byte ledmap[24]={
    000,000,000, // status LED at 0
    kmap[0],kmap[1],kmap[2], // acc values
    kmap[3],kmap[3],kmap[3], // gyro x
    kmap[4],kmap[4],kmap[4], // gyro y...
    kmap[5],kmap[5],kmap[5],    
    000,000,00,
    000,000,00,
    000,000,0      };
  ledupdate(ledmap);

  if (DEBUG) {
    for(uint16_t i=0; i<sizeof(kmap); i++) { // print out the received packet via serial
      Serial.print(kmap[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void handle_L(RF24NetworkHeader& header)
{
  byte ledmap[24];
  network.read(header,ledmap,sizeof(ledmap));
  if (DEBUG) {
    p("%010ld: Recv 'L' from %05o\n", millis(), header.from_node);
  }
  ledupdate(ledmap);
  if (DEBUG) {
    for(uint16_t i=0; i<sizeof(ledmap); i++) { // print out the received packet via serial
      Serial.print(ledmap[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void handle_T(RF24NetworkHeader& header)
{
  unsigned long time;
  network.read(header,&time,sizeof(time));
  if (DEBUG) {
    p("%010ld: Recv 'T' from %05o:%010ld\n", millis(), header.from_node, time);
  }
  add_node(header.from_node);  
  if(header.from_node != this_node)
  {
    RF24NetworkHeader header2(header.from_node/*header.from_node*/,'B');
    unsigned long nowM = micros();
    if(network.write(header2,&time,sizeof(time)))
      if (DEBUG) {
        p("%010ld: Answ 'B' to   %05o in ", millis(),header.from_node);
        Serial.print(micros()-nowM-16);
        Serial.print(F(" us.\n"));
      }
  }
}

void handle_B(RF24NetworkHeader& header)
{
  p_recv++;
  unsigned long ref_time;
  network.read(header,&ref_time,sizeof(ref_time));
  if (DEBUG) {
    p("%010ld: Recv 'B' from %05o -> %ldus round trip\n", millis(), header.from_node, micros()-ref_time);
  }
}

