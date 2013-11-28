/* WIP DRAFT, TBD
 C voltage (1-24 bytes) fixed point values
 D current (1-24 bytes) fixed point values
 E battery voltage (2 bytes)
 F error code (1 byte) + error value (1 byte)
 G 
 H reserved for home automation packet
 L LED map
 T send out timestamp
 V software version, UID, wID, location
 B reply with the just received timestamp (T->B)
 */
byte br=50; // global brightness setting
byte _r,_g,_b,_c1,_c2,_c3=50;
const int PROGMEM breakpoint = 32;
void send_K(unsigned int to){
  byte _actions=0;
  unsigned int GMAX = 32000;
  mpucheck();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//  unsigned int _br= (unsigned int) angle_z * 728 / 256 // brightness determined by the z_rotation

if ( ay>(+15000) ) {
  Serial.println(F("RIGHT"));
  int _cd= abs(gy)-breakpoint; // threshhold for movement detection and against gyro drift
  _cd=(max(_cd,breakpoint))-breakpoint;
  _cd=map(_cd,0,GMAX,0,255);
  if (gy>0) {
    if (((int)_c1-_cd)<0) {
    _c1=0;
    vibr(2);
    _actions++;
  } else {_c1-=_cd;}
  }
  if (gy<=0) {
    if (((int)_c1+_cd)>255) {_c1=255; vibr(2);_actions++;} else {_c1+=_cd;}
  }
 }

if ( ay<(-15000) ) { // switchcube prototype 3 = top
  Serial.println(F("LEFT"));
   long _csVal = cs.capacitiveSensor(32);
   Serial.println(_csVal);
   _csVal=min(_csVal,2048);
   br=map(_csVal,10,2048,5,255);
   if (_csVal > 512) _actions++;
 }

if ( ax>(+15000) ) {
  Serial.println(F("BACK"));
  int _cd= abs(gx)-breakpoint; 
  _cd=(max(_cd,breakpoint))-breakpoint;
  _cd=map(_cd,0,GMAX,0,255);
  if (gx>0) {
    if (((int)_c3-_cd)<0) {_c3=0;vibr(2);_actions++;} else {_c3-=_cd;}
  }
  if (gx<=0) {
    if (((int)_c3+_cd)>255) {_c3=255;vibr(2);_actions++;} else {_c3+=_cd;}
  }
 }

if ( ax<(-15000) ) {
  Serial.println(F("FRONT"));
  int _cd= abs(gx)-breakpoint; 
  _cd=(max(_cd,breakpoint))-breakpoint;
  _cd=map(_cd,0,GMAX,0,255);
  if (gx>0) {
    if (((int)_c2+_cd)>255) {_c2=255;vibr(2);_actions++;} else {_c2+=_cd;}
  }
  if (gx<=0) {
    if (((int)_c2-_cd)<0) {_c2=0;vibr(2);_actions++;} else {_c2-=_cd;}
  }
 }

if ( az>15000 ) {
  Serial.println(F("TOP"));
  int _cd= abs(gz)-breakpoint; 
  _cd=(max(_cd,breakpoint))-breakpoint;
  _cd=map(_cd,0,GMAX,0,255);
  if (gz>0) {
    if (((int)br-_cd)<0) {br=0;vibr(2);_actions++;} else {br-=_cd;}
  }
  if (gz<=0) {
    if (((int)br+_cd)>255) {br=255;vibr(2);_actions++;} else {br+=_cd;}
  }
 }
 
 if ( az< (-15000) ) {
  Serial.println(F("BOT"));
  
  if (strobe) br=255;
  if (!strobe) br=0;
  _actions++;
 }
 Serial.println(F("ACT"));
 Serial.println(_actions,DEC);
 Serial.println(F("ACT"));
 if (_actions>0) active();
 
if ( (abs(angle_x)<10) & (abs(angle_y)<10) ) {
  }
  
//byte _br=max(0,angle_z);
  Serial.print(br);
//  byte _c1= (unsigned int) (angle_x+90) * 364 / 257; // color
//  byte _c2= (unsigned int) (angle_y+90) * 364 / 257; // color
  _r= (unsigned int) _c1*br/255;
  _g= (unsigned int) _c2*br/255;
  _b= (unsigned int) _c3*br/255;
  Serial.print(F("\t _r: "));
  Serial.print(_r);
  Serial.print(F("\t _g: "));  
  Serial.print(_g);
  Serial.print(F("\t _b: "));    
  Serial.println(_b);
/*  byte axl=map(abs(ax),0,17000,0,255);
  byte ayl=map(abs(ay),0,17000,0,255);
  byte azl=map(abs(az),0,17000,0,255);
  byte gxl=map(2*abs(gx),0,32767,0,255);
  byte gyl=map(2*abs(gy),0,32767,0,255);
  byte gzl=map(2*abs(gz),0,32767,0,255);
  */
/*  byte kmap[24]={
    000,000,000, // status LED at 0
    axl&0xFF,ayl&0xFF,azl&0xFF, // acc values
    axl&0xFF,ayl&0xFF,azl&0xFF,
    axl&0xFF,ayl&0xFF,azl&0xFF,
    axl&0xFF,ayl&0xFF,azl&0xFF,    
    gxl,gxl,gxl,
    gyl,gyl,gyl,
    gzl,gzl,gzl };
*/
byte kmap[24]={
    _r,_g,_b, // status LED at 0
    _r,_g,_b,
    _r,_g,_b,
    _r,_g,_b,
    _r,_g,_b,
    _r,_g,_b,
    _r,_g,_b,
    _r,_g,_b};
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
      0,(((byte) millis()&0xFF)/10),_b        };

    unsigned long now = millis();
    bool ok = send_L(to, ledmap);
    if (DEBUG) {
      p(" in %ld ms.\n", (millis()-now) );
      if (ok){}
      if (!ok) p("%010ld: send_L1 timout.\n", millis()); // An error occured..
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
  if (DEBUG) p("%010ld: 'L' to   %05o", millis(),to);
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
  if (DEBUG) p("%010ld: 'K' from %05o\n", millis(), header.from_node);
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
    p("%010ld: 'L' from %05o\n", millis(), header.from_node);
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
    p("%010ld: 'T' from %05o:%010ld\n", millis(), header.from_node, time);
  }
  add_node(header.from_node);  
  if(header.from_node != this_node)
  {
    RF24NetworkHeader header2(header.from_node/*header.from_node*/,'B');
    unsigned long nowM = micros();
    if(network.write(header2,&time,sizeof(time)))
      if (DEBUG) {
        p("%010ld: ->'B' to   %05o in ", millis(),header.from_node);
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
void processPacket(){
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
        network.read(header, 0, 0); // if none of the above packet types matched, read out and flush the buffer
        if (DEBUG) {
          Serial.print(F("undefined packet type: ")); // print the unrecognized packet type
          Serial.print(header.type);
          Serial.println();
        }
        break;
    };
    ledst(); // reset the status LED to the default pattern
}
