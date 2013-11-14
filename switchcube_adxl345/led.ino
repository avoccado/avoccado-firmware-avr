
void ledst(int sta){
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
}

PROGMEM prog_uchar red[]=
{
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0,
  255,0,0};
PROGMEM prog_uchar blue[]=
{
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255,
  0,0,255};
byte pat1[]={
  10,0,0,
  10,10,0,
  0,10,0,
  0,10,10,
  0,0,10,
  10,0,10,
  10,5,0,
  5,16,5};

void colorWipe(uint32_t c, uint8_t wait) { //this is blocking with the hardcoded delay...
  for(uint16_t i=0; i<leds.numPixels(); i++) {
    leds.setPixelColor(i, c);
    leds.show();
    delay(wait);
  }
}
