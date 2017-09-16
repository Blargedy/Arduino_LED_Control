
//#include <PinChangeInt.h>
//#include <PinChangeIntConfig.h>
#include <EnableInterrupt.h>

// Change this to be at least as long as your pixel string (too long will work fine, just be a little slower)

#define PIXELS 288  // Number of pixels in the string

#define PIXEL_PORT  PORTB  // Port of the pin the pixels are connected to
#define PIXEL_DDR   DDRB   // Port of the pin the pixels are connected to
#define PIXEL_BIT   4      // Bit of the pin the pixels are connected to
//this translates to GPIO 12

// These are the timing constraints taken mostly from the WS2812 datasheets 
// These are chosen to be conservative and avoid problems rather than for maximum throughput 

#define T1H  900    // Width of a 1 bit in ns
#define T1L  600    // Width of a 1 bit in ns

#define T0H  400    // Width of a 0 bit in ns
#define T0L  900    // Width of a 0 bit in ns

#define RES 6000    // Width of the low gap between bits to cause a frame to latch

// Here are some convience defines for using nanoseconds specs to generate actual CPU delays

#define NS_PER_SEC (1000000000L)          // Note that this has to be SIGNED since we want to be able to check for negative values of derivatives

#define CYCLES_PER_SEC (F_CPU)

#define NS_PER_CYCLE ( NS_PER_SEC / CYCLES_PER_SEC )

#define NS_TO_CYCLES(n) ( (n) / NS_PER_CYCLE )

#define DELAY_CYCLES(n) ( ((n)>0) ? __builtin_avr_delay_cycles( n ) :  __builtin_avr_delay_cycles( 0 ) )  // Make sure we never have a delay less than zero

#define UNO_S0 A0 	  //pin containing state bit 0 received from ESP8266
#define UNO_S1 A1	    //pin containing state bit 1 received from ESP8266
#define UNO_S2 A2	    //pin containing state bit 2 received from ESP8266
#define UNO_S3 A3 	  //pin containing state bit 3 received from ESP8266
#define ESP_INT A4 	  //pin for ESP8266 sent interrupt

//int rpi_S0 = 4;			//pin containing state bit 0 recieved from raspberry pi
//int rpi_S1 = 5;			//pin containing state bit 1 recieved from raspberry pi
//int rpi_S2 = 6;			//pin containing state bit 2 recieved from raspberry pi
//int rpi_S3 = 7;			//pin containing state bit 3 recieved from raspberry pi
//int rpi_int = 3;		//pin for raspberry pi sent interrupt

int nextState;

boolean first_time = true;

// state number  State Code    Description
//
//      12          0000        Quick sunrise on
//      13          0001        Quick sunrise off
//      2           0010        simulate sunrise
//		  3			      0011		    Open window rave parteh with strobez
//      4           0100        Slow color change
//      5           0101        Color Swipes
//      6           0110        Theatre Lights
//      7           0111        Tron style chasing circles
//      8           1000        Back and forth snake
//      9           1001        Aurora Borealis
//      10          1010        Mood lighting
//      11          1011        Rainbow Ring

void setup() {
  //start serial connection
  Serial.begin(115200);
    
  ledsetup();
  
  //set state input pins as inputs
  pinMode(UNO_S0, INPUT);
  pinMode(UNO_S1, INPUT);
  pinMode(UNO_S2, INPUT);
  pinMode(UNO_S3, INPUT); 

  //set interrupt pins as inputs
  pinMode(ESP_INT, INPUT);
  //pinMode(rpi_int, INPUT);
 
  //attach interrupt vectors to interrupt pins
 enableInterrupt(ESP_INT, arduino_state_check, RISING);  

  //set on-board LED pin as output, and turn off
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  //set first state as white light
  nextState = 12;  
}

// Set the specified pin up as digital out
void ledsetup() {  
  bitSet( PIXEL_DDR , PIXEL_BIT );  
}

void loop() {
  switch(nextState)
  {
	case 0:    //quick sunrise on 
	{             
       fadeIn(0);       
       if(nextState == 0)
       {
         first_time = false;
         nextState = 12;
       }         
       break;
    }      
    case 1:    //quick sunset off 
    {      
       fadeOut(0);
       
       if(nextState == 1)
       {
         first_time = false;
         nextState = 12;
       }       
       break; 
    }      
    case 2:   //simulate slow sunrise 
    {     
     fadeIn(100);     
     if(nextState == 2)
     {
       first_time = false;
       nextState = 12;
     }           
     break;
    }     
    case 5:    //various color swiping 
    {    
    //ADD TRIAD GEN USEAGE HERE TO MAKE COLORS RANDOM AND INTERESTING
      first_time = false;
      if(nextState != 5)
        break;
      colorWipe(255, 0, 0, 10); // Red
      if(nextState != 5)
        break;
      colorWipe(0, 255, 0, 10); // Green
      if(nextState != 5)
        break;
      colorWipe(0, 0, 255, 10); // Blue
      if(nextState != 5)
        break;
      colorWipe(255, 255, 0, 10);
      if(nextState != 5)
        break;
      colorWipe(0, 255, 255, 10);
      if(nextState != 5)
        break;
      colorWipe(255, 0, 255, 10);
      if(nextState != 5)
        break;
      strobez(255, 255, 255, 50, 30);
      if(nextState != 5)
        break;
      strobez(255, 0, 255, 50, 40);
      if(nextState != 5)
        break;
      strobez(255, 255, 255, 50, 50);
      if(nextState != 5)
        break;
      strobez(255, 0, 255, 50, 60);            
      break;
    }    
    case 6:  //theatre lights 
    {        
      if(nextState != 6)
        break;  
     
      //declare arrays to hold 3 rgb colors
      unsigned char rgb1[3], rgb2[3], rgb3[3];   
        
      //generate random seed
      //randomSeed(analogRead(0));
      
      //generate random hue value and use triad gen to generate harmonic triad      
      double theater_hue = random(0, 36001) / 100;
      Serial.println(theater_hue);
      TriadGen(theater_hue, rgb1, rgb2, rgb3);      
         
      // Send a theater pixel chase in the three generated colors
      theaterChase(rgb1[0], rgb1[1], rgb1[2], 20); // first color      
      if(nextState != 6)
        break;        
      theaterChase(rgb2[0], rgb2[1], rgb2[2], 20); // second color     
      if(nextState != 6)
        break;         
      theaterChase(rgb3[0], rgb3[1], rgb3[2], 20); // third color      
      break;
    }
      
     case 11:      //rainbow ring 
     {
       if(nextState != 11)
         break;
       rainbowCycle(1000 , 20 , 50 );       
       break; 
     }

	 case 12:  //Full brightness white light 
	 {
      if(first_time)
	  {        
        WhiteLight();
      }
      break;
     }
      
       
    /*
    // Some example procedures showing how to display to the pixels:
      colorWipe(255, 0, 0, 10); // Red
      colorWipe(0, 255, 0, 10); // Green
      colorWipe(0, 0, 255, 10); // Blue
      rainbowCycle(1000 , 20 , 50 );
      detonate( 255 , 255 , 255 , 1000);
      first_time = true;
    */    
  }   
}


void arduino_state_check(){
  delay(10);
  
  boolean pin0; //value of arduino_S0
  boolean pin1;	//value of arduino_S1
  boolean pin2;	//value of arduino_S2
  boolean pin3; //value of arduino_S3
  
  pin0 = digitalRead(UNO_S0);
  pin1 = digitalRead(UNO_S1);
  pin2 = digitalRead(UNO_S2);
  pin3 = digitalRead(UNO_S3);
  
  nextState = state_to_base10(pin3, pin2, pin1, pin0);  
}

int state_to_base10(int bit3, int bit2, int bit1, int bit0)
{
	return (bit3*2*2*2 + bit2*2*2 + bit1*2 + bit0);
}

void WhiteLight(){
  showColor(255,255,255);
  first_time = false;
}

void fadeOut(int wait)
{
  for( int fade=256; fade>0; fade-- ) {    
    showColor( (255 * fade) / 256 ,(255*fade) /256 , (255*fade)/256 );
    delay(wait);        
  }  
  showColor( 0 , 0 , 0 );
}

void fadeIn(int wait)
{
  for( int fade=0; fade<255; fade++ ) {    
    showColor( (255 * fade) / 255 ,(255*fade) /255 , (255*fade)/255 );
    delay(wait);        
  }  
  showColor( 255 , 255 , 255 );
}

void strobez(unsigned char r , unsigned char g , unsigned char b, int repeat, int pause)
{
    for(int i = 0; i <= repeat; i++)
    {
    showColor(r, g, b);
    showColor(0, 0, 0);
    delay(pause);
    }  
}

void raveParteh()
{
  return;
}//end raveparteh()

/// <summary>
/// Convert Hue Saturation Value to RGB
/// h is from 0-360
/// s,v values are 0-1
/// r,g,b values are 0-255
/// </summary>
void HsvToRgb(double h, double s, double v, unsigned char *r, unsigned char *g, unsigned char *b)
{
  int i;
  double f, p, q, t;

/*
  while(h < 0)
    h += 360;
  while(h >= 360)
    h -= 360;
    */
    
  if(v <= 0) //leds are off
  {
    *r = *g = *b = 0;
    return;
  }
    
  else if(s <= 0) //achromatic (black and white)
  {
    *r = *g = *b = v;
    return;
  }
    
  else
  {
    h /= 60.0;       //sector 0 to 5
    i = static_cast<int>(h);
    f = h - i;     //factorial remainder part of h
    p = v * (1 - s);
    q = v * (1 - s * f);
    t = v * (1 - s * (1 - f));
    
    switch(i)
    {      
      case 0:
        *r = v * 255;
        *g = t * 255;
        *b = p * 255;
        break;
      case 1:
       *r = q * 255;
       *g = v * 255;
       *b = p * 255;
       break;
       
      case 2:
       *r = p * 255;
       *g = v * 255;
       *b = t * 255;
       break;
       
      case 3:
       *r = p * 255;
       *g = q * 255;
       *b = v * 255;
       break;
       
      case 4:
       *r = t *255;
       *g = p * 255;
       *b = v * 255;
       break;
       
      default:		// case 5:
        *r = v * 255;
        *g = p * 255;
        *b = q * 255;
        break;
    }    
  }
}              //end HsvToRgb

// r,g,b values are from 0 to 255
// h = [0,360], s = [0,1], v = [0,1]
//		if s == 0, then h = -1 (undefined)
void RgbToHsv(unsigned char r, unsigned char g, unsigned char b, double *h, double *s, double *v)
{
  double Min, Max, delta;
  Min = MIN(r, g, b);
  Max = MAX(r, g, b);
  *v = Max;			// v
  delta = Max - Min;
  
  if(Max != 0)
    *s = delta / Max;		// s
    
  else {
    // r = g = b = 0		// s = 0, v is undefined
    *s = 0;
    *h = -1;
    return;
  }
  
  if(r == Max)
    *h = (g - b) / delta;		// between yellow & magenta
    
  else if(g == Max)
    *h = 2 + (b - r) / delta;	        // between cyan & yellow
    
  else {
    *h = 4 + (r - g) / delta;	        // between magenta & cyan
    *h *= 60;				// degrees
  }
  
  if(*h < 0)
    *h += 360;
}

//takes one hue value from a hsv value as input, and generates two hsv values such that they form
//a triad harmony with one another. These three values are then converted to rgb values.
//rgb values are returned via pointers to arrays
//REQUIRES: 
//  h must be a double
//  0 <= h <= 360
//  rgbi[1], rgbi[2], rbgi[3] must exist and be valid addresses, where i = 1, 2, 3
//PROMISES: 
//  three rgb values are generated that form a triad harmony with each other, using the passed
//  h value as a reference. Saturation and Value magnitudes of 1.0 are used by default. 
void TriadGen(double h1, unsigned char* rgb1, unsigned char* rgb2, unsigned char* rgb3)
{
  double h2, h3;
  
  h2 = h1 + 120;
  h3 = h1 + 240;
  
  while(h2 >= 360)
    h2 -= 360;
    
  while(h3 >= 360)
    h3 -= 360;
  
  HsvToRgb(h1, 1.00, 1.00, &rgb1[0], &rgb1[1], &rgb1[2]);
  HsvToRgb(h2, 1.00, 1.00, &rgb2[0], &rgb2[1], &rgb2[2]);
  HsvToRgb(h3, 1.00, 1.00, &rgb3[0], &rgb3[1], &rgb3[2]);
}

int MIN(unsigned char a, unsigned char b, unsigned char c)
{  
  if(a <= b && a <= c)
    return a;
    
  else if(b <= c)
    return b;
    
  else
    return c;
}

int MAX(unsigned char a, unsigned char b, unsigned char c)
{
  if(a >= b && a >= c)
    return a;
    
  else if(b >= c)
    return b;
    
  else
    return c;
}

// Fill the dots one after the other with a color
// rewrite to lift the compare out of the loop
void colorWipe(unsigned char r , unsigned char g, unsigned char b, unsigned  char wait ) {
  for(unsigned int i=0; i<PIXELS; i+= (PIXELS/60) ) {
    
    cli();
    unsigned int p=0;
    
    while (p++<=i) {
        sendPixel(r,g,b);
    } 
     
    while (p++<=PIXELS) {
        sendPixel(0,0,0);        
    }
    
    sei();
    show();
    delay(wait);
  }
}

// Theatre-style crawling lights.
// Changes spacing to be dynmaic based on string size

#define THEATER_SPACING (PIXELS/20)

void theaterChase( unsigned char r , unsigned char g, unsigned char b, unsigned char wait ) {
  first_time = false;
  
  for (int j=0; j< 3 ; j++) {  
    for (int q=0; q < THEATER_SPACING ; q++) {      
      unsigned int step=0;      
      cli();      
      for (int i=0; i < PIXELS ; i++) {        
        if (step==q) {          
          sendPixel( r , g , b );          
        } else {          
          sendPixel( 0 , 0 , 0 );          
        }        
        step++;        
        if (step==THEATER_SPACING) step =0;        
      }      
      sei();      
      show();
      delay(wait);      
    }    
  }  
}

// I rewrite this one from scrtach to use high resolution for the color wheel to look nicer on a *much* bigger string
                                                                            
void rainbowCycle(unsigned char frames , unsigned int frameAdvance, unsigned int pixelAdvance ) {  
  // Hue is a number between 0 and 3*256 than defines a mix of r->g->b where
  // hue of 0 = Full red
  // hue of 128 = 1/2 red and 1/2 green
  // hue of 256 = Full Green
  // hue of 384 = 1/2 green and 1/2 blue
  // ...
  
  first_time = false;
  unsigned int firstPixelHue = 0;     // Color for the first pixel in the string
  
  for(unsigned int j=0; j<frames; j++) {            
    unsigned int currentPixelHue = firstPixelHue;       
    cli();            
    for(unsigned int i=0; i< PIXELS; i++) {      
      if (currentPixelHue>=(3*256)) {                  // Normalize back down incase we incremented and overflowed
        currentPixelHue -= (3*256);
      }            
      unsigned char phase = currentPixelHue >> 8;
      unsigned char step = currentPixelHue & 0xff;                 
      switch (phase) {        
        case 0: 
          sendPixel( ~step , step ,  0 );
          break;
          
        case 1: 
          sendPixel( 0 , ~step , step );
          break;

        case 2: 
          sendPixel(  step ,0 , ~step );
          break;          
      }      
      currentPixelHue+=pixelAdvance;                   
    }    
    sei();    
    show();    
    firstPixelHue += frameAdvance;           
  }
}
  
// I added this one just to demonstrate how quickly you can flash the string.
// Flashes get faster and faster until *boom* and fade to black.

void detonate( unsigned char r , unsigned char g , unsigned char b , unsigned int startdelayms) {
  while (startdelayms) {
    
    showColor( r , g , b );      // Flash the color 
    showColor( 0 , 0 , 0 );
    
    delay( startdelayms );      
    
    startdelayms =  ( startdelayms * 4 ) / 5 ;           // delay between flashes is halved each time until zero    
  }
  
  // Then we fade to black....  
  for( int fade=256; fade>0; fade-- ) {    
    showColor( (r * fade) / 256 ,(g*fade) /256 , (b*fade)/256 );        
  }  
  showColor( 0 , 0 , 0 );   
}

// Actually send a bit to the string. We turn off optimizations to make sure the compile does
// not reorder things and make it so the delay happens in the wrong place.

void  sendBit(bool) __attribute__ ((optimize(0)));

void sendBit( bool bitVal ) {  
    if (  bitVal ) {      
      bitSet( PIXEL_PORT , PIXEL_BIT );          
      DELAY_CYCLES( NS_TO_CYCLES( T1H ) - 2 );       // 1-bit width less  overhead  for the actual bit setting
                                                     // Note that this delay could be longer and everything would still work
      bitClear( PIXEL_PORT , PIXEL_BIT );      
      DELAY_CYCLES( NS_TO_CYCLES( T1L ) - 10 );       // 1-bit gap less the overhead of the loop                                  
    } else {
      bitSet( PIXEL_PORT , PIXEL_BIT );      
      DELAY_CYCLES( NS_TO_CYCLES( T0H ) - 2 );      // 0-bit width less overhead 
                                                    // **************************************************************************
                                                    // This line is really the only tight goldilocks timing in the whole program!
                                                    // **************************************************************************
      bitClear( PIXEL_PORT , PIXEL_BIT );      
      DELAY_CYCLES( NS_TO_CYCLES( T0L ) - 10  );      // 0-bit gap less overhead of the loop      
    }    
    // Note that the inter-bit gap can be as long as you want as long as it doesn't exceed the 5us reset timeout (which is A long time)
    // Here I have been generous and not tried to squeeze the gap tight but instead erred on the side of lots of extra time.
    // This has thenice side effect of avoid glitches on very long strings becuase   
}
  
void sendByte( unsigned char byte ) {    
    for( unsigned char bit = 0 ; bit < 8 ; bit++ ) {      
      sendBit( bitRead( byte , 7 ) );                // Neopixel wants bit in highest-to-lowest order
                                                     // so send highest bit (bit #7 in an 8-bit byte since they start at 0)
      byte <<= 1;                                    // and then shift left so bit 6 moves into 7, 5 moves into 6, etc      
    }         
}

void sendPixel( unsigned char r, unsigned char g , unsigned char b )  {    
  sendByte(g);          // Neopixel wants colors in green then red then blue order
  sendByte(r);
  sendByte(b);  
}

// Just wait long enough without sending any bots to cause the pixels to latch and display the last sent frame
void show() {
    DELAY_CYCLES( NS_TO_CYCLES(RES) );               
}

// Display a single color on the whole string
void showColor( unsigned char r , unsigned char g , unsigned char b ) {  
  cli();  
  for( int p=0; p<PIXELS; p++ ) {
    sendPixel( r , g , b );
  }
  sei();
  show();  
}
