/*
 *   Teency LC
 *   Si5351 Clocks Module
 *   QRP Labs Arduio Shield
 *   QRP Labs Reciever Module
 *   DS3231 RTC
 *   USB HUB, USB sound card.  USB powered.
 *   
 *   From AD9850_WSPR_LC - changed to a Si5351 clocks module
 *   
 *   WSPR beacon.  DIGI transceiver.
 *  
 * 
 *   Setting RTC on Serial
 *      #Hhh where hh is in 24 hour time even when clock is in AM PM mode
 *      #Mmm  minutes
 *      #Sss  seconds
 *      ?R   display RTC information
 *      
 *      
 *  Trying a Si5351 version.  Think it will be better then the DDS  
 *  
 *  Notes:
 *     Pin 2 is wired with a FET, so 5 volt tolerant open drain output only with inverse logic.  Not used with this version.
 *     A2 is the DAC output, used for transmitter bias.
 *     A3 is the 5 volt buffered pin 17, used to power the DDS buffer amp. Probably not needed with this Si5351 version.
 *       On DDS version it keyed the transmitter on/off.
 *     Clocks swapped on arduino shield, CLK0 is the rx vfo and buffered CLK1 is the tx vfo.
 *     DS3231 is on Wire1.  Could be moved to Wire to free two I/O pins.
 *       An edit was made in WireKinetis.h to enable Wire1, could then be reversed to save memory.
 *     Negative 4k offset in HDSDR to recieve on freq.  Offset gets desired signal away from baseband noise.  
 *  
 */


#include <Wire.h>
#include <DS3231.h>

#define SI5351 0x60     // I2C address
#define CLOCK_FREQ 27003760L;            // 3800
//  starting addresses of phase lock loop registers
#define PLLA 26
#define PLLB 34

#include "clocks_30m.h"         // full si5351 init from clockbuilder desktop


DS3231 myRTC( Wire1 );

 //  Arduino shield pin assignments for AD9850 module

// #define DATA A4     // 18  jumpers to A4 and A5 side on chipkit uC32
// #define FQUD A5     // 19
// #define WCLK  3
 //#define RESET  2    // reset is hardwired to ground on shield 
 
 #define TX_ENABLE 17  // to buffered 17 to A3 pin instead of A3(17). Powers buffer amp.

 // wiring on arduino shield but unused for now
// #define BAND0  7     // relays, drive low to enable
 //#define BAND1 A0
// #define BAND2 10
// #define BAND3 11
// #define BAND4 12

 // Nokia display 
 #include <LCD5110_Basic_t.h>
 //   ( sclk mosi d/c rst cs )
 LCD5110 LCD( 8, 9, 10, 11, 12 );    // soft spi
 extern unsigned char SmallFont[];
 extern unsigned char MediumNumbers[];
 extern unsigned char BigNumbers[];
 
 
// #define BAND5 A3             // buffered pin 17, digital 5 volt for dds buffer supply, A3 no longer wired 
 #define RX_ENABLE_LOW A1       // enables rx

// #define GPS1pps  9   // I think there are jumpers on the module to unwire these pins but not mentioned in the documentation
// #define GPSRxD   0
// #define CLK0     5
// #define CLk2     1   // TX pin, is this correct, not sure a good choice for this I/0 input

  /* switch states */
#define IDLE_ 0
#define ARM  1
#define DARM 2
#define DONE 3
#define TAP  4
#define DTAP 5
#define LONGP 6

 #define SW   20        // on board switch, wired to reset then to pin 20.  LC does not have a reset function.
 uint8_t sstate[1];     // button switch state array.  One switch.

 #define ON 1
 #define OFF 0
// #define POWER_DOWN  0x4

 #define LED1  13
 //#define LED2  43

 #define WSPR 0
 #define DIGI 1
// #define USB 2
// #define LSB 3

 #define stage(c) Serial.write(c)

 uint32_t _P1;                  // base transmit si5351 solution from which tone offsets are calculated 
 uint32_t _P2;
 uint32_t _P3;


//const float DDS_1hz  =  34.359738368;      // dds load word per hz for 125 meg clock
//const float trim_ = 15.0 / 10000000.0;     // freq error as percent of 10 meg, 50?. 

uint32_t  freq = 10138700;          // SSB base frequency
uint32_t  wspr_offset = 1500;       // 1400 to 1600 valid value
// rx has 474 caps so it is a very narrow band sdr
uint32_t  IF_freq = 4000;           // HDSDR offset away from zero beat noise
                         

//uint32_t  base;          

// tones - 1.4648 * value

// KE1MU FN54 27   running 400+ mw
const uint32_t wspr_msg[] = {
      3, 1, 2, 0, 0, 0, 2, 0, 3, 0, 2, 0, 1, 3, 3, 2, 0, 0, 3, 2, 0, 3, 0, 3, 3, 3, 1, 2, 0, 2,
      0, 0, 2, 2, 3, 0, 0, 3, 0, 3, 2, 2, 0, 2, 0, 0, 3, 0, 3, 1, 2, 0, 3, 1, 0, 1, 2, 2, 2, 1,
      1, 0, 3, 2, 0, 0, 0, 1, 1, 0, 1, 2, 1, 0, 3, 2, 1, 0, 2, 3, 0, 0, 3, 0, 3, 1, 0, 2, 0, 1,
      3, 0, 3, 2, 1, 0, 2, 2, 1, 2, 0, 2, 0, 2, 3, 2, 2, 3, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 3, 3,
      2, 3, 2, 0, 0, 3, 1, 3, 2, 2, 0, 2, 2, 3, 0, 1, 2, 2, 3, 1, 2, 0, 0, 0, 0, 2, 2, 1, 1, 2,
      3, 0, 1, 1, 0, 2, 2, 3, 3, 0, 0, 2
};
       


// 8 bands, 10 slots giving a 20 minute frame
#define NUM_BANDS 10

struct BANDS {
   uint32_t freq;
   uint16_t r_div;
   uint16_t t_div;
   uint8_t  group;
   // float tx_pwr;      // tx bias voltage
};

struct BANDS  bands[NUM_BANDS] = {
  {  474200,254, 254, 5 },               // won't tx, pll out of range, maybe rx with pll out of range. Needs R dividers.
  { 1836600, 90, 254, 4 },               // maybe tx, pll out of range
  { 3568600, 48, 194, 0 },
  {7038600, 26 , 102, 1 },
  {10138700, 18, 72, 1 },
  {14095600, 14 , 50, 2 },
  {18104600, 10 , 40, 2 },
  {21094600, 10 , 34, 3 },
  {24924600, 8 , 28, 3 },
  {28124600, 8 , 26, 3 }
};
// wspr transmit slots in 20 minute frame.  1 == tx,  0 == rx
const uint8_t slots[10] =
    { 0,      0,        0,        0,        1,       0,         0,        1,       0,       1
};

int group_enabled = 0;    // enable or disable the tx groups feature
int group = 1;            // bands that share a filter
int band = 4;             // 3 == 40 meters.   4 == 30 meters

int slot;
int mode = WSPR;  // !!! should maybe start in a mode that doesn't transmit by itsself
int wspr_sec = -1;   // for correcting time in wspr frame
int frame_count;     // for when to check frame timing.

volatile uint32_t tone_;              // FT8 tone detection variables, tone_ is in 25ns ticks
volatile uint8_t tone_available;
volatile uint8_t digi_vox;


IntervalTimer myTimer;
//IntervalTimer myTimer2;
volatile unsigned int wspr_tx_enable;     // turn wspr on or off

elapsedMillis wspr_tick;

uint8_t transmitting;
uint8_t tock;
uint8_t tx_msg;


void setup() {
// pinMode(DATA,OUTPUT);
// pinMode( FQUD, OUTPUT); 
// pinMode( WCLK, OUTPUT);
 //pinMode( RESET, OUTPUT);
// digitalWrite( DATA, LOW );
// digitalWrite( FQUD, LOW );
// digitalWrite( WCLK, LOW );
 //digitalWrite( RESET, LOW );
 pinMode( RX_ENABLE_LOW, OUTPUT );
 digitalWrite( RX_ENABLE_LOW, HIGH );
 pinMode( TX_ENABLE , OUTPUT );
 digitalWrite( TX_ENABLE, LOW );
 pinMode( SW, INPUT_PULLUP );
 analogWriteResolution( 12 );
 analogWrite(A12, 0 );            // tx bias

 pinMode( LED1, OUTPUT );
// pinMode( LED2, OUTPUT );

   Serial.begin( 1200 );       // usb, baud rate doesn't matter, Argo V is 1200 baud.
   analogReadResolution( 12 );
   
   delay( 5 );
   i2init();
   delay( 5 );
   si5351_init();

//   dds_init();

    freq = bands[band].freq;
    group = bands[band].group;
   // transmit( OFF );
    cat_qsy( freq );
    si_load_divider( bands[band].r_div, 0 , 0 );
    si_load_divider( bands[band].t_div, 1 , 1 );

   if( mode == WSPR ) myTimer.begin( wspr_core, 682687 );            // 1/1.4648
   wspr_tick = 0;

   Wire1.begin();     // had to edit WireKinetis.h to enable wire1
   // switch to wire now, rtc and si5351 both on I2C

   i2cd( SI5351, 3, 0b11111110 );      // enable vfo
   uint32_t t = calc_rx_freq_val( );   // load rx vfo with offset
   si_pll_x(PLLB, t, bands[band].r_div );

   // load tx freq to get the P1 P2 P3 saved solution
   si_pll_x( PLLA, freq, bands[band].t_div );     // wspr_offset + wspr_msg[] for sending wspr

   LCD.InitLCD();
   LCD.setFont(SmallFont);
   LCD.print("HELLO RADIO",CENTER,0);
   delay(5000);
   LCD.clrScr();


   display_freq();
   tock = 1;

      // set initial WSPR clock
   frame_time_check();
   wspr_tick = 0;
   
}

void frame_time_check(){

   noInterrupts();
   int m = myRTC.getMinute();
   int s = myRTC.getSecond();

   int tm = ( m & 1 ) ? 60 : 0;
   tm += s;
   wspr_sec = tm;
   interrupts();

   //Serial.print( "RTC : ");                   // interfere with CAT if left enabled?
   //Serial.print( tm );   Serial.write(' ');
 
}

void time_update(){
bool h12Flag, pmFlag;
bool century = false;
char months[] = "JANFEBMARAPRMAYJUNJULAUGSEPOCTNOVDEC";
char mth[4];

  int h = myRTC.getHour(h12Flag, pmFlag);
  int m = myRTC.getMinute();
  LCD.setFont(MediumNumbers);
  LCD.printNumI( h, 18, 0, 2, '/' );
  LCD.printNumI( m, 44, 0, 2, '0' );
  LCD.setFont(SmallFont);
  if( pmFlag ) LCD.print( (char*)"PM", RIGHT, 0 );
  else LCD.print( (char*)"AM", RIGHT, 0 );

  //int y = myRTC.getYear();
  int d = myRTC.getDate();
  m = myRTC.getMonth(century) - 1;
  strncpy(mth,&months[m*3],3);
  mth[3] = 0;
  LCD.print( mth, LEFT, 0 );
 // LCD.print((char *)"'", LEFT, 8 );
  LCD.printNumI( d,3,8,2,' '); 
  
}

void display_freq(){
int f;

   f = freq / 1000;
   LCD.setFont( BigNumbers );
   LCD.printNumI( f, 0, 3*8, 5, '0' );
   LCD.setFont( MediumNumbers);
  // LCD.setFont( SmallFont );
   f = freq % 1000;
   f = f / 100;  
   LCD.printNumI( f, RIGHT, 3*8 );
   LCD.setFont( SmallFont );
   if( freq < 10000000 ){
      LCD.clrRow( 3, 0, 13 );
      LCD.clrRow( 4, 0, 13 );
      LCD.clrRow( 5, 0, 13 );
   }
   if( mode == DIGI ) LCD.print((char*)"DIGI",RIGHT,2*8);
   else LCD.print((char*)"WSPR",RIGHT,2*8);
}

void tx_msg_update(){

   if( transmitting ) LCD.print( (char*) " TX", CENTER, 2 * 8 );
   else if( group_enabled ) LCD.print((char*) "grp", CENTER, 2 * 8 );
   else LCD.print( (char*) "   ", CENTER, 2 * 8 );
  
}

void loop() {
uint32_t tone2;
static uint32_t tm;


    radio_control();     // CAT
  
    if( wspr_tick >= 100 ){
       wspr_tick -= 100;
       wspr_frame();               // keep time even if not wspr mode
    }
    if( frame_count >= 31 ){
        frame_time_check();        // adjust wspr counter to match RTC
        frame_count = 0;
    }

   if( transmitting && tone_available ){         // digi mode transmit
      noInterrupts();
      tone2 = tone_;
      tone_available = 0;  
      interrupts();
      ft8_tx( tone2 );
   }


// 1 ms rate code
   if( tm != millis() ){
      tm = millis(); 
   
      if( mode == DIGI ){                   // vox check
          if( digi_vox ){
              if( --digi_vox == 0 ){
                 transmit(OFF);
              }
              else if( transmitting == 0 ){
                 transmit(ON);
              }
          }
          else tone_available = 0;
      }

      if( tock ) time_update(), tock = 0;                                 // clock display to minutes
      if( transmitting && tx_msg == 0 ) tx_msg_update(), tx_msg = 1;      // display TX when transmitting
      if( transmitting == 0 && tx_msg == 1 ) tx_msg_update(), tx_msg = 0;

      uint8_t t = switches();
      if( t > DONE ){
         button_( t );
         sstate[0] = DONE;
      }
      
   }

}

      /* run the switch state machine, generic code for multiple switches even though have only one here */
int8_t switches(){
static uint8_t press_, nopress;
static uint32_t tm;
int  i,j;
int8_t sw;
int8_t s;

   if( tm == millis() ) return 0;      // run once per millisecond
   tm = millis();
   
   sw = ( digitalRead( SW ) == LOW ) ? 1 : 0;                 
   
   if( sw ) ++press_, nopress = 0;       /* only acting on one switch at a time */
   else ++nopress, press_ = 0;           /* so these simple vars work for all of them */

   /* run the state machine for all switches in a loop */
   for( i = 0, j = 1; i < 1; ++i ){
      s = sstate[i];
      switch(s){
         case DONE:  if( nopress >= 100 ) s = IDLE_;  break;
         case IDLE_:  if( ( j & sw ) && press_ >= 30 ) s = ARM;  break; /* pressed */
         case ARM:
            if( nopress >= 30 ) s = DARM;                      /* it will be a tap or double tap */
            if( press_ >= 240 ) s = LONGP;                     // long press
         break;
         case DARM:
            if( nopress >= 240 )  s = TAP;
            if( press_ >= 30 )    s = DTAP;
         break;
      }
      sstate[i] = s; 
      j <<= 1;
   }
   
   return sstate[0];      // only one switch implemented so can return its value
}


void button_( uint8_t function ){

    if( transmitting ) return;    // ignore switch during tx or !!! adjust power out
    switch( function ){
       case TAP:          // change band
          if( ++band == NUM_BANDS ) band = 0;
          cat_qsy( bands[band].freq );
          group = bands[band].group;
          slot = 0;                     // avoid tx until settings are finished
       break;
       case DTAP:         // change mode
          if( mode == DIGI ) mode = WSPR, group_enabled = 0;
          else if( mode == WSPR && group_enabled == 0 ) group_enabled = 1;
          else if( mode == WSPR && group_enabled == 1 ) mode = DIGI, group_enabled = 0;
       break;
       case LONGP:        // save default band in eeprom
       break;
    }
    display_freq();
    tx_msg_update();
}

// called each 10th of a second
void wspr_frame(){
static int ticks, sec;
//int t;
//int i;
static int dir = 1;


   if( wspr_sec >= 0 && wspr_sec < 120 ){     // clock_setting from RTC
      // Serial.println( sec );
      sec = wspr_sec;
      wspr_sec = -1;
   }
   //  mode change
   if( wspr_tx_enable && mode != WSPR ){
      noInterrupts();
      while( wspr_tx_enable == 1 ) wspr_core();     // finish tx quickly
      interrupts();
   }

   if( ++ticks == 10 ){
      ticks = 0;

      if( ++sec == 120 ) sec = 0;
      if( sec == 2 || sec == 62 ) tock = 1;
      if( mode != WSPR ) return;

      if( sec == 119 && group_enabled && random(100) > 50 ){   // auto change frequency to next band in group
         if( band == 0 ) dir = 1;
         if( band == NUM_BANDS -1 ) dir = -1;
         if( bands[band].group == bands[band + dir].group )  cat_qsy( bands[band + dir].freq );
         else dir = (dir == 1 ) ? -1 : 1;
      }
      
      if( sec == 0 && group == bands[band].group ){   // this allows tx only on bands inside of set group 
        if( ++slot == 10 ) slot = 0;                  // group changes only using switch on radio to change bands
        if( slots[slot] == 1  ) wspr_tx_enable = 1;   // rx only bands when using wsjt-x to change bands to outside of group
      }
      if( sec == 30 ) ++frame_count;   // time check at midpoint of minute
   }
}


/*
void dds_init(){

  // the dds reset pin is hardwired to ground on the shield
  // D0 and D1 are hardired to vcc, D2 is hardwired to ground on the shield

  // digitalWrite( RESET, HIGH );
  // delay(1);
  // digitalWrite( RESET, LOW );
  
   delay(1);

   digitalWrite( WCLK, HIGH );     // writes the config in parallel mode, changing to serial mode
   digitalWrite( WCLK, LOW );
   digitalWrite( FQUD, HIGH );
   asm( "nop" ); 
   digitalWrite( FQUD, LOW );
   delay( 1 );
  
}
*/

/*
// LATESET LATECLR LATEINV registers direct writes for speed.  Chipkit info, keep or discard?  Teensy uses digitalWriteFast.
// DATA is A4  RB12
// WCLK is  3  RD0
// FQUD is A5  RB14
void load_dds( uint32_t val, uint8_t phase ){
int i;

   for( i = 0; i < 32; ++i ){
      digitalWriteFast( DATA, val & 1 );
      //if( val & 1 ) LATBSET = 1 << 12;
      //else LATBCLR = 1 << 12;
      
      val >>= 1;
      digitalWriteFast( WCLK, HIGH );
      //LATDSET = 1;
      digitalWriteFast( WCLK, LOW );
      //LATDCLR = 1;
   }

   //digitalWrite( DATA, LOW );
   //LATBCLR = 1 << 12;
   for( i = 0; i < 8; ++i ){

     // if( phase & 1 ) LATBSET = 1 << 12;
     // else LATBCLR = 1 << 12;
     digitalWriteFast( DATA, phase & 1 );
     phase >>= 1;
      digitalWriteFast( WCLK, HIGH );
      //LATDSET = 1;
      digitalWriteFast( WCLK, LOW );
      //LATDCLR = 1;
   }

   digitalWriteFast( FQUD, HIGH );
   //LATBSET = 1 << 14;
   asm( "nop" ); 
   digitalWriteFast( FQUD, LOW );
   //LATBCLR = 1 << 14;
}

*/

/*   WSPR  chipkit core timer function */
//#define WSPRTICK 27307482     // 1 bit time for 1.4648 baud.  (40M ticks in 1 second) CORE_TICK_RATE is counts in 1ms
// delay is 1/1.4648 seconds
// #define WSPRTICK 27306667        // or should it be 1.46484375 baud.  120000/8192

void  wspr_core(  ){
static int count;

  if( wspr_tx_enable == 0 )   return;   //timer + WSPRTICK;   // nothing to do

  if( count == 162 ) {   // stop the transmitter
     transmit(OFF);
     wspr_tx_enable = 0;
     count = 0;
     return;    // timer + WSPRTICK;
  }
    
  if( count == 0 ) transmit(ON);
  si_tone_offset( (float)wspr_offset + wspr_msg[count] * 1.4648 );
  
  ++count;
   
  return;    // timer + WSPRTICK;    
}

void transmit( int enable ){
static float pwr[] = { 2.4, 2.5, 2.6, 2.7 };
static int i;  

   if( enable == OFF ){        // set freq to base
       //load_dds( base, POWER_DOWN);   power down results in drift when powered up again
       //pinMode( TX_INHIBIT, OUTPUT );
       digitalWrite( TX_ENABLE, LOW );
       analogWrite(A12, 0 );
       delay( 1 );
     //  load_dds( calc_rx_freq_val( ), 0 );       // rx at SDR IF freq
       si_pll_x(PLLB, calc_rx_freq_val(), bands[band].r_div );
       i2cd( SI5351, 3, 0b11111110 );      // enable vfo 
       digitalWrite( LED1, LOW );
      // digitalWrite( LED2, LOW );
       digitalWrite( RX_ENABLE_LOW, LOW );
       transmitting = 0;
   }

   if( enable == ON ){
       if( band == 0 ) return;                      // MF transmit not possible with current divider setup
       digitalWrite( RX_ENABLE_LOW, HIGH );
       delay( 1 );
       si_pll_x(PLLA, freq, bands[band].t_div );    // starts tx at zero beat freq
       i2cd( SI5351, 3, 0b11111101 );               // enable tx vfo 
       digitalWrite( LED1, HIGH );
      // pinMode( TX_INHIBIT, INPUT );
      digitalWrite( TX_ENABLE, HIGH );
      analogWrite(A12, pwr[i] * 1240.0 );
      transmitting = 1;
      ++i;                                         // more power
      i &= 3;
   }
}

// don't really need this function anymore unless need 1/3 rate for 10 meters ( 120 mhz rx clock )
uint32_t calc_rx_freq_val( ){
uint32_t fq;

   fq = freq + IF_freq;   // tune 4k high
   return 4 * fq;

}



/*****************************************************************************************/
// TenTec Argonaut V CAT emulation

//int un_stage(){    /* send a char on serial */
//char c;

//   if( stg_in == stg_out ) return 0;
//   c = stg_buf[stg_out++];
//   stg_out &= ( STQUESIZE - 1);
//   Serial.write(c);
//   return 1;
//}

#define CMDLEN 20
char command[CMDLEN];
uint8_t vfo = 'A';

void radio_control() {
static int expect_len = 0;
static int len = 0;
static char cmd;

char c;
int done_;

    if (Serial.available() == 0) return;
    
    done_ = 0;
    while( Serial.available() ){
       c = Serial.read();
       command[len] = c;
       if(++len >= CMDLEN ) len= 0;  /* something wrong */
       if( len == 1 ) cmd = c;       /* first char */
       /* sync ok ? */
       if( cmd == '?' || cmd == '*' || cmd == '#' );  /* ok */
       else{
          len= 0;
          return;
       }
       if( len == 2  && cmd == '*' ) expect_len = lookup_len(c);    /* for binary data on the link */       
       if( (expect_len == 0 &&  c == '\r') || (len == expect_len) ){
         done_ = 1;
         break;   
       }
    }
    
    if( done_ == 0 ) return;  /* command not complete yet */
        
    if( cmd == '?' ){
      get_cmd();
      group_enabled = 0;                    // in computer control of band switching
     // operate_mode = CAT_MODE;            // switch modes on query cat command
     // if( wwvb_quiet < 2 ) ++wwvb_quiet;  // only one CAT command enables wwvb logging, 2nd or more turns it off
     // mode_display();
    }
    if( cmd == '*' )  set_cmd();
    if( cmd == '#' ){
        pnd_cmd(); 
       // if( wwvb_quiet < 2 ) ++wwvb_quiet;  // allow FRAME mode and the serial logging at the same time
    }

 /* prepare for next command */
   len = expect_len= 0;
   stage('G');       /* they are all good commands */
   stage('\r');

}

int lookup_len(char cmd2){     /* just need the length of the command */
int len;

   
   switch(cmd2){     /* get length of argument */
    case 'X': len = 0; break;
    case 'A':
    case 'B': len = 4; break;
    case 'E':
    case 'P':
    case 'M': len = 2; break;
    default:  len = 1; break ;
   }
   
   return len+3;     /* add in *A and cr on the end */
}

void set_cmd(){
char cmd2;
unsigned long val4;

   cmd2 = command[1];
   switch(cmd2){
    case 'X':   stage_str("RADIO START"); stage('\r'); break; 
    case 'O':   /* split */ 
    break;
    case 'A':   // set frequency
    case 'B':
       val4 = get_long();
       cat_qsy(val4);  
    break;
    case 'E':
       if( command[2] == 'V' ) vfo = command[3];
    break;
    case 'W':    /* bandwidth */
    break;
    case 'K':    /* keying speed */
    break;
    case 'T':    /* added tuning rate as a command */
    break;
    case 'M':
      // int i = command[2] - '0';          // FM will map to DIGI
      // mode_change(i);
      // status_display();
    break;       
   }  /* end switch */   
}

void cat_qsy( unsigned long val ){
int i;
int current_band;

  if( transmitting ) return;
  current_band = band;
  
  // see if can find freq in our table
  for( i = 0; i < NUM_BANDS; ++i ){
       if( val == bands[i].freq ){
        band = i, mode = WSPR;
        break;
       }
  }
  if( i == NUM_BANDS ) mode = DIGI;     // not qsy to a WSPR freq

  freq = val;
//  base = (float)freq * DDS_1hz;
//  base += (float)freq * trim_ * DDS_1hz;


  if( mode == DIGI ){      // what band are we in
      if( freq < 1000000 ) band = 0;
      else if( freq < 3000000 ) band = 1;
      else if( freq < 5000000 ) band = 2;
      else if( freq < 9000000 ) band = 3;
      else if( freq < 12000000 ) band = 4;
      else if( freq < 15000000 ) band = 5;
      else if( freq < 19000000 ) band = 6;
      else if( freq < 22000000 ) band = 7;
      else if( freq < 26000000 ) band = 8;
      else band = 9;
  }

  // group = bands[band].group;  ? keep tx group control with the unit
  transmit( OFF );
  if( band != current_band ){     // load dividers
       //si_load_divider( int val, int clk , int rst)
       si_load_divider( bands[band].r_div, 0 , 0 );
       si_load_divider( bands[band].t_div, 1 , 1 );
  }
  display_freq();
}



void get_cmd(){
char cmd2;
long arg;
int len; // i;

   cmd2 = command[1];   
   stage(cmd2);
   switch(cmd2){
    case 'A':     // get frequency
    case 'B': 
      arg = freq;
      stage_long(arg);
    break;
    case 'V':   /* version */
      stage_str("ER 1010-516");
    break;
    case 'W':          /* receive bandwidth */
       stage(30);
    break;
    case 'M':          /* mode. 11 is USB USB  ( 3 is CW ) vfo A, vfo B */
     // i = mode;
     // if( i > 4 ) i &= 3;                          // report UDSB, LDSB as USB, LSB.  DIGI reports as FM
     // i = i + '0';
      stage(3); stage(3);
    break;
    case 'O':          /* split */   
       stage(0);
    break;
    case 'P':         /*  passband slider */
       stage_int( 3000 );
    break;
    case 'T':         /* added tuning rate command */
    break;   
    case 'E':         /* vfo mode */
      stage('V');
      stage(vfo);
    break;
    case 'S':         /* signal strength */
       stage(7);
       stage(0);
    break;
    case 'C':      // transmitting status 
       stage(0);
       if( transmitting ) stage(1);
       else stage(0);
    break;
    case 'K':   /* wpm on noise blanker slider */
       stage( 15 - 10 );
    break;
    case 'R':
       RTC_info();
    break;   
    default:           /* send zeros for unimplemented commands */
       len= lookup_len(cmd2) - 3;
       while( len-- ) stage(0);  
    break;    
   }
  
   stage('\r');  
}


void stage_str( String st ){
unsigned int i;
char c;

  for( i = 0; i < st.length(); ++i ){
     c = st.charAt( i );
     stage(c);
  }    
}

void stage_long( long val ){
unsigned char c;
   
   c = val >> 24;
   stage(c);
   c = val >> 16;
   stage(c);
   c = val >> 8;
   stage(c);
   c = val;
   stage(c);
}


unsigned long get_long(){
union{
  unsigned long v;
  unsigned char ch[4];
}val;
int i;

  for( i = 0; i < 4; ++i) val.ch[i] = command[5-i]; // or i+2 for other endian
  return val.v;
}

void stage_int( int val ){
unsigned char c;
   c = val >> 8;
   stage(c);
   c = val;
   stage(c);
}

void stage_num( int val ){   /* send number in ascii */
char buf[35];
char c;
int i;

   itoa( val, buf, 10 );
   i= 0;
   while( (c = buf[i++]) ) stage(c);  
}

void pnd_cmd(){
char cmd2;
int v;

   command[4] = 0;
   cmd2 = command[1];      // !!! using vox or CAT for tx on/off
   switch(cmd2){
    // case '0':  rx();  break;    // enter rx mode
    // case '1':  tx();  break;    // TX
    // add set hours, set min, set seconds for RTC  #H04
    case 'H':
       v = atoi( &command[2] );
       myRTC.setHour(v);
       RTC_info();
    break;
    case 'M':
       v = atoi( &command[2] );
       myRTC.setMinute(v);
       RTC_info();
    break;
    case 'S':
       v = atoi( &command[2] );
       myRTC.setSecond(v);
       RTC_info();
    break;
    
   }

}

/********************* end Argo V CAT ******************************/


void RTC_info(){
bool century = false;
bool h12Flag;
bool pmFlag;

  // Start with the year
  Serial.print("2");
  if (century) {      // Won't need this for 89 years.
    Serial.print("1");
  } else {
    Serial.print("0");
  }
  Serial.print(myRTC.getYear(), DEC);
  Serial.print(' ');
  
  // then the month
  Serial.print(myRTC.getMonth(century), DEC);
  Serial.print(" ");

  
  // then the date
  Serial.print(myRTC.getDate(), DEC);
  Serial.print(" ");
  
  // and the day of the week
  Serial.print(myRTC.getDoW(), DEC);
  Serial.print(" ");
  
  // Finally the hour, minute, and second
  Serial.print(myRTC.getHour(h12Flag, pmFlag), DEC);
  Serial.print(" ");
  Serial.print(myRTC.getMinute(), DEC);
  Serial.print(" ");
  Serial.print(myRTC.getSecond(), DEC);
 
  // Add AM/PM indicator
  if (h12Flag) {
    if (pmFlag) {
      Serial.print(" PM ");
    } else {
      Serial.print(" AM ");
    }
  } else {
    Serial.print(" 24h ");
  }
 
  // Display the temperature
  Serial.print("T=");
  Serial.print(myRTC.getTemperature(), 2);
  

}


//  tone detection based upon counting 25 ns ticks since last zero cross
//  20.8333333333333 ns with 48meg clock
//  FT8 etc modes tx tone detection, DDS updated from loop
void digi_core( ){
int data;
static int last;
static uint32_t start_tm;                               // last timer value
static uint32_t total_tm;
uint32_t timer;
uint32_t fraction;                                      // counts past zero cross
uint32_t tm;
int spread;

  timer = SYST_CVR;
   
  data = analogRead( A0 ) - 2048;
 // tm = timer - start_tm;
  tm = start_tm - timer;        // counts down
  start_tm = timer;
  total_tm += tm;

  if( data > 0 && last <= 0 ){                          // zero cross detected
      spread = data-last;                               // last is always negative
      fraction =  ( data * tm ) / spread;               // ticks past zero
      tone_ = total_tm - fraction;                      // sub ticks past zero cross
      total_tm = fraction;                              // add fraction past zero as start amount for next cycle
      tone_available = 1;    
  }
  last = data;

  if( abs(data) > 310 ) digi_vox = 10;                   // 10ms vox hang time
  if( digi_vox == 0 ){
      total_tm = 0;
      return; // timer + CORE_TICK_RATE/3;                 // vox check only, 3k sample rate
  }
   
  return; //timer + CORE_TICK_RATE/20;                     // sample rate when transmitting
  
}



uint32_t median( uint32_t val ){
static uint32_t vals[3];
static uint8_t in;
uint8_t j,i,k;                               // low, median, high

   vals[in] = val;
   ++in;
   if( in > 2 ) in = 0;

   j = 0, i = 1, k = 2;                     // pretend they are in the correct order
   if( vals[j] > vals[k] ) k = 0, j = 2;    // swap guess high and low
   if( vals[i] < vals[j] ) i = j;           // is lower than the low guess, pick that one instead
   if( vals[i] > vals[k] ) i = k;           // is higher than the high guess

   return vals[i];
}

// ft8_tx version using 25ns ticks.
void ft8_tx( uint32_t val ){
long dds_val;
static uint32_t tm;
static uint32_t tval;
static int count;
float val2;

  if( val < 15600 || val > 240000 ) return;   // 25ns ticks for 3000 to 200 hz  

  val = median( val );                        // remove glitches in the data stream
  tval += val;
  ++count;
  
  // 3ms updates, 333 baud.  10ms updates, 100 baud.  20ms is 50 updates a second
  if( millis() - tm < 5 ) return;             // 5ms is 1/32 baud overlap of 6 baud, 10ms 1/16 fuzzy area
  tm = millis();

  val2 = (float)tval / (float)count;          // average value over N ms
  if( val2 == 0 ) return;
  //val2 = 40000000.0f / val2;                  // convert ticks to audio tone, chipkit version
  val2 =  48000000.0f / val2;                   // 20.8333333333333f ns per count
  count = 0;  tval = 0;
  
 // dds_val = (long)(( tx_vfo + val2 )  * (268.435456e6 / Reference ));  
 // wspr_to_freq( dds_val );

  // debug on arduino plotter
    Serial.println(val2);
}

// load the clock builder data
void si5351_init(){
uint8_t reg, data;
int i;

     delay(20);
     for( i = 0; i < 513; ++i ){
        reg = pgm_read_byte( &si5351_reg[i++] );
        data = pgm_read_byte( &si5351_reg[i] );
        if( reg == 255 ) break;         // end file marker
        i2cd( SI5351, reg, data );
        //Serial.print( reg );   Serial.write(' ');
        //Serial.println(data);
     }

        // set some divider registers that will never change
   for(int i = 0; i < 3; ++i ){
     i2cd(SI5351,42+8*i,0);
     i2cd(SI5351,43+8*i,1);
     i2cd(SI5351,47+8*i,0);
     i2cd(SI5351,48+8*i,0);
     i2cd(SI5351,49+8*i,0);
   }

   // i2flush();

   i2cd( SI5351, 177, 0xAC );         // PLLA PLLB soft reset
   // i2flush();
   delay(10);
   i2cd( SI5351, 177, 0xAC );
   // i2flush();
}

void i2cd( unsigned char addr, unsigned char reg, unsigned char dat ){

    i2start( addr );
    i2send(  reg );                    // register or 1st data byte if no registers in device
    i2send(  dat );
    i2stop();
}


// load a new frequency into PLL A or B 
// the output divider is fixed per band in use and not recalculated
void  si_pll_x(unsigned char pll, uint64_t freq, int out_divider ){
 uint64_t a,b,c;
 uint64_t bc128;             // floor 128 * b/c term of equations
 uint64_t pll_freq;
 uint64_t clock_freq = (uint64_t)CLOCK_FREQ;

 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3
 uint64_t r;


   // set c such that each b is 1hz change
   c = clock_freq / out_divider;     // max 1048575, for TX min divider of 24 for 25mhz clock, 26 for 27mhz clcok 
   if( c > 1048575 ) c = 1048575;    // vfo on high bands will have dividers less than 24
   
   pll_freq = freq * (uint64_t)out_divider;
   a = pll_freq / (clock_freq);
   r = pll_freq - a * (clock_freq);
   b = ( c * r ) / (clock_freq);
   bc128 =  (128 * r)/ (clock_freq);   // 128*b/c, b = c*r, sub c*r for b,  128*c*r/c,   128*r, div by cfreq, get some fraction of 128
   P1 = 128 * a + bc128 - 512;
   P2 = 128 * b - c * bc128;
   if( P2 >= c ) P2 -= c, ++P1;        // could this happen with truncation of unsigned values? have seen it once I think
   P3 = c;

   // save base solution for generating fractional xmit tone offsets, tx uses PLLA
   if( pll == PLLA ){
       _P1 = P1;  _P2 = P2; _P3 = P3;
   }
   i2cd(SI5351, pll + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 1, (P3 & 0x000000FF));
   i2cd(SI5351, pll + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, pll + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 4, (P1 & 0x000000FF));
   i2cd(SI5351, pll + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, pll + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, pll + 7, (P2 & 0x000000FF));
    
}


// init has loaded other registers with the clock builder values to allow this simplified code to work
void si_load_divider( int val, int clk , int rst){
 
   val = 128 * val - 512;
   i2cd( SI5351, 44+8*clk, (val >> 16 ) & 3 );
   i2cd( SI5351, 45+8*clk, ( val >> 8 ) & 0xff );
   i2cd( SI5351, 46+8*clk, val & 0xff );   
   if( rst ) i2cd( SI5351, 177, 0xA0 );         // PLLA PLLB soft reset needed?
}

void si_tone_offset( float val ){
 // P3 or c has been chosen such that a change of 1 hz changes P2 by 128
 uint32_t P1;            // PLL config register P1
 uint32_t P2;            // PLL config register P2
 uint32_t P3;            // PLL config register P3

   P1 = _P1;  P2 = _P2;  P3 = _P3;    // base value

   P2 += (uint32_t)(128.0 * val);     // add the tone offset, positive only
   while( P2 >= P3 ) P2 -= P3, ++P1;

   // transmitting using PLLA
   i2cd(SI5351, PLLA + 0, (P3 & 0x0000FF00) >> 8);
   i2cd(SI5351, PLLA + 1, (P3 & 0x000000FF));
   i2cd(SI5351, PLLA + 2, (P1 & 0x00030000) >> 16);
   i2cd(SI5351, PLLA + 3, (P1 & 0x0000FF00) >> 8);
   i2cd(SI5351, PLLA + 4, (P1 & 0x000000FF));
   i2cd(SI5351, PLLA + 5, ((P3 & 0x000F0000) >> 12) | ((P2 & 0x000F0000) >> 16));
   i2cd(SI5351, PLLA + 6, (P2 & 0x0000FF00) >> 8);
   i2cd(SI5351, PLLA + 7, (P2 & 0x000000FF));

}

void i2init(){
    Wire.begin();
}

void i2start( unsigned char address ){

  Wire.beginTransmission( address);
  
}

void i2send( unsigned char data ){

  Wire.write( data );
  
}

void i2stop( ){

  Wire.endTransmission();
  
}
