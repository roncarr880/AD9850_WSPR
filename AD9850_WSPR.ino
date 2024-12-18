/*
 *   Chipkit  uc32
 *   AD9850 DDS
 *   QRP Labs Arduio Shield
 *   QRP Labs Reciever Module
 *   
 *   WSPR transmitter on fixed frequency
 *  
 *  !!! add a real time clock 
 *  On rx, the dds has jitter starting around 40 mhz, may need to rx 30 meters at 1/3 freq.  20 meters and up will 
 *      need to be at 1/3 freq and will need bandpass filters on RX.
 */

 //  Arduino shield pin assignments for AD9850 module

 #define DATA A4     // 18  jumpers to A4 and A5 side on chipkit uC32
 #define FQUD A5     // 19
 #define WCLK  3
 //#define RESET  2    // reset is hardwired to ground on shield 
 #define TX_INHIBIT 2  // connected to DDS comparitor POT to disable square wave output during RX

 // wiring but unused for now
 #define BAND0  7     // relays, drive low to enable
 #define BAND1 A0
 #define BAND2 10
 #define BAND3 11
 #define BAND4 12
 #define BAND5 A3
 #define RX_ENABLE_LOW A2

 #define GPS1pps  9   // I think there are jumpers on the module to unwire these pins but not mentioned in the documentation
 #define GPSRxD   0
 #define CLK0     5
 #define CLk2     1   // TX pin, is this correct, not sure a good choice for this I/0 input

 #define ON 1
 #define OFF 0
 #define POWER_DOWN  0x4

 #define LED1  13
 #define LED2  43
 

const float DDS_1hz  =  34.359738368;      // dds load word per hz for 125 meg clock
const float trim_ = 66.0 / 10000000.0;     // freq error as percent of 10 meg, 50?

uint32_t  freq = 10138700;          // SSB base frequency
uint32_t  wspr_offset = 1500;       // 1400 to 1600 valid value
// rx has 474 caps so it is a very narrow band sdr
uint32_t  IF_freq = 4000;           // HDSDR offset away from zero beat noise ( was 4915000 K2 IF freq )

//uint32_t  IF_base;

// seeing some freq jumps, is it float values being non deterministic or 125 mhz osc jumping
// calc the base freq word one time
uint32_t  base;          

// tones - 1.4648 * value
//  the current message is   "KE1MU FN54 23"
/*
const uint32_t wspr_msg[] = {
      3, 1, 2, 2, 0, 2, 2, 2, 3, 2, 2, 2, 1, 1, 3, 2, 0, 0, 3, 0, 0, 1, 0, 1, 3, 3, 1, 2, 0, 2,
      0, 2, 2, 2, 3, 2, 0, 1, 0, 1, 2, 2, 0, 0, 0, 0, 3, 0, 3, 1, 2, 0, 3, 1, 0, 1, 2, 0, 2, 3,
      1, 2, 3, 2, 0, 0, 0, 1, 1, 2, 1, 0, 1, 2, 3, 0, 1, 0, 2, 3, 0, 0, 3, 0, 3, 1, 0, 2, 0, 3,
      3, 2, 3, 2, 1, 0, 2, 2, 1, 2, 0, 0, 0, 2, 3, 2, 2, 1, 0, 2, 1, 3, 1, 0, 1, 1, 0, 0, 3, 1,
      2, 3, 2, 0, 0, 1, 1, 3, 2, 2, 0, 2, 2, 3, 0, 1, 2, 0, 3, 1, 2, 2, 0, 0, 0, 2, 2, 1, 1, 0,
      3, 2, 1, 1, 0, 2, 2, 1, 3, 0, 0, 2
 };
 */

// KE1MU FN54 27   running 400+ mw
const uint32_t wspr_msg[] = {
      3, 1, 2, 0, 0, 0, 2, 0, 3, 0, 2, 0, 1, 3, 3, 2, 0, 0, 3, 2, 0, 3, 0, 3, 3, 3, 1, 2, 0, 2,
      0, 0, 2, 2, 3, 0, 0, 3, 0, 3, 2, 2, 0, 2, 0, 0, 3, 0, 3, 1, 2, 0, 3, 1, 0, 1, 2, 2, 2, 1,
      1, 0, 3, 2, 0, 0, 0, 1, 1, 0, 1, 2, 1, 0, 3, 2, 1, 0, 2, 3, 0, 0, 3, 0, 3, 1, 0, 2, 0, 1,
      3, 0, 3, 2, 1, 0, 2, 2, 1, 2, 0, 2, 0, 2, 3, 2, 2, 3, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 3, 3,
      2, 3, 2, 0, 0, 3, 1, 3, 2, 2, 0, 2, 2, 3, 0, 1, 2, 2, 3, 1, 2, 0, 0, 0, 0, 2, 2, 1, 1, 2,
      3, 0, 1, 1, 0, 2, 2, 3, 3, 0, 0, 2
};
       

/*
// message "KE1MU FN54 13"
const uint32_t wspr_msg[] = {
      3, 1, 2, 2, 0, 0, 2, 2, 3, 0, 2, 0, 1, 3, 3, 0, 0, 0, 3, 0, 0, 1, 0, 1, 3, 3, 1, 0, 0, 0,
      0, 0, 2, 2, 3, 0, 0, 3, 0, 3, 2, 2, 0, 0, 0, 0, 3, 0, 3, 3, 2, 2, 3, 3, 0, 1, 2, 2, 2, 3,
      1, 2, 3, 2, 0, 0, 0, 3, 1, 2, 1, 0, 1, 2, 3, 2, 1, 0, 2, 3, 0, 0, 3, 2, 3, 3, 0, 2, 0, 1,
      3, 0, 3, 2, 1, 2, 2, 2, 1, 2, 0, 2, 0, 2, 3, 0, 2, 3, 0, 0, 1, 1, 1, 2, 1, 1, 0, 2, 3, 1,
      2, 1, 2, 2, 0, 3, 1, 3, 2, 2, 0, 2, 2, 1, 0, 3, 2, 0, 3, 3, 2, 2, 0, 0, 0, 2, 2, 1, 1, 2,
      3, 2, 1, 3, 0, 0, 2, 3, 3, 2, 0, 2
}; 
*/

/*
// 8 bands with two dummy giving a 20 minute frame
const uint32_t bands[10] = {
   3568600, 7038600, 10138700, 14095600, 18104600, 21094600, 24924600, 28124600, 50293000, 50293000
};
const uint8_t bands_active[10] =
    { 0,      0,        0,        0,        0,       0,         0,        0,       0,       0
};
*/

/*
const uint32_t bands[10] = {
   18104600,18104600,18104600,18104600,18104600,18104600,18104600,18104600,18104600,18104600
}; */

// 17 meter filter
/*
const uint32_t bands[10] = {
 14095600 ,14095600, 14095600,  18104600, 18104600, 18104600,  14095600,14095600, 18104600,  18104600
};*/


// 30 meter filter
// rx only on 30 meters, tx on 30 and 40.  WSJTX rx only on 30.
//const uint32_t bands[10] = {
// 7038600, 10138700, 10138700, 10138700, 10138700, 10138700, 10138700, 10138700, 10138700, 10138700 
//};
// rx on 40 meters only
const uint32_t bands[10] = {
 10138700,  7038600,  7038600,  7038600,  7038600,  7038600,  7038600,  7038600,  7038600,  7038600
};

const uint8_t bands_active[10] = {
     1,      0,       1,        0,        0,        0,        1,        0,       0,         0
};


volatile unsigned int wspr_tx_enable;     // turn wspr on or off

void setup() {
 pinMode(DATA,OUTPUT);
 pinMode( FQUD, OUTPUT); 
 pinMode( WCLK, OUTPUT);
 //pinMode( RESET, OUTPUT);
 digitalWrite( DATA, LOW );
 digitalWrite( FQUD, LOW );
 digitalWrite( WCLK, LOW );
 //digitalWrite( RESET, LOW );
 pinMode( RX_ENABLE_LOW, OUTPUT );
 digitalWrite( RX_ENABLE_LOW, HIGH );

 pinMode( LED1, OUTPUT );
 pinMode( LED2, OUTPUT );
 
   delay( 50 );
   dds_init();

    base = (float)freq * DDS_1hz;
    base += (float)freq * trim_ * DDS_1hz;
  //  IF_base = (float)IF_freq * DDS_1hz;
  //  IF_base += ( float)IF_freq * trim_ * DDS_1hz;

   attachCoreTimerService( wspr_core );
}

void loop() {
 static int band; 
//uint32_t  val;
//static int count;      // 162 bits to send
//static int mod;
//int timer;


    freq = bands[band];
    base = (float)freq * DDS_1hz;
    base += (float)freq * trim_ * DDS_1hz;
    
    if( bands_active[band] ) wspr_tx_enable = 1;
    else transmit( OFF );         // load the receive frequency
    delay( 1000 * 2 * 60 );       // wait two minutes
    if( ++band == 10 ) band = 0;

/*******
    // send the wspr message over and over

   // val = (float)freq * DDS_1hz;
   // val += (float)freq * trim_ * DDS_1hz;
    val = base;
    val += ( float)wspr_offset * DDS_1hz;
    val += (float)wspr_msg[count] * 1.4648 * DDS_1hz;
    load_dds( val );

    timer = 683;
    if( ++mod == 3 ) mod = 0, --timer;    // delay 683, 683, 682, etc.
    delay( timer );

    ++count;
    if( count >= 162 ){
       // val = (float)freq * DDS_1hz;      // set base freq
       // val += (float)freq * trim_ * DDS_1hz;
        load_dds( base );
        count = 0;
        delay( 9408 );   // end of transmission dead time 9 seconds
    } 
    *****/
}

void dds_init(){

  // the dds reset pin is hardwired to ground on the shield
  // D0 and D1 are hardired to vcc, D2 is hardwired to ground on the shield

  // digitalWrite( RESET, HIGH );
  // delay(1);
  // digitalWrite( RESET, LOW );
  
   delay(1);

   digitalWrite( WCLK, HIGH );
   digitalWrite( WCLK, LOW );
   digitalWrite( FQUD, HIGH );
   asm( "nop" ); 
   digitalWrite( FQUD, LOW );
   delay( 1 );
  
}


// LATESET LATECLR LATEINV registers direct writes for speed
// DATA is A4  RB12
// WCLK is  3  RD0
// FQUD is A5  RB14
void load_dds( uint32_t val, uint8_t phase ){
int i;

   for( i = 0; i < 32; ++i ){
      //digitalWrite( DATA, val & 1 );
      if( val & 1 ) LATBSET = 1 << 12;
      else LATBCLR = 1 << 12;
      
      val >>= 1;
      //digitalWrite( WCLK, HIGH );
      LATDSET = 1;
      //digitalWrite( WCLK, LOW );
      LATDCLR = 1;
   }

   //digitalWrite( DATA, LOW );
   LATBCLR = 1 << 12;
   for( i = 0; i < 8; ++i ){

      if( phase & 1 ) LATBSET = 1 << 12;
      else LATBCLR = 1 << 12;
      phase >>= 1;
      //digitalWrite( WCLK, HIGH );
      LATDSET = 1;
      //digitalWrite( WCLK, LOW );
      LATDCLR = 1;
   }

   //digitalWrite( FQUD, HIGH );
   LATBSET = 1 << 14;
   asm( "nop" ); 
   //digitalWrite( FQUD, LOW );
   LATBCLR = 1 << 14;
}


/*   WSPR  core timer function */
#define WSPRTICK 27307482     // 1 bit time for 1.4648 baud.  (40M ticks in 1 second) CORE_TICK_RATE is counts in 1ms
// delay is 1/1.4648 seconds
// #define WSPRTICK 27306667        // or should it be 1.46484375 baud.  120000/8192

uint32_t  wspr_core( uint32_t timer ){
static int count;
static uint32_t val;
static uint32_t wspr_val;


  if( wspr_tx_enable == 0 )   return timer + WSPRTICK;   // nothing to do

  if( count == 162 ) {   // stop the transmitter
     transmit(OFF);
     wspr_tx_enable = 0;
     count = 0;
     return timer + WSPRTICK;
  }
    
  if( count == 0 ){     // float precalc
      val = base;
      val += ( float)wspr_offset * DDS_1hz;
      wspr_val = uint32_t(1.4648 * DDS_1hz);
  }

  load_dds( val + wspr_msg[count] * wspr_val, 0 );
  digitalWrite( LED2, count & 1 );


  if( count == 0 ) transmit(ON);
  ++count;
   
  return timer + WSPRTICK;    
}

void transmit( int enable ){

   if( enable == OFF ){        // set freq to base
       //load_dds( base, POWER_DOWN);   power down results in drift when powered up again
       pinMode( TX_INHIBIT, OUTPUT );
       digitalWrite( TX_INHIBIT, LOW );
       load_dds( calc_rx_freq_val( ), 0 );       // rx at SDR IF freq
       digitalWrite( LED1, LOW );
       digitalWrite( LED2, LOW );
       digitalWrite( RX_ENABLE_LOW, LOW );
   }

   if( enable == ON ){
       digitalWrite( RX_ENABLE_LOW, HIGH );
       digitalWrite( LED1, HIGH );
       pinMode( TX_INHIBIT, INPUT );
   }
}

uint32_t calc_rx_freq_val( ){
uint32_t val;
uint32_t rx_freq;

   rx_freq = 4 * (freq - IF_freq);
   if( rx_freq > 44000000 ) rx_freq = rx_freq / 3;     // use 1/3 clock, will need a bandpass filter for >= 20 meters
   //  or attenuator on then also /3
   val = (float)rx_freq * DDS_1hz;
   val += (float)rx_freq * trim_ * DDS_1hz;
   return val;
}
