
/* ********************
  wspr tx on solar power
  PIC16F690
  DDS board
  QRP LABS arduino shield
  5 volt NIMH battery
  Canaduino WWVB receiver

  A high side switch keeps the DDS off to reduce battery drain.  This causes frequency drift as it starts cold.
  The frequency is counted before and after the wspr transmission to adjust the drift compensation amount.
  The PIC SPI module is used for clocking the DDS.
  The PIC UART is used at 10 baud for receiving the time signal.
  Timer2 is used to produce a 1ms interrupt for timing.
  The internal voltage reference is read with the A/D converter to calculate the battery voltage.
  The ground lead of the 7805 has a diode to ground to increase the charging voltage.  A BS170 is across the
    diode to control charging ( on or off ).  A 4.7 ohm series resistor keeps charge current low.

*********************** */

/*   Wiring
  Processor                  QRP Labs Shield I/O pin or other connection
11   RB6  spi clk                3
13   RB4  fq_updt               A5     ( sdi does work as an output )
 9   RC7  spi data              A4
12   RB5  RX                     0     Canaduino wwvb rx signal. Can use shield gps connector. 
10   RB7  TX                     unused

 4   RA3                        VPP programming, for reset function wired to shield pin 1
 3   RA4                        xtal osc
 2   RA5                        xtal osc
19   RA0                        isp data
18   RA1                        isp clock
17   RA2                        freq count input - arduino pin 5
 1   VDD                        VCC +5
20   VSS                        Ground

16   RC0                        Led0
15   RC1                        Led1
14   RC2                        wwvb power - wire to 3.3
 7   RC3                        DDS power control via high side switch
 6   RC4                        Charge control, high is off. shorts diode in 7805 ground lead to reduce volts in
 5   RC5                        
 8   RC6
   (  RC7    used   for   spi  wired as above )

  Arduino shield power assignments
  Vin    5 volt battery side to DDS board
  +5     cut pin between boards, DDS has circuit to get reduced power from Vin
  3.3    Power to WWVB reciever using an output pin

  Add diode to 7805 ground lead to get 5.6 volts out for charging.  Add FET across it to control charging.

*/

#include "p16f690.h"

/* dds and tx commands */
#define POWER_DOWN  0x4
#define POWER_UP    0
#define ENABLE 1
#define DISABLE 0

/*  flags bits */
#define WSPR_TICK 1
#define FRAME 2
#define VCC_CHECK 4
#define VIN_CHECK 8
#define PRE_TX 16

/* limits for battery: ok to tx, charge on and off, think bat ok should be 117 */
/* numbers are A/D readings of the internal 0.6v reference */
#define BAT_OK 119
#define BAT_LOW 109
#define BAT_HIGH 105

/* port C single bit i/o */
#asm
LED0 equ 0
LED1 equ 1
WWVB_ON equ 2
DDS_PWR equ 3          ; inverse logic and high side switch is now dds power control
BAT_CHARGE equ 4       ; inverse logic
#endasm

/* took quite some time to learn that VP6 is active when using the internal oscillator but needs to 
   be enabled in register VRCON when using a HS crystal  */

#asm
    ;  changed osc to external crystal
    __CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF & _HS_OSC & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF

  ;  __CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF &  _INTRC_OSC_NOCLKOUT & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF

#endasm

#define CAL 14      /* freq a bit low, adj value in about 7.5 hz steps */
                    /* might be different for 30 vs 20 meters */

/* eeprom */
/*extern char EEPROM[2] = {0xff,0xff};     eeprom as one big array */
/*   dds load value per 1 hz for 125mhz xtal = 34.359738368 */
/*  20 meters solution , and each 1.4648 step adds 50 ( 50.33 ) */
/* 2 band solution, in load_base read an i/o pin and load band in use */
extern char base20[4] = { 0x1c, 0xde, 0xf0, 0xbc };
extern char base30[4] = { 0x14, 0xc4, 0x62, 0x9b };

/* wspr msg packed, last byte has just 2 entries, rest 4, message starts with LSBits */
extern char wspr_msg[41] = {
0xA7, 0xA8, 0xAB, 0xB5, 0x30, 0x44, 0x9F, 0x88, 0xBA, 0x44, 
0xA, 0x30, 0x27, 0x47, 0xE2, 0xB9, 0x40, 0x19, 0x39, 0xE1, 
0x30, 0x87, 0xBC, 0x1B, 0x9A, 0x80, 0x6B, 0xD8, 0x51, 0x70, 
0x2E, 0xD4, 0x8A, 0x4E, 0x72, 0xA, 0x68, 0xB1, 0x85, 0x36, 
0x8 };
extern char slots[6] = { 2, 10, 18, 28, 38, 48 };     /* xmit on these minutes, save 50-60 for wwvb rx */
extern char EE_DRIFT = 15;



/* access ram  - 16 locations, use wisely */
#pragma pic 112

char w_temp;    
char status_temp;
char pclath_temp;
char _temp;                        /* temp location for subtracts and shifts */
char _temp2;                       /* for interrupt subs and shifts */
char _eedata;                      /* eedata in access ram */
/* 10 more locations for access ram */
char ms_L, ms_H;                   /* milli seconds 0 to 1000  0x3e8 */
char sec;                          /* 0 - 60 */
char min;                          /*  */
char wspr_tick_L;                  /* wspr timing 683ms 683ms 682ms  0x2ab */
char wspr_tick_H;
/* 4 more */
char flags;
char dv;                    /* dummy variable for when need one */
char txcount;
char freq_count;



/* bank 0 ram - 80 locations */
/* any arrays need to be in page 0 or page 1 as the IRP bit is not updated */
#pragma pic  32
char transmitting;
char  wwvb_data[11];
char vcc;
char bat_ok;
char vin_ok;
char acc0,acc1,acc2,acc3;   /* 32 bit working registers */
char arg0,arg1,arg2,arg3;    
/* at 24 */
char charging;
char drift;
char drift_l;
char drift_h;
char trans_count;           /* tx's since wwvb match */

/* bank 1 ram - 80 bytes */
#pragma pic  160
char pre_tx;
char post_tx;
char tmr0_pre_L;       /* prescaler value */
char tmr0_pre_H;       /* timer value */
char tmr0_post_L;
char tmr0_post_H;


/* bank 2 ram - no arrays - 80 locations - function args and statics are here */
#pragma pic 288


/*  ********
#pragma shortcall   - use for functions that do not call other pages
#pragma longcall    - use for functions that call other pages
                    - do not allow code to pass over the page boundary
********  */

#pragma shortcall

main(){       /* any page */

for(;;){    /* loop main needed */
static char i;
static char pre_tx;


   if( flags & PRE_TX ){     /* tx early to remove the starting hook as we come on freq */
      #asm        
        bcf flags, 4 
      #endasm
      pre_tx = 0;            /* pretty much a duplicate of code below except the minute */
      dv = min + 1;
      for( i = 0; i < 6; ++i ) if( slots[i] == dv ) pre_tx = 1;
      if( bat_ok == 0 || vin_ok == 0 ) pre_tx = 0;
      if( pre_tx ){
         dds_on();
         load_base();
         arg3 = arg2 = arg0 = 0;      /* warm up slightly off freq */
         arg1 = 20;
         dsub();
         clock_dds( POWER_UP );
         if( trans_count != 255 ) ++trans_count;   /* tx's since wwvb match */
      }
   }

   if( flags & FRAME ){
      #asm
       bcf flags,1
      #endasm
     /* **
      for( i = 0; i < 6; ++i ) if( slots[i] == min ) transmitting = 1;
      if( bat_ok == 0 || vin_ok == 0 ) transmitting = 0;
     *** */
      if( pre_tx ){
         transmitting = 1;
        /* clock_dds( POWER_UP ); */
      }
   }

   if( flags & WSPR_TICK ){
      #asm
        bcf flags,0     ; are these atomic 
      #endasm
      if( transmitting ) wspr_tx( ENABLE );
      else if( pre_tx ){
         if( ++pre_tx == 7 ) freq_count = 1, post_tx = 0;
      }
      else if( post_tx ){
         if( ++post_tx == 5 ) freq_count = 1, pre_tx = 0;
      }
   }

   if( PIR1 & 0x20 ) wwvb_rx();

   /* voltage checks. Control charging, transmitting and when wwvb rx is on */
   if( flags & VIN_CHECK ) check_vin();
   if( flags & VCC_CHECK ) check_bat();

  /* adjustment of drift compensation */
   if( freq_count == 3 ){      /* get frequency count */
      freq_count = 0;
                               /* read result and save pre or post tx */      
      if( pre_tx ){
         clock_dds( POWER_DOWN );
         tmr0_pre_H = TMR0;
         tmr0_pre_L = read_prescaler();
         clock_dds( POWER_UP );
      }

      if( post_tx ){           /* get count and turn off tx */
         clock_dds( POWER_DOWN ); 
         tmr0_post_H = TMR0;
         tmr0_post_L = read_prescaler();
         dds_off();

         if( tmr0_post_H != tmr0_pre_H ){     /* assume off by one */
            tmr0_post_L += 128;               /* one of these additions should overflow */
            tmr0_pre_L += 128;
         }
  /* fudge factor, algorithm seems to bias towards positive drift as reported by wsjt-x */
         if( tmr0_post_L != 255 ) ++tmr0_post_L; 

         if( tmr0_post_L > tmr0_pre_L && drift != 0 ) --drift;
         if( tmr0_post_L < tmr0_pre_L ) ++drift;
         short_blink( drift );
       /*  short_blink( tmr0_post_L ); */
      }

   }


}
}


char read_prescaler(){
static char tmr;
static char val;

/* scope trigger */
/* ***
  #asm
   banksel PORTC
   bsf PORTC, RC5
  #endasm
*** */

     val = 0;
     tmr = TMR0;

  while( tmr == TMR0 ){
     #asm
      banksel PORTA
      bcf PORTA, RA2
      bsf PORTA, RA2
     #endasm
     ++val;
   }
   #asm
    banksel TRISA
    bsf TRISA, RA2       ; return pin to input, timer runs freely
   #endasm

/* scope trigger */
/* ****
  dv = dv;
  #asm
   banksel PORTC
   bcf PORTC, RC5
  #endasm
*** */


   /* negate value */
   return ( ~val + 1 );

}

void check_vin(){           /* try to determine if solar power is available */
static char v1, v2;

    #asm
     bcf flags, 3
    #endasm

   if( transmitting ) return;
   if( charging == 0 ){
      vin_ok = 1;
      v1 = vcc;        /* get a base value */
      return;
   }

 /*  charging is on, measure vcc. */
 /* if lower value than previous, then have sunlight */
   v2 = vcc;
   if( v2 == v1 ) return;            /* can't tell */
    
   vin_ok = ( v2 < v1 ) ? 1 : 0;     /* inverse - higher battery is lower reading */
   v1 = v2;

}


char read_vp6(){

   ADCON0 = 0xb4;     /* VP6 selected */
   ADCON0 |= 1;       /* module on */
   small_delay(40);
   ADCON0 |= 2;       /* GO */
   while( ADCON0 & 2 );  /* wait not done */
   ADCON0 = 0xb4;     /* module off but selecting correct channel */

   return ADRESL; 
}

void check_bat(){            /* get a reading of the battery level */

    #asm
     bcf flags, 2
    #endasm

    if( transmitting ) return;
   
  
    #asm
     banksel PORTC
     bsf PORTC, BAT_CHARGE    ;  turn off charging for battery reading
    #endasm
    delay(100);

      /* read internal 0.6 volts with reference as VDD - device errata with VP6 ? */
      /* needed to turn on vp6 when using external osc */

   vcc = read_vp6();  

/* changed to 5 volt operation */
   if( vcc > BAT_OK ) bat_ok = 0;     /* new limits for 5 volt,  was 168,163 for 4 volt */
   if( vcc <= BAT_OK ) bat_ok = 1;

   if( vcc > BAT_LOW ) charging = 1;    /* put in new limits, experimental ones */
   if( vcc < BAT_HIGH ) charging = 0; 

   short_blink( vcc );   /* debug */

   if( charging == 1 ){    /* turn charging on and off  */
     #asm
      banksel PORTC
      bcf PORTC, LED0            ; reverse led indicator so is off when battery is low
      bcf PORTC, BAT_CHARGE
     #endasm
   }
   else{
     #asm
      banksel PORTC
      bsf PORTC, LED0            ; and on when battery is full
      bsf PORTC, BAT_CHARGE
     #endasm
   }


/*   voltage break points as displayed determined by experiment
    143 = 4.0
    147 = 3.9   raw calculation of 0.6/4.0 * 1024 would have 4.0 volts as 154 
    151 = 3.8    which is quite far off from the measured values
    155 = 3.7
    159 = 3.6
    163 = 3.5
    167 = 3.4
*/


   /* LED display */

  /*      keep    , more accurate break points than above comment from a different program */
/*
   switch( vcc ){
      case 136:  PORTC = 6; break;     /* 4.2 
      case 137:  PORTC = 7; break;
      case 138:  PORTC = 8; break;     /* 4.1 
      case 139:  PORTC = 9; break;     /* 4.1 
      case 140:  PORTC = 0; break;
      case 141:  PORTC = 1; break;     /* 4.0 
      case 142:  PORTC = 2; break;     /* 4.0 
      case 143:  PORTC = 3; break;
      case 144:  PORTC = 4; break;     /* 3.9 
      case 145:  PORTC = 5; break;     /* 3.9 
      default:  PORTC = 0xf; break;
   }
*/

}


void led_control( char t ){

   if( vcc > 124 ){       /* keep led's off when battery really low */
     #asm
      banksel PORTC
      bcf PORTC, 0
      bcf PORTC, 1
     #endasm
     return;
   }

   dv = t & 3;
   #asm
     banksel PORTC
     btfsc dv, 0 
     bsf PORTC, 0
     btfss dv, 0
     bcf PORTC, 0
     btfsc dv, 1 
     bsf PORTC, 1
     btfss dv, 1
     bcf PORTC, 1
   #endasm

}


void small_delay( char count ){

   while( count-- );

}



/*
   this wwvb receiver seems different from my previous one.  It does not respond to noise and seems to
   produce a 0,1 or sync onl which perhaps is not always correct.  Received signal edges seem pre-defined.
   As such I think there needs to be more sanity checking other than framing errors and 0,1 or sync received.
*/
#define SYNC 0x80
#define ZERO 0xfe
#define ONE  0xf0

void wwvb_rx(){
static char dat;
static char i;
static char ok;
static char t;
static char wmin;
static char errors;       /* receive 60 seconds of valid data before allowing decode on 10 elements */
static char dist;

   #asm
    banksel PIR1
    bcf PIR1, RCIE
   #endasm

   dat = RCREG;
   if( RCSTA & 0x6 ){      /* errors */
      #asm
       ; banksel RCSTA
       bcf RCSTA, CREN
       bsf RCSTA, CREN
      #endasm
      dat = 0xff;
    }

   for( i = 0; i < 10; ++i ){
      dv = wwvb_data[i+1];
      wwvb_data[i] = dv;
   }  
   wwvb_data[10] = dat;

   /* extra sanity checks, look for 1 minute of error free signal */

   if( errors ) --errors;

   /* if no match in 48 hours, loosen the requirements, just match 1st 11 seconds */
   if( trans_count != 255 ){
      if( dat != SYNC && dat != ONE && dat != ZERO ) errors = 60;

      /* distance between syncs  1 9 or 10 */
      ++dist;
      if( dat == SYNC ){
         if( dist != 1 && dist != 9 && dist != 10 ) errors = 60;
         dist = 0;
      }
   }

   /* match first 11 of wwvb data frame */
   if( wwvb_data[0] == SYNC && wwvb_data[1] == SYNC && wwvb_data[10] == SYNC && wwvb_data[5] == ZERO ){

      ok = (errors) ? 0 : 1;        /* if errors then decode is defeated */
      wmin = 0;
      t = 40;                       /* weights 40,20,10,8,4,2,1 */
      for( i = 2; i < 5; ++i ){
         dv = wwvb_data[i];
         if( dv != ONE && dv != ZERO ) ok = 0;
         if( dv == ONE ) wmin += t;
         t >>= 1;
      }
      t = 8;
      for( i = 6; i < 10; ++i ){
         dv = wwvb_data[i];
         if( dv != ONE && dv != ZERO ) ok = 0;
         if( dv == ONE ) wmin += t;
         t >>= 1;
      }

      if( ok == 1 && wmin < 60){            /* a match, should be at 10 seconds */     
          no_interrupts();
           sec = 10;
           min = wmin;
          interrupts();
          short_bell();
          trans_count = 0;
       /* done for this hour, so turn off rx here, and turn off the LED's, and return */  
          #asm
           banksel PORTC
           bcf PORTC, WWVB_ON
          ; bcf PORTC, LED1        ; short_bell cleans up the led's
          ; bcf PORTC, LED0
          #endasm
          return;
      }

   }

/*  LED's */

   if( dat == SYNC ) dv = 3;
   else if( dat == ONE ) dv = 2; 
   else if( dat == ZERO ) dv = 1;
   else dv = 0;

   led_control( dv );

  /* dv = ( dat == SYNC ) ? 3 : (dat == ONE) ? 2 : (dat == ZERO ) ? 1 : 0; */

}


void delay( char count ){           /* delay up to 255 ms */
static char t;

   while( count-- ){
      t = ms_L;
      while( t == ms_L );
   }
}


/* keeping the dds control signals at zero volts to avoid latchup as the dds power is turned on and off */
/* boost power during tx */
void dds_on(){

   /* power up */
   #asm
    banksel PORTC
    bcf  PORTC, BAT_CHARGE     ;  enable low, turn on more power
    bcf  PORTC, DDS_PWR        ;  high side switch on
   #endasm

   delay( 20 );     /* high side switch is slow soft start */

/* init DDS, clock and load to get into serial mode, twice just in case */
/* RB6 is clock, RC7 is data, ( SDI ) RB4 as the load pulse */

  #asm
     banksel PORTB
     bcf PORTB, RB6    ; clock low
     bsf PORTB, RB6    ; clock high
     nop
     bsf PORTB, RB4    ; load 
     bcf PORTB, RB4
     nop               ; once more
     bcf PORTB, RB6    ; clock low
     bsf PORTB, RB6    ; clock high
     nop
     bsf PORTB, RB4    ; load 
     bcf PORTB, RB4
  #endasm       

/* set up SPI */
   SSPSTAT = 0x00;      /* default is zero, different clock to data timing with CKE 0x40 set? */
   SSPCON  = 0x30;      /* need clock idle high for correct data timing to clock rising edge */
  /* SSPBUF  = 0;  */   /* dummy write to set buffer full flag, didn't work, ignoring buffer full */
   clock_dds( POWER_DOWN );
}

void dds_off(){

   
   SSPCON = 0;          /* spi off */
   PORTB = 0;           /* clock, fq_up low */
   #asm
    banksel PORTC
    bcf PORTC, RC7       ; sdo low
    bsf PORTC, DDS_PWR   ; high side switch off
   #endasm

   if( charging == 0 ){
     #asm
      banksel PORTC
      bsf PORTC, BAT_CHARGE   ; turn off boost power
     #endasm
   }


}


void wspr_tx( char var ){
static char b;              /* di bits sent */
static char c;              /* char position */     

   if( var == DISABLE || txcount == 162 ){
      txcount = 0;
      transmitting = 0;
      if( var != DISABLE ){
        post_tx = 1;                   /* set up for post tx freq count */
        load_base();
        arg3 = arg2 = arg0 = 0;        /* pre init freq with offset */
        arg1 = 20;
        dsub();
        arg1 = drift_h;                /* add drift compensation */
        arg0 = drift_l;
        #asm
         ASRD arg1,arg0        ; shift out 2 fractional bits
         ASRD arg1,arg0
        #endasm
        dadd();
        arg1 = 0;   arg0 = drift;      /* we go 5 ticks past end of message, add that in ( 4/5ths of it ) */
        dadd();
        dadd();                        /* and we were early by 4 or 5 ticks */
        clock_dds( POWER_UP );
      }
      else{
        clock_dds( POWER_DOWN ); 
        dds_off(); 
      }
      led_control( 0 );
      drift_l = drift_h = 0;
      return;
   }

   b = txcount & 3;
   c = txcount >> 2;
   var = wspr_msg[c];
   while( b-- ) var >>= 2;
   var &= 3;
   led_control( var );
   ++txcount;
   arg0 = 0;
   while( var-- ) arg0 += 50;       /* wspr offset */
   load_base();
   arg3 = arg2 = 0;
   arg1 = CAL;                     /* freq calibration in 7.5 hz steps */
   dadd();
   arg1 = drift_h;                 /* drift compensation */
   arg0 = drift_l;
   #asm
    ASRD arg1,arg0        ; shift out 2 fractional bits
    ASRD arg1,arg0
   #endasm
   dadd();
   clock_dds( POWER_UP );

   /* calc new drift for next symbol tx */
   zacc();
   arg3 = arg2 = arg1 = 0;
   arg0 = drift;
  /* try secondary curve with less drift compensation near end of transmission */
   if( txcount < 60 ) ++arg0;
   acc1 = drift_h;
   acc0 = drift_l;
   dadd();
   drift_h = acc1;
   drift_l = acc0;

}

void load_base(){

   acc3 = base20[0];
   acc2 = base20[1];
   acc1 = base20[2];
   acc0 = base20[3];

}



clock_dds( char powered ){
   /* data to load is in the accumulator */

   SSPBUF = bitrev(acc0);     /* bitrev has ample delay for spi to finish byte */
   SSPBUF = bitrev(acc1);
   SSPBUF = bitrev(acc2);
   SSPBUF = bitrev(acc3);
   SSPBUF = bitrev(powered);

   /*  toggle load pin */
   small_delay(1);           /* spi clocks not done yet */
   #asm
     banksel PORTB
     bsf PORTB, RB4
     nop
     bcf PORTB, RB4
   #endasm

}


char bitrev( char data ){    /* reverse the bits in a byte */

  #asm
    clrw
    btfsc  _bitrev,7
    iorlw  0x01
    btfsc  _bitrev,6
    iorlw  0x02
    btfsc  _bitrev,5
    iorlw  0x04
    btfsc  _bitrev,4
    iorlw  0x08
    btfsc  _bitrev,3
    iorlw  0x10
    btfsc  _bitrev,2
    iorlw  0x20
    btfsc  _bitrev,1
    iorlw  0x40
    btfsc  _bitrev,0
    iorlw  0x80
    movwf  _bitrev
  #endasm

    return data;
}

/*
   short( 16 entries ) table lookup method, maybe would be 14 instructions + call / return
      needs source and dest in access ram
   above is 16 + save/load in C using some banksels + call/return
   this idea grew longer in instruction time as PCLATH needed to be fiddled with

   save to source from w
   and 0xf to f
   call lookup  3 instructions  call, jump, return
   save to dest
   swap dest
   swap source to f
   and 0xf
   call lookup
   ior dest
   load dest
   return result in w

*/



/* ******************
#asm
bitrev2
   movwf source
   movlw high bit_rev_table       ; more trouble than its worth?  Not saving time from the 1st algorithm
   movwf PCLATH                   ; and uses 2 access ram locations
   movf source,w
   call bit_rev_table
   movwf dest
   swapf dest,f
   swapf source,w
   call bit_rev_table
   iorwf dest,w
   return

bit_rev_table
   andlw 0xf
   addwf PCL,f
   dt 0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe, 0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf


        IF ((HIGH ($)) != (HIGH (bit_rev_table)))
            ERROR "bit_rev_table CROSSES PAGE BOUNDARY!"
        ENDIF
#endasm
***************** */


 
void zacc(){     /* zero the accumulator */
   
   acc3 = acc2 = acc1 = acc0 = 0;

}


char dadd(){    /* 32 bit add */

#asm 
     banksel arg0       ;this banksel needed if move the variables out of the shared ram area
     movf    arg0,W
     addwf   acc0,F
     movlw   1
     btfsc   STATUS,C
     addwf   acc1,F
     btfsc   STATUS,C
     addwf   acc2,F
     btfsc   STATUS,C
     addwf   acc3,F


     movf    arg1,W
     addwf   acc1,F
     movlw   1
     btfsc   STATUS,C
     addwf   acc2,F
     btfsc   STATUS,C
     addwf   acc3,F

     movf    arg2,W
     addwf   acc2,F
     movlw   1
     btfsc   STATUS,C
     addwf   acc3,F

     movf    arg3,W
     addwf   acc3,F
     
   #endasm

  /* should these functions return CARRY, need to merge 3 places it could overflow into a var */

}

char dsub(){    /* 32 bit sub */

   #asm

     banksel arg0
     movf    arg0,W
     subwf   acc0,F     ; borrow is carry 0
     movlw   1
     btfss   STATUS,C   ; propagate borrow to the 3 upper bytes
     subwf   acc1,F
     btfss   STATUS,C
     subwf   acc2,F
     btfss   STATUS,C
     subwf   acc3,F

     movf    arg1,W
     subwf   acc1,F
     movlw   1
     btfss   STATUS,C
     subwf   acc2,F
     btfss   STATUS,C
     subwf   acc3,F

     movf    arg2,W
     subwf   acc2,F
     movlw   1
     btfss   STATUS,C
     subwf   acc3,F

     movf    arg3,W
     subwf   acc3,F
     
   #endasm

}


interrupts(){

   #asm
      bsf   INTCON,GIE
      bsf   INTCON,PEIE   ;  need for tmr2 ? : yes  
   #endasm
}

/* is the compiler sensitive to no extra returns on the end of the file? Seems it is */
/* or it doesn't like #asm #endasm as the last block */

no_interrupts(){

   #asm
     bcf   INTCON,GIE
     btfsc INTCON,GIE    ;see AN576.  What devices have this issue?
     goto $-2
   #endasm
}


#pragma longcall

/* can have stub functions here to call into page1 */
void short_bell(){

   long_bell();
}

void short_blink( char val ){

   long_blink( val );
}

void short_big_delay( char val ){

   long_big_delay( val );
}

/* end of page 0 program section */


#asm
           org  0x800
#endasm

/********************  start of page 1 code section *************************/



/* any page. Save _temp if using any calls to other functions */
_interrupt(){  /* status has been saved */
static char mod;

   /* code here */
   /* can we get an accurate freq count */
   if( ms_L == 0 && ms_H == 0 ){
      if( freq_count == 1 ){        /* start count */
         #asm
           banksel TMR0
           clrf TMR0               ; start count at zero
         #endasm
         ++freq_count;
      }
      else if( freq_count == 2 ){   /* end count */
         #asm
          banksel PORTA
          bsf PORTA, RA2
          banksel TRISA
          bcf TRISA, RA2           ; set freq in as output driving high, overdrives signal
         #endasm
         ++freq_count;
      }
   }

   if( ++ms_L == 0 ) ++ms_H;
   if( ms_L == 0xe8 && ms_H == 0x3 ){    /* one second */
      ms_L = 0;
      ms_H = 0;
      if( ++sec == 60 ){
          sec = 0; ++min;
          if( min == 60 ){           /* and turn off wwvb rx */
             min = 0;
             #asm
                banksel PORTC
                bcf PORTC, WWVB_ON
             #endasm
          }
          if( min == 50  ){   /*  turn on wwvb rx, runs for 10 minutes each hour */
             #asm
                banksel PORTC
                bsf PORTC, WWVB_ON
             #endasm
          }
          if( (min & 1) == 0 ){         /* reset wspr tick at start of 2 min intervals */
            wspr_tick_L = 0xaa;         /* set to end next tick below */
            wspr_tick_H = 2;
            flags |= FRAME;
          }
      }
      if( sec == 29 ) flags |= VIN_CHECK;
      if( sec == 30 ) flags |= VCC_CHECK;
      if( sec == 50 && ( min & 1 ) ) flags |= PRE_TX;
   }

   if( ++wspr_tick_L == 0 ) ++wspr_tick_H;
   if( wspr_tick_L == 0xab && wspr_tick_H == 0x2 ){
      flags |= WSPR_TICK;
      wspr_tick_L = 0;
      wspr_tick_H = 0;
      if( ++mod > 2 ){           /* gen timing 683 683 682 .... */
         mod = 0;
         ++wspr_tick_L;          /* count starting from 1 */
      }
   }
   

/* restore status */
#asm
       banksel    PIR1                       ; clear timer 2 interrupt flag
       bcf        PIR1,TMR2IF
       
	movf		pclath_temp,w		; retrieve copy of PCLATH register
	movwf		PCLATH			; restore pre-isr PCLATH register contents	
	movf		status_temp,w		; retrieve copy of STATUS register
	movwf		STATUS			; restore pre-isr STATUS register contents
	swapf		w_temp,f
	swapf		w_temp,w		; restore pre-isr W register contents

#endasm
}



init(){       /* any page, called once from reset */

/* enable clock at 4 or 8 meg */
 /*  OSCCON= 0x70; */  /* external crystal or osccon =   0x60 is 4 meg internal clock or 0x70 for 8 meg */
   EECON1 = 0;     /* does the free bit sometime power up as 1 and cause corruption */


/* init variables */
  ms_L = ms_H = 0;                   /* milli seconds 0 to 1000  0x3e8 */
  sec = 0;                          /* 0 - 60 */
  min = 0;                          /*  */
  wspr_tick_L = 0;                  /* wspr timing 683ms 683ms 682ms  0x2ab */
  wspr_tick_H = 0;
  bat_ok = vin_ok = 1;
  transmitting = 0;
  flags = 0;
  txcount = 0;
  charging = 0;
  drift = EE_DRIFT;
  drift_h = 0;
  drift_l = 0;
  pre_tx = post_tx = 0;

/* init pins */


  /* OPTION_REG = 0b11100111; radix binary doesn't work with this compiler !! */
   OPTION_REG = 0xe7;
   #asm
     banksel PORTA
     bsf PORTA, RA2     ; high when enabled for stopping freq count, input for now
   #endasm

   ANSEL = 0;
   ANSELH = 0;
   TRISC = 0x00;          /*  SDO as output,2 unused outputs, dds_pwr, charge, wwvb, led's as outputs */
   PORTC = 0x18;          /*  wwvb off, led's off, dds_pwr off, charge off  */
 /* PORTC = 0x1c; */       /* turn wwvb rx on for testing */
  
   PORTB = 0;
   TRISB = 0xaf;      /* 0b10101111, rb6 as spi clk, RB4 works as an output in spi mode */

/* set up timer2 for a millis interrupt */
   PR2 = 124;           /* 2000000 / 16 / 125 = 1000.  one less count needed ?  124 or 125 */
   T2CON = 6;           /* on and 16:1 prescale */
   PIE1 = 2;            /* enable timer2 interrupts */
   INTCON = 0xc0;
                        /* interrupt function to clear TMR2IF in PIR1 */

/* set up uart for 10 baud recieve - wwvb */
   TXSTA = 0x0;           /* will tx pin be available for i/o if tx not enabled ?, or 0x20 */
   BAUDCTL = 0x08;        /* brg16 */
   SPBRGH = 0xc3;
   SPBRG = 0x4d;        /* c350 -1 calc value :  or a little lower if framing errors alot */
                        /* this works extremely well for how simple it is */  
   RCSTA = 0x90;        /* note: clear cren when errors */

/* set A/D to read internal reference for battery check */
   ADCON1= 0x20;         /* 8/32  clock */
   ADCON0= 0xB4;         /* set to read internal 0.6 volts */
   VRCON = 0x10;         /* turn on vp6 */


}

void long_bell(){     /* flash lights on low pin count board when wwvb match */

   for( dv = 0; dv < 4; ++dv ){
      #asm
       banksel PORTC
       bsf PORTC, LED0
       bcf PORTC, LED1
      #endasm
      delay(60);
      #asm
       banksel PORTC
       bcf PORTC, LED0 
       bsf PORTC, LED1
      #endasm
      if( dv != 3 ) delay(60);
   }
   #asm
    banksel PORTC
    bcf PORTC, LED1     ; turn off the light left on
   #endasm

}

/* display a number with blinking led's */
void long_blink( char val ){

  led_control( 0 );
  long_big_delay( 2 ); 

   while( val >= 100 ){
      led_control( 3 );
      delay( 255 );
      led_control( 1 );
      delay( 255 );
      delay( 128 );
      val -= 100;
      if( PIR1 & 0x20 ) return;        /* wwvb data available */
   }
    
   led_control( 0 ); 
   long_big_delay( 4 );

   while( val >= 10 ){
      led_control( 3 );
      delay( 255 );
      led_control( 2 );
      delay( 255 );
      delay( 128 );
      val -= 10;
      if( PIR1 & 0x20 ) return;        /* wwvb data available */
   }

   led_control(0); 
   long_big_delay( 4 );

   while( val  ){
      led_control( 1 );
      delay( 255 );
      led_control( 0 );
      delay( 255 );
      delay( 128 );
      val -= 1;
      if( PIR1 & 0x20 ) return;        /* wwvb data available */
   }
  led_control( 0 );
  long_big_delay( 8 ); 

}


/* big_delay - delay in 1/4 sec chunks - page1 so called long */
void long_big_delay(char sec4 ){

  while( sec4-- ){
    if( PIR1 & 0x20 ) break;        /* wwvb data available */
    delay( 255 );
  }

}


