
/* ********************

  wspr tx on solar power
    modified from the DDS version wspr_16f to use the Si5351 
  PIC16F690
  Si5351
  QRP LABS arduino shield
  5 volt NIMH battery
  Canaduino WWVB receiver

  A high side switch keeps the DDS off to reduce battery drain.  This causes frequency drift as it starts cold.
  The frequency is counted before and after the wspr transmission to adjust the drift compensation amount.
  For PIC16F690, master mode I2C must be done in firmware.
  The PIC UART is used at 10 baud for receiving the time signal.
  Timer2 is used to produce a 1ms interrupt for timing.

DDS powered by battery, tx powered by solar.
Reduced battery by 1 cell, now running on about 4 volts.  If the battery voltage gets too high, the unit runs the transmitter to use the
  extra stored power.

*********************** */

/* solar panel doesn't seem to put out 130 ma at 7 volts in weak sunlight ( or about 1 watt ) */


/*   Wiring
  Processor                  QRP Labs Shield I/O pin or other connection
11   RB6  spi clk                3     unused now
13   RB4  SCL                   A5  
 9   RC7  SDA                   A4
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
         removed and wired change control to ground, and a diode from 5 volt reg to +5
         Regulated input is always 5.6 and diode drops to 5.0 to charge battery.
         DDS no longer powered by battery but only via solar.
 5   RC5                        
 8   RC6                        Wired to high side output via 10k ( AN8 )
   (  RC7    used   for   spi  wired as above )

  Arduino shield power assignments
  Vin    5 volt battery side to DDS board
  +5     cut pin between boards, DDS has circuit to get reduced power from Vin
  3.3    Power to WWVB reciever using an output pin


*/

#include "p16f690.h"

/* dds and tx commands */
#define POWER_DOWN  0
#define POWER_UP    1
#define ENABLE 1
#define DISABLE 0

#define SI5351 0x60
#define CLK_TX   1
#define CLK_OFF  0
#define PLLA 26
#define PLLB 34
#define CLK0_DIV 45
#define CLK1_DIV 53
 

/*  flags bits */
#define WSPR_TICK 1
#define FRAME 2
/*#define VCC_CHECK 4
#define VIN_CHECK 8 */
#define PRE_TX 16
#define PRE_PRE 32

/* numbers are A/D readings of the internal 0.6v reference */
/* 3 cells or LiON, 4.1v == 137,  3.7v == 151 */
/* overcharge control is transmit as much as possible */
#define BAT_OK 151
#define BAT_HIGH 137 

/* port C single bit i/o */
#asm
LED0 equ 0
LED1 equ 1
WWVB_ON equ 2
DDS_PWR equ 3          ; inverse logic and high side switch is now dds power control
;BAT_CHARGE equ 4       ; inverse logic
#endasm

/* took quite some time to learn that VP6 is active when using the internal oscillator but needs to 
   be enabled in register VRCON when using a HS crystal  */

#asm
    ;  changed osc to external crystal
    __CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF & _HS_OSC & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF

  ;  __CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF &  _INTRC_OSC_NOCLKOUT & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF

#endasm

#define CAL 3       /* freq a bit low,  */
                    /* might be different for 30 vs 20 meters */

/* eeprom */
/*extern char EEPROM[2] = {0xff,0xff};     eeprom as one big array */
/*   dds load value per 1 hz for 125mhz xtal = 34.359738368 */
/*  20 meters solution , and each 1.4648 step adds 50 ( 50.33 ) */

extern char base20[8] = { 0x68, 0x2E, 0x00, 0x0C, 0x18, 0x00, 0x37, 0x42 };       /* 37,42 : 3B 42 */

/* wspr msg packed, last byte has just 2 entries, rest 4, message starts with LSBits */
/* extern char wspr_msg[41] = {
0xA7, 0xA8, 0xAB, 0xB5, 0x30, 0x44, 0x9F, 0x88, 0xBA, 0x44, 
0xA, 0x30, 0x27, 0x47, 0xE2, 0xB9, 0x40, 0x19, 0x39, 0xE1, 
0x30, 0x87, 0xBC, 0x1B, 0x9A, 0x80, 0x6B, 0xD8, 0x51, 0x70, 
0x2E, 0xD4, 0x8A, 0x4E, 0x72, 0xA, 0x68, 0xB1, 0x85, 0x36, 
0x8 }; */

extern char wspr_msg[162] = {
      3, 1, 2, 2, 0, 2, 2, 2, 3, 2, 2, 2, 1, 1, 3, 2, 0, 0, 3, 0, 0, 1, 0, 1, 3, 3, 1, 2, 0, 2,
      0, 2, 2, 2, 3, 2, 0, 1, 0, 1, 2, 2, 0, 0, 0, 0, 3, 0, 3, 1, 2, 0, 3, 1, 0, 1, 2, 0, 2, 3,
      1, 2, 3, 2, 0, 0, 0, 1, 1, 2, 1, 0, 1, 2, 3, 0, 1, 0, 2, 3, 0, 0, 3, 0, 3, 1, 0, 2, 0, 3,
      3, 2, 3, 2, 1, 0, 2, 2, 1, 2, 0, 0, 0, 2, 3, 2, 2, 1, 0, 2, 1, 3, 1, 0, 1, 1, 0, 0, 3, 1,
      2, 3, 2, 0, 0, 1, 1, 3, 2, 2, 0, 2, 2, 3, 0, 1, 2, 0, 3, 1, 2, 2, 0, 0, 0, 2, 2, 1, 1, 0,
      3, 2, 1, 1, 0, 2, 2, 1, 3, 0, 0, 2
   };
extern char slots[6] = { 2, 10, 18, 28, 38, 48 };     /* xmit on these minutes, save 50-60 for wwvb rx */
extern char EE_DRIFT = 1;


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
/* char vin_ok;*/
char acc0,acc1,acc2,acc3;   /* 32 bit working registers */
char arg0,arg1,arg2,arg3;    
/* at 24 */
/*char charging;*/
char drift;
char drift_l;
char drift_h;
char trans_count;           /* tx's since wwvb match */
char calibrate;

/* bank 1 ram - 80 bytes */
#pragma pic  160
char pre_tx;
char post_tx;
char tmr0_pre_L;       /* prescaler value */
char tmr0_pre_H;       /* timer value */
char tmr0_post_L;
char tmr0_post_H;
char save_cal;
char save_cal_L;
char solution[8];


/* bank 2 ram - no arrays - 80 locations - function args and statics are here */
#pragma pic 288
char total_nacks;


/*  ********
#pragma shortcall   - use for functions that do not call other pages
#pragma longcall    - use for functions that call other pages
                    - do not allow code to pass over the page boundary
********  */

#pragma shortcall

main(){       /* any page */

for(;;){    /* loop main needed */
static char i;
static char pre_pre;




   if( flags & PRE_PRE ){            /* longer warmup than pre_tx in dds power down mode ( no tx but osc is on ) */
      #asm        
        bcf flags, 5 
      #endasm
      pre_pre = 0;
      dv = min + 1;
      for( i = 0; i < 6; ++i ) if( slots[i] == dv ) pre_pre = 1;
      check_bat();
      if( vcc < BAT_HIGH ) pre_pre = 1;         /* avoid overcharge by transmitting ( inverse, lower number is higher voltage ) */
      if( pre_pre ){
         dv = 0;
         if( read_5_volts() == 0 ) pre_pre = 0;    /* check for solar power, high result ( 2 bits ) */
         else dv = 2;
         if( check_bat() == 0 ) pre_pre = 0;
         else dv += 1;
         led_control( dv );                /* led's,  5volt ok , bat ok */
         if( transmitting ) pre_pre = 0;
         if( pre_pre ) dds_on();
       /*  short_blink( vcc );    */       /* !!! debug only */
      }
   }

   if( flags & PRE_TX ){     /* get a starting freq count */
      #asm        
        bcf flags, 4 
      #endasm
      pre_tx = pre_pre;  pre_pre = 0;
    
      if( pre_tx ){
         load_base();
         arg3 = arg2 = arg0 = 0;      /* warm up dds */
         arg1 = calibrate ;            /* calibrate off freq needed?  */
         dadd();
         clock_dds( POWER_UP );
         if( trans_count != 255 ) ++trans_count;   /* tx's since wwvb match */      
      }
      else led_control( 0 );             /* turn off the led's after 2 min if no tx */
   }

   if( flags & FRAME ){
      #asm
       bcf flags,1
      #endasm
      if( pre_tx ){
         transmitting = 1;
        /* clock_dds( POWER_UP ); */
      }
   }

   if( flags & WSPR_TICK ){
      #asm
        bcf flags,0
      #endasm
      if( transmitting == 1 ) transmitting = 2;           /* skip one tick at the start */
      else if( transmitting == 2 ) wspr_tx( ENABLE );
      else if( pre_tx ){
         if( ++pre_tx == 8 ) freq_count = 1, post_tx = 0;      /* this might want to be 8 since we now skip a tick at the start */
      }
      else if( post_tx ){
         if( ++post_tx == 5 ) freq_count = 1, pre_tx = 0;
      }
   }

   if( PIR1 & 0x20 ) wwvb_rx();

  /* adjustment of drift compensation */
   if( freq_count == 3 ){      /* get frequency count */
      freq_count = 0;
                               /* read result and save pre or post tx */      
      if( pre_tx ){
         clock_dds( POWER_DOWN );
         save_cal = tmr0_pre_H = TMR0;
         save_cal_L = tmr0_pre_L = read_prescaler();
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
      /*   if( tmr0_post_L != 255 ) ++tmr0_post_L;  */

         if( tmr0_post_L > tmr0_pre_L && drift != 0 ) --drift;
         if( tmr0_post_L < tmr0_pre_L ) ++drift;
        /* short_blink( drift ); */
       /*  short_blink( tmr0_post_L ); */

         /* long term drift compensation, frequency seems to measure about 700 hz low for some reason */
         /* is the 8 mhz xtal a bit fast ? */
         if( save_cal == 0x17 ){      /* expected 0x1a */
            if( save_cal_L < 128 && calibrate < 20 ) calibrate += 1;
            if( save_cal_L > 162 && calibrate > 1 ) calibrate -= 1;
         }
      
       /*  short_blink( save_cal ); */
       /*  if( save_cal == 0x17 ){
            short_bell();
            delay( 255 );
            delay( 255 );
            short_blink( save_cal_L );
         } */
              
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


char read_5_volts(){

   ADCON0 = 0xA0;     /* AN8 selected */
   ADCON0 |= 1;       /* module on */
   small_delay(120);
   ADCON0 |= 2;       /* GO */
   while( ADCON0 & 2 );  /* wait not done */
   ADCON0 = 0xA0;     /* module off but selecting correct channel */

   return ADRESH;     /* just upper 2 bits */
}

void led_control( char t ){

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
   produce a 0,1 or sync only which perhaps is not always correct.  Received signal edges seem pre-defined.
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



char read_vp6(){

   ADCON0 = 0xb4;     /* VP6 selected, right justified result */
   ADCON0 |= 1;       /* module on */
   small_delay(120);
   ADCON0 |= 2;       /* GO */
   while( ADCON0 & 2 );  /* wait not done */
   ADCON0 = 0xb4;     /* module off but selecting correct channel */

   return ADRESL; 
}

char check_bat(){            /* get a reading of the battery level */

      /* read internal 0.6 volts with reference as VDD  */
      /* needed to turn on vp6 when using external osc */

   vcc = read_vp6();

  /* short_blink( vcc ); */   /* !!! debug */
  /* delay( 255 );   */        

/* changed to 4 volt operation */
   if( vcc > BAT_OK ) bat_ok = 0;     /* new limits for 5 volt,  was 168,163 for 4 volt */
   else bat_ok = 1;

   return bat_ok;

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





/* keeping the ( dds ) control signals at zero volts to avoid latchup as the dds power is turned on and off */
/* si5351 with pullups on SDA SCL, SDA and SCL should become high as si5351 powers up */
void dds_on(){

   I2init();
   /* power up */
   #asm
    banksel PORTC
   ; bcf  PORTC, BAT_CHARGE     ;  enable low, turn on more power
    bcf  PORTC, DDS_PWR        ;  high side switch on
   #endasm

   delay( 40 );     /* high side switch is slow soft start, Si5351 startup 20 + 20 */
   si5351_init();
   
   clock_dds( POWER_DOWN );
   write_divider( 50 );
}

void dds_off(){

   
   #asm
    banksel PORTC
   ; bcf PORTC, RC7       ; sdo low
    bsf PORTC, DDS_PWR   ; high side switch off
   #endasm


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
        arg1 = calibrate;
        dadd();
        arg1 = drift_h;                /* add drift compensation */
        arg0 = drift_l;
        #asm
         ASRD arg1,arg0        ; shift out 3 ( was 2 ) fractional bits
         ASRD arg1,arg0
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

   var = wspr_msg[txcount];
   led_control( var );
   ++txcount;
   arg0 = 0;
   while( var-- ) arg0 += 10;       /* wspr offset */
   load_base();
   arg3 = arg2 = 0;
   arg1 = calibrate;
   dadd();
   arg1 = drift_h;                 /* drift compensation */
   arg0 = drift_l;
   #asm
    ASRD arg1,arg0        ; shift out 3 fractional bits
    ASRD arg1,arg0
    ASRD arg1,arg0
   #endasm
   dadd();
   clock_dds( POWER_UP );

   /* calc new drift for next symbol tx */
   zacc();
   arg3 = arg2 = arg1 = 0;
   arg0 = drift;
  /* try secondary curve with less drift compensation near end of transmission */
  /* if( txcount < 42 ) ++arg0; */
   acc1 = drift_h;
   acc0 = drift_l;
   dadd();
   drift_h = acc1;
   drift_l = acc0;

}

void load_base(){
static char i;

   for( i = 0; i < 8; ++i ){
      dv = base20[i];
      solution[i] = dv;
   }

   zacc();
   acc0 = solution[7];
   acc1 = solution[6];

}



clock_dds( char powered ){
 
    solution[7] = acc0;
    solution[6] = acc1;
    adjust_solution();
    write_solution( PLLB );
    clock( powered );           /* clock enables and powered have same definitions so powered == enable tx */
}


void adjust_solution(){        /* check for overflow of P2 term and fix */

   zarg();                                    /* check if P2 is larger than P3 */
   arg0 = solution[1];  arg1 = solution[0];   /* load P3 */
   while( dsub() == 0 ) ++solution[4];        /* sub until no borrow */
   dadd();                                    /* add back on borrow */
   solution[6] = acc1;  solution[7] = acc0;

}


 
void zacc(){     /* zero the accumulator */
   
   acc3 = acc2 = acc1 = acc0 = 0;

}

void zarg(){

   arg3 = arg2 = arg1 = arg0 = 0;
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

   dv = 0;             /* a variable in access ram */
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
     btfss   STATUS,C
     incf    dv,F

     movf    arg1,W
     subwf   acc1,F
     movlw   1
     btfss   STATUS,C
     subwf   acc2,F
     btfss   STATUS,C
     subwf   acc3,F
     btfss   STATUS,C
     incf    dv,F

     movf    arg2,W
     subwf   acc2,F
     movlw   1
     btfss   STATUS,C
     subwf   acc3,F
     btfss   STATUS,C
     incf    dv,F

     movf    arg3,W
     subwf   acc3,F
     btfss   STATUS,C
     incf    dv,F
     
   #endasm

   return dv;

}



/***   PIC 18F has much better carry instructions *******
char dsub(){  /* sub the arg from the accum, return borrow 

   #asm
     movf    arg0,W
     subwf   acc0,F
     movf    arg1,W
     subwfb  acc1,F
     movf    arg2,W
     subwfb  acc2,F
     movf    arg3,W
     subwfb  acc3,F
   #endasm

   return (( STATUS & 1 ) ^ 1 );       /* return borrow as a true value 
}  ****************/


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



/***************   I2C functions  Si5351 functions *****************/

#define SDA_LOW TRISC = TRISC & ( 0xff ^ 0x80 )
#define SDA_HIGH TRISC = TRISC | 0x80
#define SCL_HIGH TRISB = TRISB | 0x10
#define SCL_LOW  TRISB = TRISB & (0xff ^ 0x10)

void I2init(){
   
   SDA_HIGH;
   SCL_HIGH;

}

void I2start(){

#asm
  banksel PORTB
  bcf PORTB,RB4
  bcf PORTC,RC7
 ; banksel TRISB
#endasm

    SDA_LOW;
    SCL_LOW;
}

void I2stop(){

    SDA_LOW;
    SCL_HIGH;
    SDA_HIGH;
}

char I2send( char dat ){
static char nack;
static char i;

    for( i = 0; i < 8; ++i ){
       if( dat & 0x80 ) SDA_HIGH;
       else SDA_LOW;
       SCL_HIGH;
       SCL_LOW;
       dat = dat << 1;
    }
    SDA_HIGH;
    SCL_HIGH;
    nack = PORTC & 0x80;
    SCL_LOW;
    if( nack ) ++total_nacks;
    return nack;     /* high is nack, low is ack */

}


char si_adr;

void si5351_init(){

   clock(CLK_OFF);

   for( si_adr = 16; si_adr < 24; ++si_adr ) si_write( 0x80 );     /* power down all outputs */

   for( si_adr = 42; si_adr <= 49; ++si_adr ){
      si_write(  0x00 );     /* dividers that don't change */
      si_adr += 8;
      si_write( 0x00);
      si_adr += 8;
      si_write( 0x00 );
      si_adr -= 16;
   }
   si_adr = 43;
   si_write( 1 );      /* a + b/c  c = 1 */
   si_adr = 43+8;
   si_write(  1 );
   si_adr = 43+16;
   si_write( 1 );
   si_adr = 16;
   si_write( 0x6d );    /* clock 0 assigned to pllb with x ma drive,  c d e f == 2 4 6 8 ma drive */
 /****  ++si_adr;
   si_write( 0x4c );    /* clock 1 assigned to plla with 2ma drive 
   ++si_adr;
   si_write( 0x4c );    /* clock 2 ? - unused for this radio *****/
}

void si_write( /* char reg, */  char val){          /* single si5351 write register */ 

   I2start();
   I2send( SI5351 << 1 );
   I2send( si_adr );
   I2send( val );
   I2stop( );
}

void clock( char val ){            /* control si5351 clocks, CLK_RX, CLK_TX, CLK_OFF */

   si_adr = 3;
   si_write(/* 3,*/ val ^ 0xff );
}



void write_divider( /*char address,*/ char div ){    

  /* 128 * val - 512, same as (val-4) * 128 */
  /* div = ( div - 4 ) >> 1;        mult by 256 and divide by 2 == mult by 128, here is divide by 2 */
  /* si_write(  address, div );        write high byte is mult by 256, dividers are even, low byte is always zero */  
   si_adr = CLK0_DIV;

   si_write( (div-4) >> 1 );   /* all preceding in one command */

   si_adr = 177;
   si_write( /* 177, */ 0xA0 );                   /* reset pll's */
  /* delay(1); */
  /* si_write(  177, 0xA0 );  */                 /* double reset needed ? */
}


/*  pass the PLLA or PLLB to this function also */
void write_solution( char pllx ){                  /* loads the PLL information packet */
static char k;

   I2start();
   I2send( SI5351 << 1 );
   I2send(pllx);
   for( k = 0; k < 8; ++k ) I2send( solution[k] );
   I2stop();
}


/***************************************************/



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

   if( ++ms_L == 0 ) ++ms_H;
   if( ms_L == 0xe8 && ms_H == 0x3 ){    /* one second */
      ms_L = 0;
      ms_H = 0;

/* freq count */
      if( freq_count == 1 ){        /* start count */
         #asm
           nop
           nop
           nop
           nop
           nop
           nop
           nop
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

      if( sec == 50 && ( min & 1 ) ) flags |= PRE_TX;
      if( sec == 10 && ( min & 1 ) ) flags |= PRE_PRE;
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
 /* bat_ok = vin_ok = 1; */
  transmitting = 0;
  flags = 0;
  txcount = 0;
 /* charging = 0; */
  drift = EE_DRIFT;
  drift_h = 0;
  drift_l = 0;
  pre_tx = post_tx = 0;
  calibrate = CAL;

/* init pins */


  /* OPTION_REG = 0b11100111; radix binary doesn't work with this compiler !! */
   OPTION_REG = 0xe7;
   #asm
     banksel PORTA
     bsf PORTA, RA2     ; high when enabled for stopping freq count, input for now
   #endasm

   ANSEL = 0;
   ANSELH = 1;            /* AN8 as analog input */
   TRISC = 0xc0;          /*  SDO as output, 01000000 rc6 as analog, dds_pwr, charge, wwvb, led's as outputs, rc7 sda input */
   PORTC = 0x18;          /*  wwvb off, led's off, dds_pwr off, charge off  */
 /* PORTC = 0x1c; */       /* turn wwvb rx on for testing */
  
   PORTB = 0;
   TRISB = 0xbf;      /* 0b10111111, rb6 as spi clk, RB4 works as an input SCL */

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

/* set A/D to read high side switch out, solar power check */
   ADCON1= 0x20;         /* 8/32  clock */
   ADCON0= 0xA0;         /* set to read AN8, right justified result */
   VRCON = 0x10;         /* turn on vp6 */

   I2init();


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


