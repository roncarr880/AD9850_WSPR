
/* 16F690   wspr tx on solar power */

/*   Wiring
  Processor                  QRP Labs Shield I/O pin or other connection
11   RB6  spi clk                3
13   RB4  fq_updt               A5     ( sdi does work as an output )
 9   RC7  spi data              A4
12   RB5  RX                    Canaduino wwvb rx signal

 4   RA3                        VPP programming
 3   RA4                        xtal osc
 2   RA5                        xtal osc
19   RA0                        isp data
18   RA1                        isp clock
 1   VDD                        VCC
20   VSS                        Ground

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



#asm
    ;  will want to change osc to external crystal
    __CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF

#endasm

#define CAL 10      /* freq a bit low */

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
extern char slots[6] = { 4, 8, 16, 32, 40, 48 };     /* xmit on these minutes, save 50-60 for wwvb rx */



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
/*char led_timer;*/



/* bank 0 ram - 80 locations */
/* any arrays need to be in page 0 or page 1 as the IRP bit is not updated */
#pragma pic  32
char dv;                    /* dummy variable for when need one */
char transmitting;
char  wwvb_data[11];
char vcc;
char bat_ok;
char vin_ok;
char acc0,acc1,acc2,acc3;   /* 32 bit working registers */
char arg0,arg1,arg2,arg3;    
/* at 24 */

/* bank 1 ram - 80 bytes - use may speed code using bank1 registers when writing SFR's */
#pragma pic  160



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



   if( flags & FRAME ){
      #asm
       bcf flags,1
      #endasm

    /*  PORTC ^= 2; */
      for( i = 0; i < 6; ++i ) if( slots[i] == min ) transmitting = 1;
      /* if bat_ok == 0 || vin not ok then transmitting = 0; */
   }

   if( flags & WSPR_TICK ){
      #asm
        bcf flags,0     ; are these atomic anyway 
      #endasm

  /* if( min < 4 )   PORTC ^= 1;     */ /* !!! debug is it running ? */
      if( transmitting == 1 ) wspr_tx( ENABLE );
   }

   if( PIR1 & 0x20 ) wwvb_rx();    /* !!! check if nighttime also, vin not on */

   /* voltage checks. Control charging, transmitting and when wwvb rx is on */
   if( flags & VIN_CHECK ) check_vin();
   if( flags & VCC_CHECK ) check_bat();


}
}

/* could add a fet and test this with a digital pin, low is ok */
void check_vin(){

    #asm
     bcf flags, 3
    #endasm

   vin_ok = 1;

}

void check_bat(){            /* get a reading of the battery level */
static char charging;

    #asm
     bcf flags, 2
    #endasm

 /* !!! turn off high pass switch */

      /* read internal 0.6 volts with reference as VDD */
/*   voltage break points as displayed determined by experiment
    143 = 4.0
    147 = 3.9   raw calculation of 0.6/4.0 * 1024 would have 4.0 volts as 154 
    151 = 3.8    which is quite far off from the measured values
    155 = 3.7
    159 = 3.6
    163 = 3.5
    167 = 3.4
*/
   ADCON0 = 0xb4;
   ADCON0 |= 1;      /* module on */
   short_delay(40);  /* reads one step lower than ms delay */
   ADCON0 |= 2;      /* GO */

   while( ADCON0 & 2 );  /* wait not done */

   vcc= ADRESL;  /* result has inverse relationship with vdd */

   ADCON0= 0xb4;  /*  !!! select Vin channel instead module off but selecting correct channel */
   if( vcc > 168 ) bat_ok = 0;
   if( vcc < 163 ) bat_ok = 1;
/*
   if vin is ok and vcc >= 150 start charging  = 1
   if vin is not ok or vcc <= 140 stop charging = 0

   if( charging == 1 ) turn on high pass switch
   else turn off high pass switch

*/


   /* !!! LED display */

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

void short_delay( char count ){

   while( count-- );

}

#define SYNC 0x80
#define ZERO 0xfe
#define ONE  0xf0

void wwvb_rx(){
static char dat;
static char i;
static char ok;
static char t;
static char wmin;


   #asm
    banksel PIR1
    bcf PIR1, RCIE
   #endasm

   dat = RCREG;
   if( RCSTA & 0x6 ){      /* errors */
      #asm
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

   /* match first 11 of wwvb data frame */
   if( wwvb_data[0] == SYNC && wwvb_data[1] == SYNC && wwvb_data[10] == SYNC && wwvb_data[5] == ZERO ){

      ok = 1;
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

      if( ok == 1 ){            /* a match, should be at 10 seconds */     
          no_interrupts();
           sec = 10;
           min = wmin;
          interrupts();
          short_bell();       
      }

   }

/* !!! LED's on low pin count board */
   if( dat == 0xff ) PORTC = 0x8;
   else if( dat == SYNC ) PORTC = 0x4;
   else if( dat == ONE )  PORTC = 0x2;
   else if( dat == ZERO ) PORTC = 0x1;
   else PORTC = 7;
  /* led_timer = 0; */

}


void delay( char count ){           /* delay up to 255 ms */
static char t;

   while( count-- ){
      t = ms_L;
      while( t == ms_L );
   }
}


void wspr_tx( char var ){
static char b;              /* di bits sent */
static char c;              /* char position */
static char count;     

   if( var == DISABLE ){
      b = c = 0;
      count = 0;
      clock_dds( POWER_DOWN );
      transmitting = 0;
      return;
   }
   if( count == 162 ){      /* end message, recursive call ok? */
      wspr_tx( DISABLE );
      return;
   }

   b = count & 3;
   c = count >> 2;
   var = wspr_msg[c];
   while( b-- ) var >>= 2;
   var &= 3;
   /* PORTC = var;   */               /* should stop on 2 for last of msg */
   ++count;
   arg0 = 0;
   while( var-- ) arg0 += 50;       /* wspr offset */
   load_base();
   arg3 = arg2 = 0;
   arg1 = CAL;                     /* freq calibration */
   dadd();
   arg1 = 0;                       /* drift compensation */
   arg0 = count;
   arg0 += ( count >> 2 );         /* not to exceed 255  about count + count>>1 */
   dadd();
   clock_dds( POWER_UP ); 

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
   short_delay(1);           /* spi clocks not done yet */
   #asm
     banksel PORTB
     bsf PORTB, RB4
     nop
     bcf PORTB, RB4
   #endasm

  /*   PORTC ^= 8;  */ /* !!! debug only light LED on low pin count board */ 
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

/*
   short( 16 entries ) table lookup method, maybe would be 14 instructions + call / return
      needs source and dest in access ram
   above is 16 + save/load in C using some banksels + call/return

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
}


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

  /* !!! should these functions return CARRY */

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
/* if( ++led_timer = 255 ) PORTC = 0; */
   if( ms_L == 0xe8 && ms_H == 0x3 ){    /* one second */
      ms_L = 0;
      ms_H = 0;
      if( ++sec == 60 ){
          sec = 0; ++min;
          if( min == 60 ) min = 0;      /* !!! and turn off wwvb rx */
          if( min == 50 ) ;             /* !!! turn on wwvb rx */
          if( (min & 1) == 0 ){         /* reset wspr tick at start of 2 min intervals */
            wspr_tick_L = 0xaa;         /* set to end next tick below */
            wspr_tick_H = 2;
            flags |= FRAME;
          }
      }
      if( sec == 29 ) flags |= VIN_CHECK;
      if( sec == 30 ) flags |= VCC_CHECK;
   }

   if( ++wspr_tick_L == 0 ) ++wspr_tick_H;
   if( wspr_tick_L == 0xab && wspr_tick_H == 0x2 ){
      flags |= WSPR_TICK;
      wspr_tick_L = 0;
      wspr_tick_H = 0;
      if( ++mod > 2 ){           /* gen timing 683 683 682 .... */
         mod = 0;
         ++wspr_tick_L;
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
   OSCCON= 0x70;   /* 60 is 4 meg internal clock or 70 for 8 meg */
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

/* init pins */

    /*  ANSEL = 0x04;     /* one analog pin RA2 */
    /*  TRISA TRISB */
   ANSEL = 0;
   ANSELH = 0;
   TRISC = 0x70;          /* !!!for low pin count demo board + SDO as output */
   PORTC = 0;
   PORTB = 0xff - 0x10;
   TRISB = 0b10101111;    /* for rb6 as spi clk, and will RB4 work as an output in spi mode? */

/* init DDS, clock and load to get into serial mode, twice just in case */
/* RB6 is clock, trying SDI RB4 as the load pulse */

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

/* timer2 for a millis interrupt */
   PR2 = 124;           /* 2000000 / 16 / 125 = 1000.  one less count needed ?  124 or 125 */
   T2CON = 6;           /* on and 16:1 prescale */
   PIE1 = 2; 
   INTCON = 0xc0;
  /* interrupt to clear TMR2IF in PIR1 */

   wspr_tx( DISABLE );  /* power down dds, and init the static vars in wspr_tx */

/* set up uart for 10 baud recieve - wwvb */
   TXSTA = 0x0;           /* will tx pin be available for i/o if tx not enabled ?, or 0x20 */
   BAUDCTL = 0x08;      /* brg16 */
   SPBRGH = 0xc3;
   SPBRG = 0x4d;        /* c350 -1 calc value :  or a little lower if framing errors alot */  
   RCSTA = 0x90;        /* note: clear cren when errors */

/* set A/D to read internal reference for battery check */
   ADCON1= 0x20;         /* 8/32  clock */
   ADCON0= 0xB4;         /* set to read internal 0.6 volts */


}

void long_bell(){     /* flash lights on low pin count board when wwvb match, !!! debug only function */
                      /* this function is not a good candidate for page2 as it calls delay in page1 */

   PORTC = 0xf;
   delay(100);
   PORTC = 0;
   delay(100);

   PORTC = 0xf;
   delay(100);
   PORTC = 0;
   delay(100);

   PORTC = 0xf;
   delay(100);
   PORTC = 0;
   delay(100);

   PORTC = 0xf;
   delay(100);
   PORTC = 0;
   delay(1);

}


