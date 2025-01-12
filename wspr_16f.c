
/* 16F690 shell for C */

/*   Wiring

*/

#include "p16f690.h"

#define POWER_DOWN  0x4
/*  flags bits */
#define WSPR_TICK 1
#define FRAME 2



#asm
    ;  will want to change osc to external crystal
    __CONFIG    _CP_OFF & _CPD_OFF & _BOR_OFF & _PWRTE_ON & _WDT_OFF & _INTRC_OSC_NOCLKOUT & _MCLRE_ON & _FCMEN_OFF & _IESO_OFF

#endasm



/* eeprom */
/*extern char EEPROM[2] = {0xff,0xff};     eeprom as one big array */
/*   dds load value per 1 hz for 125mhz xtal = 34.359738368 */
/*  20 meters solution , and each 1.4648 step adds 50 ( 50.33 ) */
extern char base20[4] = { 0x1c, 0xde, 0xf0, 0xbc };

/* wspr msg packed, last byte has just 2 entries, rest 4, message starts with LSBits */
extern char wspr_msg[41] = {
0xA7, 0xA8, 0xAB, 0xB5, 0x30, 0x44, 0x9F, 0x88, 0xBA, 0x44, 
0xA, 0x30, 0x27, 0x47, 0xE2, 0xB9, 0x40, 0x19, 0x39, 0xE1, 
0x30, 0x87, 0xBC, 0x1B, 0x9A, 0x80, 0x6B, 0xD8, 0x51, 0x70, 
0x2E, 0xD4, 0x8A, 0x4E, 0x72, 0xA, 0x68, 0xB1, 0x85, 0x36, 
0x8 };
extern char slots[6] = { 4, 8, 16, 32, 42, 54 };     /* xmit on these minutes */



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



/* bank 0 ram - 80 locations */
/* any arrays need to be in page 0 or page 1 as the IRP bit is not updated */
#pragma pic  32
char dv;                    /* dummy variable for when need one */
char transmitting;
char  wwvb_data[11];

/* bank 1 ram - 80 bytes - use may speed code using bank1 registers when writing SFR's */
#pragma pic  160
char acc0,acc1,acc2,acc3;   /* 32 bit working registers */
char arg0,arg1,arg2,arg3;    



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
      no_interrupts();
      #asm
       bcf flags,1
      #endasm
      interrupts();
    /*  PORTC ^= 2; */
      for( i = 0; i < 6; ++i ) if( slots[i] == min ) transmitting = 1;
   }

   if( flags & WSPR_TICK ){
      no_interrupts();
      #asm
        bcf flags,0     ; are these atomic anyway 
      #endasm
      interrupts();
   if( min < 4 )   PORTC ^= 1;
      if( transmitting == 1 ) wspr_tx( 1 );
   }

   if( PIE1 & 0x20 ) wwvb_rx();


}
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
    bcf PIE1, RCIE
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
      }

   }

/* !!! LED's on low pin count board */
   if( dat == 0xff ) PORTC = 0x8;
   else if( dat == SYNC ) PORTC = 0x4;
   else if( dat == ONE )  PORTC = 0x2;
   else if( dat == ZERO ) PORTC = 0x1;
   else PORTC = 0;

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
static char i;

   if( var == 0 ){
      b = c = 0;
      clock_dds( POWER_DOWN );
      transmitting = 0;
      return;
   }
   if( c == 40 && b == 2 ){      /* end message, recursive call ok? */
      wspr_tx( 0 );
      return;
   }

   var = wspr_msg[c];
   for( i = 0; i < b; ++i ) var >>= 2;
   var &= 3;

   if( ++b == 4 ) b = 0, ++c;
   arg0 = 0;
   while( var-- ) arg0 += 50;       /* wspr offset */
   load_base();
   arg3 = arg2 = arg1 = 0;
   dadd();
   clock_dds( 0 ); 

}

void load_base(){

   acc3 = base20[0];
   acc2 = base20[1];
   acc1 = base20[2];
   acc0 = base20[3];

}


spi_send( char dat ){

 /*  while( (SSPSTAT & 1) == 0 ); hangs here */  /* previous xfer is complete */

   /* just need 8 instructions between calls, bitrev has at least 8 */
   SSPBUF = dat;

}

clock_dds( char power_down ){
   /* data to load is in the accumulator */

   spi_send(bitrev(acc0));    /* lsb bytes and bits first */
   spi_send(bitrev(acc1));
   spi_send(bitrev(acc2));
   spi_send(bitrev(acc3));
   spi_send(bitrev(power_down));      /* zero normal, or power down bit */

   /*  toggle load pin */
   #asm
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
}


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

no_interrupts(){

   #asm
     bcf   INTCON,GIE
     btfsc INTCON,GIE    ;see AN576.  What devices have this issue?
     goto $-2
   #endasm
}


#pragma longcall

/* can have stub functions here to call into page1 */

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
      if( ++sec == 60 ){
          sec = 0; ++min;
          if( min == 60 ) min = 0;
          if( (min & 1) == 0 ){         /* reset wspr tick at start of 2 min intervals */
            wspr_tick_L = 0xaa;         /* set to end next tick below */
            wspr_tick_H = 2;
            flags |= FRAME;
          }
      }
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
  sec = 0;                          /* 0 - 120 */
  min = 0;                          /* counts by 2 or call this frame count */
  wspr_tick_L = 0;                  /* wspr timing 683ms 683ms 682ms  0x2ab */
  wspr_tick_H = 0;

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
/* is there a DDS reset pin or is it tied low or high */

  /* RB6 is clock, trying SDI RB4 as the load pulse */
   PORTB = 0xff - 0x40 - 0x10;     /* clock low */
   PORTB = 0xff - 0x10;            /* clock high */
   PORTB = 0xff;                   /* load pulse */
   PORTB = 0xff - 0x10;        
   PORTB = 0xff - 0x40 - 0x10;     /* clock low */
   PORTB = 0xff - 0x10;            /* clock high */
   PORTB = 0xff;                   /* load pulse */
   PORTB = 0xff - 0x10;        

/* set up SPI */
   SSPSTAT = 0x00;      /* default is zero, different clock to data timing with CKE 0x40 set? */
   SSPCON  = 0x30;      /* need clock idle high for correct data timing to clock rising edge */
  /* SSPBUF  = 0;  */       /* dummy write to set buffer full flag, !!! didn't work */

/* timer2 for a millis interrupt */
   PR2 = 124;           /* 2000000 / 16 / 125 = 1000.  one less count needed ?  124 or 125 */
   T2CON = 6;           /* on and 16:1 prescale */
   PIE1 = 2; 
   interrupts();        /* INTCON = 0xc0; */
  /* interrupt will clear TMR2IF in PIR1 */

   wspr_tx( 0 );                   /* power down dds */

/* set up uart for 10 baud recieve - wwvb */
   TXSTA = 0;           /* will tx pin be available for i/o if tx not enabled ? */
   BAUDCTL = 0x08;      /* brg16 */
   SPBRGH = 0xc3;
   SPBRG = 0x4d;        /* c350 -1 calc value :  or a little lower if framing errors alot */  
   RCSTA = 0x90;        /* note: clear cren when errors */


}

