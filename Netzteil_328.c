//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

//#include "twislave.c"
#include "lcd.c"

#include "adc.c"

#include "defines.h"



#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPSTEP			0x01FF

// Define fuer Slave:
#define LOOPLED			7

#define POTPIN			0
#define BUZZERPIN		0

#define OCR0A_TEMP 0x80;
#define OCR0A_STROM 0xE0;

#define TEMP_DELAY   550

#define BEEP_ONTIME  2
#define BEEP_OFFTIME  6
#define BEEP_OFFTIMEDELAY  24

volatile    	uint16_t loopcount0=0;
volatile       uint16_t loopcount1=0;

volatile    uint16_t timercount0=0;
volatile    uint16_t timercount1=0;
volatile    uint8_t beepcounter=0;
volatile    uint8_t beeptime=4; // 
volatile    uint8_t beepburstcounter=0; // 

volatile    uint8_t beep_ontime=2;
volatile    uint8_t beep_offtime=6;

volatile    uint8_t adccount0=0;
volatile    uint8_t blinkcount=0;
volatile  uint8_t pwmimpuls = 0;

volatile    uint8_t pwmpos=0;

volatile    uint16_t led_temp=0; // Eingang von Kuehlkoerper, sinkend
volatile    uint16_t stromreg = 0; // Eingang von Stromregelung, sinkend

volatile    uint8_t status=0;

void delay_ms(unsigned int ms);

void slaveinit(void)
{
   OUTDDR |= (1<<DDD0);		//Pin 0 von PORT D als Ausgang fuer Schalter: ON		
   
   OUTDDR |= (1<<BLINK_PIN);		//Pin 1 von PORT D als Ausgang fuer Schalter: OFF
   OUTPORT &= ~(1<<BLINK_PIN); // LO
   
   OUTDDR |= (1<<BEEP_PIN);		//Pin 2 von PORT D als Ausgang fuer Buzzer
   
   OUTDDR |= (1<<PWM_FAN_PIN);		//Pin 3 von PORT D als Ausgang fuer LED TWI
   OUTPORT &= ~(1<<PWM_FAN_PIN); // LO   
   
   OUTDDR |= (1<<OUT_OFF_PIN); // Aushang fuer Not-OFF
   OUTPORT &= ~(1<<OUT_OFF_PIN); // LO
   
   LOOPLEDDDR |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop
	LOOPLEDPORT |= (1<<LOOPLED);		//Pin 5 von PORT D als Ausgang fuer LED Loop


	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

   ADCDDR &= ~(1<<ADC_TEMP_PIN);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
   ADCDDR &= ~(1<<ADC_STROM_PIN);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
 //  ADCDDR &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
//   ADCDDR &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up

   OSZIDDR |= (1<<OSZIA);
}

void timer0 (void) 
{ 
// Timer fuer Exp
//	TCCR0 |= (1<<CS00)|(1<<CS02);	//Takt /1024
//	TCCR0 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256
   TCCR0A = 0;
   TCCR0A |= (1<<WGM01);
//Timer fuer Servo	
	TCCR0B |= (1<<CS01);	//Takt /64 Intervall 64 us
	
	//TIFR |= (1<<TOV0); 				//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK0 |= (1<<TOIE0);			//Overflow Interrupt aktivieren
   TIMSK0 |= (1<<OCIE0A);
   TCNT0 = 0x00;					//RŸcksetzen des Timers
   OCR0A = OCR0A_TEMP;
}


ISR(TIMER0_COMPA_vect)
{
   //OSZITOGG;
   //OUTPORT &= ~(1<<BEEP_PIN);
   if (status & (1<<BEEP_ON))
   {
      //OSZITOGG;
      OUTPORT ^= (1<<BEEP_PIN);
   }
}

ISR(TIMER0_OVF_vect)
{
  // OSZIHI;
   {
 //     OUTPORT ^= (1<<BLINK_PIN); // 
   }
   
}


// Timer2 fuer Takt der Messung
void timer2(void)
{
   //lcd_gotoxy(10,1);
	//lcd_puts("Tim2 ini\0");
   //PRR&=~(1<<PRTIM2); // write power reduction register to zero
   

//   TCCR2A |= (1<<WGM21);// Toggle OC2A
   TCCR2A |= (1<<COM2A1) | (1<<COM2A0) | (1<<WGM21) | (1<<WGM20);
 //  TCCR2B |= (1<<WGM22);
 //  TCCR2A |= (1<<WGM20);
 //  TCCR2A |= (1<<COM2A0);                  // CTC
   
   /*
    CS22	CS21	CS20	Description
    0    0     0     No clock source
    0    0     1     clk/1
    0    1     0     clk/8
    0    1     1     clk/32
    1    0     0     clk/64
    1    0     1     clk/128
    1    1     0     clk/256
    1    1     1     clk/1024
    */
   
   //TCCR2B |= (1<<CS22); //
   //TCCR2B |= (1<<CS21);//
   TCCR2B |= (1<<CS20) | (1<<CS21) |(1<<CS22)  ;
   
   TIMSK2 |= (1<<OCIE2A);      // CTC Interrupt En
   TIMSK2 |=(1<<TOIE2);        //interrupt on Compare Match A
	
   TIMSK2 |= (1<<TOIE2);						//Overflow Interrupt aktivieren
   TCNT2 = 0;                             //RŸcksetzen des Timers
	//OSZILO;
   OCR2A = TIMER2_COMPA; // 20ms
   
//   OCR2A = 0x02;
   
   //DDRB |= (1<<PORTB3);
   TIFR2 |= (1<<TOV2);							//Clear TOV0 Timer/Counter Overflow Flag. clear pending interrupts
}

#pragma mark TIMER2_COMPA
// Timer2 fuer Takt der Messung und Signal an Triac
ISR(TIMER2_COMPA_vect) // CTC Timer2
{
   //OSZITOGG;
   timercount0++;
   //OSZILO;
   OUTPORT &= ~(1<<PWM_FAN_PIN); // Triac off
   if (timercount0 > 5) // Takt teilen, 1s
   {
       //OSZITOGG;
      timercount0=0;
      
      if ((status & (1<<FAN_ON)) )
      {
         //OSZITOGG;
         if (beepburstcounter > 2)
         {
            beep_offtime = BEEP_OFFTIMEDELAY;
         }
         
         if (beepcounter == beep_offtime)
         {
            OSZITOGG;
            OUTPORT |= (1<<BLINK_PIN);
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            OUTPORT &= ~(1<<BLINK_PIN);
            status &= ~(1<<BEEP_ON);
         }
         beepcounter++;
      }
      else if ( (status & (1<<STROM_ON)))
      {
        
         if (beepburstcounter > 2)
         {
            beep_offtime = BEEP_OFFTIMEDELAY;
         }
         
         if (beepcounter == beep_offtime)
         {
            OSZITOGG;
            OUTPORT |= (1<<BLINK_PIN);
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            OUTPORT &= ~(1<<BLINK_PIN);
            status &= ~(1<<BEEP_ON);
            
         }
         beepcounter++;
      }
      else
      {
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
            beep_offtime = BEEP_OFFTIME;
         }
         beepcounter++;
      }
 
      timercount1++;
      
      status |= (1<<PWM_ADC);// ADC messen ausloesen
      
        
   }
}
//#pragma mark TIMER2_OVF
ISR(TIMER2_OVF_vect)
{
   OUTPORT |= (1<<PWM_FAN_PIN); // Triac on
}
   
   

void main (void) 
{
   wdt_reset();
    MCUSR=0;
    WDTCSR|=_BV(WDCE) | _BV(WDE);
   
    WDTCSR=0;
	slaveinit();
		
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	_delay_ms(200);
	lcd_cls();
	lcd_puts("READY\0");
	
   _delay_ms(1000);
	//timer0();
	uint16_t loopcount0=0;
   uint16_t loopcount1=0;

	uint8_t loopcount=0;

	_delay_ms(50);

	//lcd_clr_line(0);
   initADC(0);
   _delay_ms(50);
   timer0();
   _delay_ms(50);
   timer2();
   _delay_ms(50);
   sei();
   //status |= (1<<PWM_ADC);
//   status |= (1<<PID_FIRST_RUN); // K Prop ist beim Aufheizen kleiner
   #pragma mark while  
	while (1)
   {
      
      wdt_reset();
      //Blinkanzeige
      loopcount0++;
      //OSZITOGG;
      // Stromregelung checken
      
      stromreg = readKanal(ADC_STROM_PIN); 
      if (stromreg < STROM_MIN)
      {
         //OCR2A = TIMER2_COMPA_STROM;
         OCR0A = OCR0A_STROM;
         if (!(status & (1<<STROM_ON)))
         {
            status |= (1<<STROM_ON);
            beepcounter = BEEP_OFFTIME-1;
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
         }
      }
      else if (stromreg > STROM_MIN +1)
      {
         if ((status & (1<<STROM_ON)))
         {
            status &= ~(1<<STROM_ON);
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
            status &= ~(1<<BEEP_ON);
         }
      }
      
      if (led_temp < TEMP_MAX)
      {
         //OCR2A = TIMER2_COMPA_TEMP;
         OCR0A = OCR0A_TEMP;
         if (!(status & (1<<FAN_ON)))
         {
            status |= (1<<FAN_ON);
            beepcounter = BEEP_OFFTIME-1;
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
         }
         if (led_temp < TEMP_OFF)
         {
            OUTPORT |= (1<<OUT_OFF_PIN); // output OFF
         }
      }
      else if (led_temp > (TEMP_MAX + 1))
      {
         if ((status & (1<<FAN_ON)))
         {
            status &= ~(1<<FAN_ON);
            OUTPORT &= ~(1<<OUT_OFF_PIN); // Output wieder ON
            beepburstcounter = 0;
            beep_offtime = BEEP_OFFTIME;
            status &= ~(1<<BEEP_ON);
         }
      }
      
      
      
      if (status & (1<<PWM_ADC)) // ADC tempsensor lesen, beep einschalten 
      {
         
         status &= ~(1<<PWM_ADC);
         led_temp = readKanal(ADC_TEMP_PIN); 
         
         lcd_gotoxy(0,0);
         
         lcd_putc('T');
         lcd_putc(' ');
         lcd_putint12(led_temp);
         pwmimpuls = led_temp-TEMP_DELAY;
         lcd_putc(' ');
         lcd_putint12(pwmimpuls);
         
         OCR2A = pwmimpuls;

         lcd_gotoxy(0,1);
         lcd_putc('I');
         lcd_putint12(stromreg);
         lcd_putc(' ');
         lcd_putint(beepcounter);
         
         if ((status & (1<<FAN_ON))  || (status & (1<<STROM_ON)))
         {
            lcd_puts("F/S");
         }
         else
         {
            lcd_puts("    ");
         }
         
         //   status |= (1<<FAN_ON);
         lcd_gotoxy(16,0);
         lcd_putc('S');
         lcd_putint(status);
         
      }
      
      if (loopcount0>=LOOPSTEP)
      {
         
         loopcount0=0;
         LOOPLEDPORT ^=(1<<LOOPLED);
         loopcount1++;
       }
      //	LOOPLEDPORT &= ~(1<<LOOPLED);
   }//while


// return 0;
}
