//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//



#include <avr/io.h>
#include <avr/delay.h>
//#include <avr/interrupt.h>
//#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>

//#include "twislave.c"
#include "lcd.c"

#include "adc.c"
#include "onewire.c"
#include "ds18x20.c"
#include "crc8.c"
//#include "uart328.c"

#include "defines.h"



#define LOOPLEDPORT		PORTD
#define LOOPLEDDDR		DDRD
#define LOOPSTEP			0x01FF

// Define fuer Slave:
#define LOOPLED			7
//#define TWILED			5

// Define fuer mySlave PORTD:
//#define LOOPLED			2
//#define TWILED			7


#define TASTE1		38
#define TASTE2		46
#define TASTE3		54
#define TASTE4		72
#define TASTE5		95
#define TASTE6		115
#define TASTE7		155
#define TASTE8		186
#define TASTE9		205
#define TASTEL		225
#define TASTE0		235
#define TASTER		245
#define TASTATURPORT PORTC

#define TASTATURPIN		3
#define POTPIN			0
#define BUZZERPIN		0



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


volatile    uint8_t pwmpos=0;

volatile    uint16_t led_temp=0; // Eingang von Kuehlkoerper, sinkend
volatile    uint16_t stromreg = 0; // Eingang von Stromregelung, sinkend



volatile    uint8_t lastwert=0;
volatile    int16_t fehler=0;
volatile    int16_t lastfehler=0;
volatile    int16_t fehlersumme=0;

volatile    double stellwert=200.0;
volatile    uint8_t status=0;





void delay_ms(unsigned int ms);



uint8_t Tastenwahl(uint8_t Tastaturwert)
{
if (Tastaturwert < TASTE1)
return 1;
if (Tastaturwert < TASTE2)
return 2;
if (Tastaturwert < TASTE3)
return 3;
if (Tastaturwert < TASTE4)
return 4;
if (Tastaturwert < TASTE5)
return 5;
if (Tastaturwert < TASTE6)
return 6;
if (Tastaturwert < TASTE7)
return 7;
if (Tastaturwert < TASTE8)
return 8;
if (Tastaturwert < TASTE9)
return 9;
if (Tastaturwert < TASTEL)
return 10;
if (Tastaturwert < TASTE0)
return 0;
if (Tastaturwert < TASTER)
return 12;

return -1;
}



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

	/*
	DDRB &= ~(1<<PB0);	//Bit 0 von PORT B als Eingang fŸr Taste 1
	PORTB |= (1<<PB0);	//Pull-up

	DDRB &= ~(1<<PB1);	//Bit 1 von PORT B als Eingang fŸr Taste 2
	PORTB |= (1<<PB1);	//Pull-up
	*/

	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 4 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD

	DDRC &= ~(1<<ADC_TEMP_PIN);	//Pin 0 von PORT C als Eingang fuer ADC
//	PORTC |= (1<<DDC0); //Pull-up
	DDRC &= ~(1<<ADC_STROM_PIN);	//Pin 1 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC1); //Pull-up
	DDRC &= ~(1<<DDC2);	//Pin 2 von PORT C als Eingang fuer ADC 	
//	PORTC |= (1<<DDC3); //Pull-up
	DDRC &= ~(1<<DDC3);	//Pin 3 von PORT C als Eingang fuer Tastatur 	
//	PORTC |= (1<<DDC3); //Pull-up

   OSZIDDR |= (1<<OSZIA);
	
	
}



void delay_ms(unsigned int ms)/* delay for a minimum of <ms> */
{
	// we use a calibrated macro. This is more
	// accurate and not so much compiler dependent
	// as self made code.
	while(ms){
		_delay_ms(0.96);
		ms--;
	}
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
   OCR0A = 0xA0;
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


//
/*
void timer2()// Atmega8
{
	//----------------------------------------------------
	// Set up timer 0
	//----------------------------------------------------
   
    //TCCR0A = _BV(WGM01);
   // TCCR0B = _BV(CS00) | _BV(CS02);
   // OCR0A = 0x2;
   // TIMSK0 = _BV(OCIE0A);
    
   DDRB |= (1<<PORTB3);
   DDRD |= (1<< PORTD6);   // OC0A Output
   
   TCCR2 |= (1<<WGM21);   // fast PWM  top = 0xff
   TCCR2 |= (1<<WGM20);   // PWM
   //TCCR0A |= (1<<WGM02);   // PWM
   
   TCCR2 |= (1<<COM21);   // set OC0A at bottom, clear OC0A on compare match
   
   //TCCR2 |= 1<<CS02;
   TCCR2 |= 1<<CS01;
   //TCCR2 |= 1<<CS00;
   
   OCR2=100;
   TIMSK |= (1<<OCIE2);
   
}
*/

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
      if (beepcounter == 0)
      {
       //  status &= ~BEEP_ON;
      }
      
      if ((status & (1<<FAN_ON)) )
      {
         //OSZITOGG;
         if (beepburstcounter > 2)
         {
            beep_offtime = 24;
         }
         
         if (beepcounter == beep_offtime)
         {
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
            
         }
         beepcounter++;
      }
      else if ( (status & (1<<STROM_ON)))
      {
         OSZITOGG;
         if (beepburstcounter > 2)
         {
            beep_offtime = 24;
         }
         
         if (beepcounter == beep_offtime)
         {
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
            
         }
         beepcounter++;
      }
 
      else
      {
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
            beep_offtime = 8;
         }
         beepcounter++;
      }
 
      /*
      if ( (status & (1<<STROM_ON)))
      {
         OSZITOGG;
         if (beepburstcounter > 2)
         {
            beep_offtime = 24;
         }
         
         if (beepcounter == beep_offtime)
         {
            status |= (1<<BEEP_ON);
            beepcounter = 0;
            beepburstcounter++;
         }
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
            
         }
         beepcounter++;
      }
      else 
      {
         if (beepcounter == beep_ontime)
         {
            status &= ~(1<<BEEP_ON);
         }
         
         beepcounter++;
      }
*/
      /*
      if (beepcounter % beeptime == 0)
      {
         beepburstcounter++;
         status ^= (1<<BEEP_ON);
      }
      */
      timercount1++;
      
      status |= (1<<PWM_ADC);// ADC messen ausloesen
      
        
   }
   // LOOPLEDPORT |= (1<<LOOPLED);
   //if (sw>10)
   {
      //      OUTPORT |= (1<<PWM_OUT_PIN); // Triac on
   }
   //OSZIHI;
}
//#pragma mark TIMER2_OVF
ISR(TIMER2_OVF_vect)
{
   //OSZIHI;
   /*
   //OSZITOGG;
   //OSZIHI;
   timercount0++;
   if (timercount0 > TIMER2_PWM_TAKT) // Takt teilen
   {
      //OSZITOGG;
      timercount0=0;
      timercount1++;
      if (timercount1 > stellwert)
      {
         OUTPORT &= ~(1<<PWM_OUT_PIN); // Triac off
         status |= (1<<PWM_ADC);// ADC messen ausloesen
      }
      else if (timercount1 > TIMER2_ENDWERT)
      {
         timercount1 = 0;
         OUTPORT |= (1<<PWM_OUT_PIN); // Triac on
      }
      
   }
   // LOOPLEDPORT |= (1<<LOOPLED);
   if (stellwert>10)
   {
      
   }
    */
   OUTPORT |= (1<<PWM_FAN_PIN); // Triac on
 //  OUTPORT ^= (1<<BEEP_PIN);
}





//

/*
void timer2 (uint8_t wert) 
{ 
//	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

//Takt fuer Servo
	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2 |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC

	//OC2 akt
//	TCCR2 |= (1<<COM20);		//	OC2 Pin zuruecksetzen bei CTC


	TIFR |= (1<<TOV2); 				//Clear TOV2 Timer/Counter Overflow Flag. clear pending interrupts
	TIMSK |= (1<<OCIE2);			//CTC Interrupt aktivieren

	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2 = wert;					//Setzen des Compare Registers auf Servoimpulsdauer
} 
*/

/*
ISR(TIMER0_OVF_vect) 
{ 
	ADCImpuls++;
	Servopause++;
	//lcd_clr_line(1);

	//lcd_gotoxy(10,1);
	//lcd_puts("Tim\0");
	//delay_ms(400);
	//lcd_cls();
	//lcd_clr_line(0);
	//lcd_gotoxy(0,1);
	//lcd_puts("Stop Servo\0");
	//lcd_puts(" TP\0");
	//lcd_putint1(TWI_Pause);
	//	Intervall 64 us, Overflow nach 16.3 ms
	
	if (Servopause==3)	// Neues Impulspaket nach 48.9 ms
	{

		if (TWI_Pause)
		{
//			lcd_gotoxy(19,0);
//			lcd_putc(' ');
			timer2(Servoimpulsdauer);	 // setzt die Impulsdauer
			if (SERVOPORT &  (1<<SERVOPIN1)) // Servo ist ON
			{
				SERVOPORT |= (1<<SERVOPIN0); // Schaltet Impuls an SERVOPIN0 ein
			}
			SERVOPORT |= (1<<5);// Kontrolle auf PIN D5
		}
		Servopause=0;
	}
	
}
*/
/*
ISR(TIMER2_COMP_vect) // Schaltet Impuls an SERVOPIN0 aus
{
//		lcd_clr_line(1);
//		lcd_puts("Timer2 Comp\0");
		TCCR2=0;
		SERVOPORT &= ~(1<<SERVOPIN0);//	SERVOPIN0 zuruecksetzen
		SERVOPORT &= ~(1<<5);// Kontrolle auf PIN D5 OFF
		//delay_ms(800);
		//lcd_clr_line(1);
		
}
*/

   volatile  uint8_t pwmimpuls = 0;
   

void main (void) 
{
   MCUSR = 0;
	wdt_disable();

	slaveinit();
	//PORT2 |=(1<<PC4);
	//PORTC |=(1<<PC5);
	
	//uint16_t ADC_Wert= readKanal(0);
		
	/* initialize the LCD */
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);

	lcd_puts("Guten Tag\0");
	delay_ms(200);
	lcd_cls();
	lcd_puts("READY\0");
	

	uint8_t Tastenwert=0;
	uint8_t TastaturCount=0;
	
	uint16_t TastenStatus=0;
	uint16_t Tastencount=0;
	uint16_t Tastenprellen=0x01F;
	//timer0();
	
	//initADC(TASTATURPIN);
	
	uint16_t loopcount0=0;
   uint16_t loopcount1=0;
  

	//uint16_t startdelay1=0;

	uint8_t loopcount=0;
	//LOOPLEDPORT |=(1<<LOOPLED);
	
	delay_ms(200);

	lcd_clr_line(0);
   initADC(0);
   timer0();
   timer2();
   sei();
   status |= (1<<PWM_ADC);
//   status |= (1<<PID_FIRST_RUN); // K Prop ist beim Aufheizen kleiner
   #pragma mark while  
	while (1)
   {
      //Blinkanzeige
      loopcount0++;
      //OSZITOGG;
      // Stromregelung checken
      
      stromreg = readKanal(ADC_STROM_PIN); 
      if (stromreg < STROM_MIN)
      {
         if (!(status & (1<<STROM_ON)))
         {
         status |= (1<<STROM_ON);
         beepcounter = beep_offtime-1;
         beepburstcounter = 0;
         beep_offtime = 8;
         }
      }
      else if (stromreg > STROM_MIN +1)
      {
         if ((status & (1<<STROM_ON)))
         {
         status &= ~(1<<STROM_ON);
         beepburstcounter = 0;
         beep_offtime = 8;
         }
      }
      
      
      if (led_temp < TEMP_MAX)
      {
         if (!(status & (1<<FAN_ON)))
         {
            status |= (1<<FAN_ON);
            beepcounter = beep_offtime-1;
            beepburstcounter = 0;
            beep_offtime = 8;
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
  //          status &= ~(1<<BEEP_ON);
            beepburstcounter = 0;
            beep_offtime = 8;
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
         pwmimpuls = led_temp-550;
         lcd_putc(' ');
         lcd_putint12(pwmimpuls);
         
         OCR2A = pwmimpuls;
 //        lcd_putc(' ');
 //        lcd_putint12(loopcount++);
         
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
         //lcd_gotoxy(6,0);
         //lcd_putint(timercount1);
         //lcd_putc(' ');
         //lcd_putint12(adccount0);
      }
      
      
      
      
      
      
      
   
      

		
		/**	Ende Startroutinen	***********************/
		
		
		/*
		
		if (!(PINB & (1<<PB0))) // Taste 0
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P0 Down\0");
			
			if (! (TastenStatus & (1<<PB0))) //Taste 0 war nich nicht gedrueckt
			{
				//RingD2(5);
				TastenStatus |= (1<<PB0);
				Tastencount=0;
				//lcd_gotoxy(0,1);
				//lcd_puts("P0 \0");
				//lcd_putint(TastenStatus);
				//delay_ms(800);
			}
			else
			{
				
				
				Tastencount ++;
				//lcd_gotoxy(7,1);
				//lcd_puts("TC \0");
				//lcd_putint(Tastencount);
				
				if (Tastencount >= Tastenprellen)
				{
				}
			}//else
			
		}	// Taste 0
		
		
		if (!(PINB & (1<<PB1))) // Taste 1
		{
			//lcd_gotoxy(12,1);
			//lcd_puts("P1 Down\0");
			
			if (! (TastenStatus & (1<<PB1))) //Taste 1 war nicht nicht gedrueckt
			{
				TastenStatus |= (1<<PB1);
				Tastencount=0;
				//lcd_gotoxy(3,1);
				//lcd_puts("P1 \0");
				//lcd_putint(Servoimpulsdauer);
				//delay_ms(800);
				
			}
			else
			{
				//lcd_gotoxy(3,1);
				//lcd_puts("       \0");
				
				Tastencount ++;
				if (Tastencount >= Tastenprellen)
				{
					
					
						Tastencount=0;
					TastenStatus &= ~(1<<PB1);
				}
			}//	else
			
		} // Taste 1
		>*/
		/* ******************** */
		//		initADC(TASTATURPIN);
		//		Tastenwert=(uint8_t)(readKanal(TASTATURPIN)>>2);
		
		Tastenwert=0;
		
		//lcd_gotoxy(3,1);
		//lcd_putint(Tastenwert);
		
		if (Tastenwert>23)
		{
			/*
			 0: 
			 1: 
			 2: 
			 3: 
			 4: 
			 5: 
			 6: 
			 7: 
			 8: 
			 9: 
			 */
			
			TastaturCount++;
			if (TastaturCount>=50)
			{
				
				//lcd_clr_line(1);
//				lcd_gotoxy(8,1);
//				lcd_puts("T:\0");
//				lcd_putint(Tastenwert);
				
            uint8_t Taste=0;//=Tastenwahl(Tastenwert);
				
				
				
				TastaturCount=0;
				Tastenwert=0x00;
				
				switch (Taste)
				{
					case 0://
					{ 
						
					}break;
						
					case 1://
					{ 
					}break;
						
					case 2://
					{ 
						
					}break;
						
					case 3://
					{ 
						
					}break;
						
					case 4://
					{ 
						
					}break;
						
					case 5://
					{ 
						
						
					}break;
						
					case 6://
					{ 
						}break;
						
					case 7://
					{ 
						
					}break;
						
					case 8://
					{ 
						
					}break;
						
					case 9://
					{ 
					}break;
						
						
				}//switch Tastatur
			}//if TastaturCount	
			
		}//	if Tastenwert
		
		//	LOOPLEDPORT &= ~(1<<LOOPLED);
	}//while


// return 0;
}
