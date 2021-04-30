//
//  defines.h
//  TWI_Slave
//
//  Created by Ruedi Heimlicher on 19.09.2016.
//
//

#ifndef defines_h
#define defines_h
//defines.h

#define MAX_TEMP  128

#define PWM_FAKTOR   2.0

#define K_DELTA   .10

#define K_PROP    3.0
#define K_PROP_RED   1.0

#define K_PROP_LO   1.8
#define K_PROP_HI   4.0


#define K_INT     0.5

#define K_DIFF    0.1

#define K_WINDUP_HI  40
#define K_WINDUP_LO  -10

#define TIMER2_ENDWERT 0x9E // OV 20ms

#define TIMER2_COMPA 0x9F // OV 20ms


#define TIMER2_PWM_INTERVALL 0xFF // Paketlaenge


#define OSZIPORT				PORTB
#define OSZIDDR            DDRB
#define OSZIA					1
#define OSZILO OSZIPORT &= ~(1<<OSZIA)
#define OSZIHI OSZIPORT |= (1<<OSZIA)
#define OSZITOGG OSZIPORT ^= (1<<OSZIA)
// Define fuer Slave:
#define TOPLED_PIN			1 // Blinkt waehrend heizen, voll wenn Temp erreicht

// ADC
#define ADCPORT PORTC
#define ADCDDR    DDRC
#define ADC_TEMP_PIN    0 // von Kuehlkoerper
#define ADC_STROM_PIN   1 // von OP Amp, sinkend


// I/O
#define OUTPORT   PORTD
#define OUTDDR    DDRD

#define BLINK_PIN       1 // OUT active HI
#define BEEP_PIN        2 // OUT altern
#define PWM_FAN_PIN     3 // OUT active HI

#define OUT_OFF_PIN     4 // Not-Aus, active HI


// Bits status
#define PWM_ON       0
#define PWM_ADC      1
#define FAN_ON       3

#define BEEP_ON      4

#define STROM_ON     5 // Stromregelung wirkt

#define STROM_MIN   300
#define TEMP_MAX     610 // Fan starten, beep
#define TEMP_OFF     550 // Ausschalten

#define TIMER2_BLINK_TAKT 2

//avr-size  --mcu=attiny85 -C Laminator.elf

#endif /* defines_h */
