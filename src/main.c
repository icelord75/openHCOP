/*
//      _______ __  _________ _________
//      \_     |  |/  .  \   |   \ ___/
//        / :  |   \  /   \  |   / _>_
//       /      \ | \ |   /  |  /     \
//      /   |___/___/____/ \___/_     /
//      \___/--------TECH--------\___/
//       ==== ABOVE SCIENCE 1994 ====
//
//   Coil on Plug I4 Controller AT44mod Version 0.3 / 2017
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// OUTPUT
#define IGNITION1 PB0
#define IGNITION2 PB1
#define IGNITION3 PB2
#define IGNITION4 PB3
#define LED       PA3
#define TACHO     PA1
// INPUT
#define CYP1      PA0
#define IGN       PA2

//#define F_CPU sysclk_get_cpu_hz()

uint8_t CYLINDER=0;
uint8_t WASFIRE=0;
uint8_t INFIRE=0;
uint8_t WASCYP=0;
uint8_t SPARKS=0;
uint16_t RPM=0;
uint64_t LASTMILLIS=0;
uint64_t _1000us=0, _millis=0;
uint8_t LEDSTATUS=0;

//UART
volatile uint8_t c;
volatile uint8_t coef;
volatile uint8_t Rece_bit;
volatile uint8_t count;
volatile uint8_t start;
uint8_t temp;

ISR( __vectorPCINT4,  ISR_NOBLOCK) // PCINT4 - CYP PA4
{

// Check CYP
        if ( ((PINA & (1 << 4))==0 ) && (!WASCYP)) { // CYP - LOW
                CYLINDER=1; WASCYP=1;
        }
        if ( ((PINA & (1 << 4))==1 ) && (WASCYP)) { // GONE CYP
                WASCYP=0;
        }
}

ISR( __vectorPCINT13, ISR_NOBLOCK) // PCINT13 - IGN PB5
{
// Check FIRE
        if ( ((PINB & (1 << 5))==1) && (!INFIRE)) { // Start FIRE - HIGH
                INFIRE=1;
        }
        if ( ((PINB & (1 << 5))==0) && (INFIRE)) { // GONE FIRE
                INFIRE=0;
        }
}

ISR(__vectorTIM0_OVF) {
        _1000us += 256;
        while (_1000us > 1000) {
                _millis++;
                _1000us -= 1000;
        }
}

// safe access to millis counter
uint64_t millis() {
        uint64_t m;
        cli();
        m = _millis;
        sei();
        return m;
}

int main(void)
{

// Setup
        DDRB = PB0+PB1+PB2+PB2;
        DDRA = PA1+PA3;

        sei();

        PORTA |= _BV(LED); // LED ON on start
        LASTMILLIS=millis();

// Main loop
        while (1)
        {
                if (INFIRE==1)// Fire coil?
                { WASFIRE=1;
                  PORTA |= _BV(TACHO);
                  switch (CYLINDER) { // Set fire for required coil 1-3-4-2
                  case 1: PORTB |= _BV(IGNITION1);
                  case 2: PORTB |= _BV(IGNITION3);
                  case 3: PORTB |= _BV(IGNITION4);
                  case 4: PORTB |= _BV(IGNITION2);
                  }
                  SPARKS++; 
// Ignition delay (10° crankshaft rotation after ignition point)
                  if (RPM > 11000 ) {
                          _delay_us(150);
                  } else
                  if (RPM > 10000 ) {
                          _delay_us(160);
                  } else
                  if (RPM > 9000 ) {
                          _delay_us(180);
                  } else
                  if (RPM > 8600 ) {
                          _delay_us(190);
                  } else
                  if (RPM > 7600 ) {
                          _delay_us(210);
                  } else
                  if (RPM > 7000 ) {
                          _delay_us(230);
                  } else
                  if (RPM > 6000 ) {
                          _delay_us(270);
                  } else
                          _delay_us(300);
// Stop igniting
                  PORTB=0; // Off ignition
                  PORTA &= ~_BV(TACHO); // Off TACHO
                } else
                {
                        if (WASFIRE==1) { // ready for next cylinder?
                                PORTB=0; // disable all coils and TACHO for some reasons
                                WASFIRE=0;
                                CYLINDER++;
                                if (CYLINDER>4) { // only 4 cylinder model :)
                                        CYLINDER=1;
                                        if (LEDSTATUS==0) {// Change LED on each _FULL_ cycle
                                                PORTA &= ~_BV(LED); // LED OFF
                                                LEDSTATUS=1;
                                        } else {
                                                PORTA |= _BV(LED); // LED ON
                                                LEDSTATUS=0;
                                        }
                                }
                        } // was fire
                }
                if (SPARKS >= 8) { // TWO CYCLES
                        RPM = 30 * 1000 / (millis() - LASTMILLIS) * SPARKS;
                        LASTMILLIS = millis();
                        SPARKS = 0;
                }
        } // main loop
}
