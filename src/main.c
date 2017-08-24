/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //       ==== ABOVE SCIENCE 1994 ====
   //
   //   Ab0VE TECH - HONDA Open Coil on Plug Controller
   //             Inline4 - AT24/44mod
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//*** CONFIG ****
#define WATCHDOG_DELAY WDTO_30MS // Set watchdog for 30 mSec

//#define DIRECT_FIRE // !!! Use DirectFire coils instead Coil-on-plug !!!
#define MULTI_FIRE // Multifire - only for Coil-on-plug
#define MULTI_FIRE_DELAY 20 // 20us for recharge coil

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

//VARS
uint8_t CYLINDER=0;
uint8_t WASFIRE=0;
uint8_t INFIRE=0;
uint8_t WASCYP=0;
uint8_t SPARKS=0;
uint16_t RPM=0;
uint64_t LASTMILLIS=0;
uint64_t _1000us=0, _millis=0;
uint8_t LEDSTATUS=0;

#if defined(DIRECT_FIRE) && defined(MULTI_FIRE)
#error "Multifire is NOT capable with DirectFire!"
#endif

// PCINT4 - CYP PA4
ISR( __vectorPCINT4,  ISR_NOBLOCK) {
// Check CYP
        if ( ((PINA & (1 << 4))==0 ) && (!WASCYP)) { // CYP - LOW
                CYLINDER=1; WASCYP=1;
        }
        if ( ((PINA & (1 << 4))==1 ) && (WASCYP)) { // GONE CYP
                WASCYP=0;
        }
}

// PCINT13 - IGN PB5
ISR( __vectorPCINT13, ISR_NOBLOCK) {
// Check FIRE
        if ( ((PINB & (1 << 5))==1) && (!INFIRE)) { // Start FIRE - HIGH
                INFIRE=1;
        }
        if ( ((PINB & (1 << 5))==0) && (INFIRE)) { // GONE FIRE
                INFIRE=0;
        }
}

// TIMER
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
        DDRB = IGNITION1 + IGNITION2 + IGNITION3 + IGNITION4;
        DDRA = TACHO + LED;

        wdt_enable (WATCHDOG_DELAY); // Enable WATCHDOG

        sei();

        PORTA |= _BV(LED); // LED ON on start
        LASTMILLIS=millis();

// Main loop
        while (1) {
                wdt_reset (); // reset WDR
                if (INFIRE==1) { // Fire coil?
                        WASFIRE=1;
                        PORTA |= _BV(TACHO); // TACHO output
                        switch (CYLINDER) { // Set fire for required coil 1-3-4-2
                        case 0: WASFIRE = 0; break; // SKIP fire until first CYP signal
                        case 1: PORTB |= _BV(IGNITION1); break;
                        case 2: PORTB |= _BV(IGNITION3); break;
                        case 3: PORTB |= _BV(IGNITION4); break;
                        case 4: PORTB |= _BV(IGNITION2); break;
                        }
                        SPARKS++;

#ifdef DIRECT_FIRE // DirectFire Coils
                        // Ignition sustaining (10° crankshaft rotation after ignition point)
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
#else // Coil-on-plug
                        _delay_us(10);

            #ifdef MULTI_FIRE
                        if (RPM < 2500) { // Extra spark on low RPM
                                PORTB=0; // Off ignition
                                _delay_us(MULTI_FIRE_DELAY);
                                switch (CYLINDER) { // Set fire for required coil 1-3-4-2
                                case 0: WASFIRE = 0; break; // SKIP fire until first CYP signal
                                case 1: PORTB |= _BV(IGNITION1); break;
                                case 2: PORTB |= _BV(IGNITION3); break;
                                case 3: PORTB |= _BV(IGNITION4); break;
                                case 4: PORTB |= _BV(IGNITION2); break;
                                }
                                _delay_us(20);
                        }
            #endif // MULTI_FIRE

#endif // COP or DF
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
