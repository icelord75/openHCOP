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

#define ECU_FIRE_SUSTAIN         // Use IGN signal for fire sustaining

//#define COP_SUSTAIN 10         // 10µSec COP fire trigger sustaining
#define MULTI_FIRE               // Multifire - only for Coil-on-plug
#define MULTI_FIRE_DELAY 200     // 200µSec for recharge coil
#define MULTI_FIRE_SUSTAIN 1000  // 1000µSec for multifire sustaining

//#define DIRECT_FIRE            // Use DirectFire coils instead Coil-on-plug

// OUTPUT
#define IGNITION1 PB0
#define IGNITION2 PB1
#define IGNITION3 PB2
#define IGNITION4 PA7
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

#if defined(ECU_FIRE_SUSTAIN) && defined(COP_SUSTAIN)
#error "No ECU and Defined sustaining in same time"
#endif

#define _NOP __asm__ __volatile__ ("nop");

// PCINT0 - CYP1 PA0
ISR( __vectorPCINT0,  ISR_NOBLOCK) {
// Check CYP
        if ( ((PINA & _BV(CYP1))==0 ) && (!WASCYP)) { // CYP - LOW
                CYLINDER=1; WASCYP=1;
        }
        if ( ((PINA & _BV(CYP1))==1 ) && (WASCYP)) { // GONE CYP
                WASCYP=0;
        }
}

// PCINT2 - IGN PA2
ISR( __vectorPCINT2, ISR_NOBLOCK) {
// Check FIRE
        if ( ((PINA & _BV(IGN))==1) && (!INFIRE)) { // Start FIRE - HIGH
                INFIRE=1;
        }
        if ( ((PINA & _BV(IGN))==0) && (INFIRE)) { // GONE FIRE
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
        DDRB = _BV(IGNITION1) | _BV(IGNITION2) | _BV(IGNITION3);
        DDRA = _BV(TACHO) | _BV(LED) | _BV(IGNITION4);
        GIMSK = _BV(PCIE0);
        MCUCR = _BV(ISC00); //ISC00 - any change
        PCMSK0 = _BV(PCINT0) | _BV(PCINT2);
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
                        case 3: PORTA |= _BV(IGNITION4); break;
                        case 4: PORTB |= _BV(IGNITION2); break;
                        }
                        SPARKS++;
#ifdef DIRECT_FIRE // DirectFire Coils
            #ifdef ECU_FIRE_SUSTAIN
                        while ((PINB & _BV(IGN))==1) { _NOP }; // Delay while IGN signal present
            #else
                        // Ignition sustaining
                        int DIRECT_FIRE_DELAY=map(ROM,200,10000,2000,800); // linear 200rpm->2000µSec 10Krpm->800µSec
                        _delay_us(DIRECT_FIRE_DELAY);
            #endif
#else // Coil-on-plug
            #ifdef ECU_FIRE_SUSTAIN
                        while ((PINB & _BV(IGN))==1) { _NOP }; // Delay while IGN signal present
            #else
                        _delay_us(COP_SUSTAIN);
            #endif
            #ifdef MULTI_FIRE
                        if (RPM < 2500) { // Extra spark on low RPM
                                PORTB = 0; // Off ignition
                                PORTA &= ~_BV(IGNITION4); // Off IGNITION4
                                _delay_us(MULTI_FIRE_DELAY);
                                switch (CYLINDER) { // Set fire for required coil 1-3-4-2
                                case 0: WASFIRE = 0; break; // SKIP fire until first CYP signal
                                case 1: PORTB |= _BV(IGNITION1); break;
                                case 2: PORTB |= _BV(IGNITION3); break;
                                case 3: PORTA |= _BV(IGNITION4); break;
                                case 4: PORTB |= _BV(IGNITION2); break;
                                }
                                _delay_us(MULTI_FIRE_SUSTAIN);
                        }
            #endif // MULTI_FIRE
#endif // COP or DF
// Stop igniting
                        PORTB = 0; // Off ignition
                        PORTA &= ~_BV(IGNITION4); // Off IGNITION4
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
