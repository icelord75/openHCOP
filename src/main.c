/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //        ==== ABOVE SINCE 1994 ====
   //
   //   Ab0VE TECH - HONDA Open Coil on Plug Controller
   //             Inline4 - AT24/44mod
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// OUTPUT
#define IGNITION1 PINB0
#define IGNITION2 PINB1
#define IGNITION3 PINB2
#define IGNITION4 PINA7
#define LED       PINA3
#define TACHO     PINA1

// INPUT
#define CYP1      PINA0
#define IGN       PINA2

//VARS
uint8_t CYLINDER    = 0;
uint8_t SPARKS      = 0;
uint16_t RPM        = 0;
uint8_t LEDSTATUS   = 0;

#define WATCHDOG_DELAY WDTO_30MS   // Set watchdog for 30 mSec

void Led_ON(){
        PORTA |= _BV(LED); // LED ON
}

void Led_OFF(){
        PORTA &= ~_BV(LED); // LED OFF
}

// TIMER
ISR(TIM1_COMPA_vect) {
        if (LEDSTATUS == 0) { // Change LED on 2/10 sec
                Led_OFF();
                LEDSTATUS = 1;
        } else {
                Led_ON();
                LEDSTATUS = 0;
        }
        RPM=SPARKS*60*10/8;
        SPARKS=0;
}

void Ignite_ON() {
        switch (CYLINDER) { // Set fire for required coil 1-3-4-2
        case 0: break; // SKIP fire until first CYP signal
        case 1: PORTB |= _BV(IGNITION1); break;
        case 2: PORTB |= _BV(IGNITION3); break;
        case 3: PORTA |= _BV(IGNITION4); break;
        case 4: PORTB |= _BV(IGNITION2); break;
        }
}

void Ignite_Off() {
        PORTB = 0; // Off IGNITION1/2/3
        PORTA &= ~_BV(IGNITION4); // OFF IGNITION4
}

// PCINT0 Vector handling PINS: PCINT0 and PCINT2
ISR(PCINT0_vect) {
        cli();
// Check CYP
        if ( (PINA & _BV(CYP1))==0) { // CYP - LOW
                CYLINDER=1;
        }
// Check IGN
        if ( ((PINA & _BV(IGN))==0) && (CYLINDER > 0)) { // Start FIRE - LOW after first CYP1
                PORTA |= _BV(TACHO); // ON TACH
                Ignite_ON();
                SPARKS++;
                while ((PINA & _BV(IGN))==0) ;
                Ignite_Off();
                PORTA &= ~_BV(TACHO); // OFF TACH
                CYLINDER++;
                if (CYLINDER > 4) { // if we lost CYP1 signal
                        CYLINDER = 1;
                }
        }
        sei();
}

int main(void) {
        cli();
        CYLINDER = 0; // Make shure for wait CYP1
// Setup
        DDRB = _BV(IGNITION1) | _BV(IGNITION2) | _BV(IGNITION3);
        DDRA = _BV(TACHO) | _BV(LED) | _BV(IGNITION4);
        GIMSK = _BV(PCIE0);
        MCUCR |= _BV(ISC00); //ISC00 - any change
        PCMSK0 = _BV(PCINT0) | _BV(PCINT2);

//Timer1 is used as 1/10 sec time base
//Timer Clock = 1/1024 of sys clock
//Mode = CTC (Clear Timer On Compare)
        TCCR1B |= ((1<<WGM12)|(1<<CS12)|(1<<CS10));
        OCR1A = 8000000/1024/10;  //Compare value
        TIMSK1 |= (1<<OCIE1A);    //Output compare 1A interrupt enable

        wdt_enable (WATCHDOG_DELAY); // Enable WATCHDOG
        sei();

// Main loop
        while ( 1 ) {
                wdt_reset (); // reset WDR
                _delay_us(1);
        }
}
