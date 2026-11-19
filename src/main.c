/*
   //      _______ __  _________ _________
   //      \_     |  |/  .  \   |   \ ___/
   //        / :  |   \  /   \  |   / _>_
   //       /      \ | \ |   /  |  /     \
   //      /   |___/___/____/ \___/_     /
   //      \___/--------TECH--------\___/
   //        ==== ABOVE SINCE 1994 ====
   //
   //   Ab0VE TECH - Open HONDA Coil on Plug Controller
   //             Inline4 - ATTINY24/44mod
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// OUTPUT
#define IGNITION1 PINB0
#define IGNITION2 PINB1
#define IGNITION3 PINB2
#define IGNITION4 PINA7
#define LED PINA3
#define TACHO PINA1

// INPUT
#define CYP1 PINA0
#define IGN PINA2

// Max coil dwell (safety cut-off if IGN from ECU stays LOW too long).
// Normally the spark happens on the IGN rising edge; this limit only
// matters if the ECU pulse is longer than the limit or the line is stuck.
// Timer0 tick = 1024 / 8 MHz = 128 us, max ~32.7 ms.
// 20 ms matches the effective limit of the previous firmware
// (that one actually ran at 1 MHz because of CKDIV8 in lfuse 0x52).
#define MAX_DWELL_US 20000UL
#define DWELL_TICKS ((MAX_DWELL_US + 64UL) / 128UL)
#if DWELL_TICKS < 1 || DWELL_TICKS > 256
#error "MAX_DWELL_US out of range (128..32768 us)"
#endif

// LED blink period while waiting for first CYP1 (toggle interval)
#define LED_BLINK_MS 500UL
#define LED_BLINK_TICKS ((uint16_t)(F_CPU / 1024UL * LED_BLINK_MS / 1000UL))

// CYP1 noise filter.
// Timer1 (free running, 128 us tick, overflows after ~8.4 s) measures the time
// since the last accepted CYP1 pulse. A pulse that comes sooner than 3/8 of the
// last known CYP1 period is ignored: the engine cannot speed up that much
// within one camshaft revolution, so it is noise.
// The period is "known" only when the accepted pulse agreed with the firing
// order counter (cylinder 1 was next anyway) and did not come more than twice
// later than expected. Otherwise (the pulse had to correct the order, CYP1
// pulses were lost, the engine stalled) the period is forgotten and the next
// pulse is accepted unconditionally, so a real pulse is not rejected after a
// false one, after lost IGN pulses (ignition cut) or after a restart.
// Note: every dwell start resets the shared prescaler, so Timer1 loses up to
// one tick per spark. It affects both compared values equally.

#define WATCHDOG_DELAY WDTO_30MS // Set watchdog for 30 mSec

// Helpers are called from ISRs and must really be inlined: a call from an ISR
// makes it save every call-clobbered register and adds ~4 us to the spark latency
#define INLINE inline __attribute__((always_inline))

// STATE (changed only inside ISRs)
static volatile uint8_t NEXT_CYL = 0; // coil for the next dwell: 0 = not synced, 1..4 = firing order position
static volatile uint8_t DWELL_ACTIVE = 0;
static volatile uint8_t PREV_PINA = 0;
static volatile uint16_t CYP_PERIOD = 0;     // last CYP1 period in Timer1 ticks, 0 = unknown
static volatile uint8_t CYP_TIMER_VALID = 0; // Timer1 has not overflowed since the last accepted CYP1

static INLINE void Led_Toggle(void)
{
        PINA = _BV(LED); // writing 1 to PINx toggles the output
}

static INLINE void Dwell_Timer_Start(void)
{
        TCCR0B = 0;                           // stop
        TCNT0 = 0;
        TIFR0 = _BV(OCF0A);                   // clear pending compare flag
        GTCCR = _BV(PSR10);                   // reset prescaler -> exact timeout
        TCCR0B = _BV(CS02) | _BV(CS00);       // clk/1024, start
}

static INLINE void Dwell_Timer_Stop(void)
{
        TCCR0B = 0;
        TIFR0 = _BV(OCF0A);
}

// Start charging the coil for position 1..4, firing order 1-3-4-2
static INLINE void Ignite_ON(uint8_t pos)
{
        switch (pos)
        {
        case 1:
                PORTB |= _BV(IGNITION1);
                break;
        case 2:
                PORTB |= _BV(IGNITION3);
                break;
        case 3:
                PORTA |= _BV(IGNITION4);
                break;
        case 4:
                PORTB |= _BV(IGNITION2);
                break;
        }
}

// Spark: switch off all coils (only ignition bits, other port functions intact)
static INLINE void Ignite_Off(void)
{
        PORTB &= ~(_BV(IGNITION1) | _BV(IGNITION2) | _BV(IGNITION3));
        PORTA &= ~_BV(IGNITION4);
}

static INLINE void Fire(void)
{
        Ignite_Off();
        Dwell_Timer_Stop();
        PORTA &= ~_BV(TACHO); // OFF TACH
        DWELL_ACTIVE = 0;
}

// Dwell safety limit reached -> fire now
ISR(TIM0_COMPA_vect)
{
        if (DWELL_ACTIVE)
        {
                Fire();
        }
        else
        {
                Dwell_Timer_Stop();
        }
}

// LED blink before sync (disabled on the first CYP1)
ISR(TIM1_COMPA_vect)
{
        OCR1A += LED_BLINK_TICKS;
        if (NEXT_CYL == 0)
        {
                Led_Toggle();
        }
}

// No CYP1 for ~8.4 s: elapsed time is no longer measurable (enabled after sync)
ISR(TIM1_OVF_vect)
{
        CYP_TIMER_VALID = 0;
}

// PCINT0: CYP1 (PA0) and IGN (PA2), both edges.
//
// The entry is hand written in assembler: a C ISR saves every register its
// body uses (11 of them) before it even reads PINA, which delays the spark by
// ~6 us. The stub saves only r24/r25, reads PINA at once and on the IGN rising
// edge switches the coils off using only instructions that do not touch SREG
// (lds/sbrs/sbrc/cbi), so SREG is saved after the spark. Then it saves the
// call-clobbered registers and calls Pcint0_Tail() for the rest in C.
// Switching the coils off unconditionally is the same as Fire()'s coil part:
// outside of a dwell they are off anyway.
static void Pcint0_Tail(uint8_t pins, uint8_t changed) __attribute__((used));

ISR(PCINT0_vect, ISR_NAKED)
{
        __asm__ volatile(
            "push r24\n"
            "in r24, %[pina]\n" // pins
            "push r25\n"
            "lds r25, %[prev]\n" // previous pins
            // IGN rising edge = spark: high now, low before
            "sbrs r24, %[ign]\n"
            "rjmp 1f\n"
            "sbrc r25, %[ign]\n"
            "rjmp 1f\n"
            "cbi %[portb], %[ign1]\n"
            "cbi %[portb], %[ign2]\n"
            "cbi %[portb], %[ign3]\n"
            "cbi %[porta], %[ign4]\n"
            "1:\n"
            "sts %[prev], r24\n"
            "push r0\n"
            "in r0, __SREG__\n"
            "push r0\n"
            "push r1\n"
            "clr r1\n"
            "eor r25, r24\n" // changed
            // Rest of the handler in C (normal call, so save what it may clobber)
            "push r18\n"
            "push r19\n"
            "push r20\n"
            "push r21\n"
            "push r22\n"
            "push r23\n"
            "push r26\n"
            "push r27\n"
            "push r30\n"
            "push r31\n"
            "mov r22, r25\n" // Pcint0_Tail(pins = r24, changed = r22)
            "rcall %x[tail]\n"
            "pop r31\n"
            "pop r30\n"
            "pop r27\n"
            "pop r26\n"
            "pop r23\n"
            "pop r22\n"
            "pop r21\n"
            "pop r20\n"
            "pop r19\n"
            "pop r18\n"
            "pop r1\n"
            "pop r0\n"
            "out __SREG__, r0\n"
            "pop r0\n"
            "pop r25\n"
            "pop r24\n"
            "reti"
            :
            : [pina] "I"(_SFR_IO_ADDR(PINA)),
              [porta] "I"(_SFR_IO_ADDR(PORTA)),
              [portb] "I"(_SFR_IO_ADDR(PORTB)),
              [ign] "I"(IGN),
              [ign1] "I"(IGNITION1),
              [ign2] "I"(IGNITION2),
              [ign3] "I"(IGNITION3),
              [ign4] "I"(IGNITION4),
              [prev] "i"(&PREV_PINA),
              [tail] "i"(Pcint0_Tail));
}

// PCINT0 handler body after the spark (called from the stub above)
static void Pcint0_Tail(uint8_t pins, uint8_t changed)
{
        // IGN rising edge: coils are already off, finish the spark
        if ((changed & _BV(IGN)) && (pins & _BV(IGN)))
        {
                if (DWELL_ACTIVE)
                {
                        Fire();
                }
        }

        // CYP1 falling edge = cylinder 1 is next (unless filtered out as noise)
        if ((changed & _BV(CYP1)) && !(pins & _BV(CYP1)))
        {
                // If the overflow flag is already pending, the elapsed counter
                // cannot be trusted even if the timer overflow ISR has not run
                // and cleared CYP_TIMER_VALID yet.
                if (TIFR1 & _BV(TOV1))
                {
                        TIFR1 = _BV(TOV1);
                        CYP_TIMER_VALID = 0;
                }
                uint16_t elapsed = TCNT1;
                uint8_t next = NEXT_CYL;
                uint8_t valid = CYP_TIMER_VALID;
                uint16_t period = CYP_PERIOD;
                // period / 8 * 3 without multiplication: a __mulhi3 call here
                // would make the ISR save every call-clobbered register
                uint16_t eighth = period >> 3;
                uint16_t min_period = eighth + (eighth << 1);
                if (next == 0 || !valid || elapsed >= min_period)
                {
                        // Trust the measured period only if the counter agreed
                        // and the pulse is not suspiciously late
                        CYP_PERIOD = (next == 1 && valid && (period == 0 || elapsed / 2 <= period)) ? elapsed : 0;
                        TCNT1 = 0;
                        TIFR1 = _BV(TOV1);
                        TIMSK1 = _BV(TOIE1); // blink off, overflow watch on
                        CYP_TIMER_VALID = 1;
                        NEXT_CYL = 1;
                        Led_Toggle();
                }
        }

        // IGN falling edge = start dwell on the next coil
        if ((changed & _BV(IGN)) && !(pins & _BV(IGN)))
        {
                uint8_t pos = NEXT_CYL;
                if (pos != 0 && !DWELL_ACTIVE)
                {
                        Ignite_ON(pos);
                        PORTA |= _BV(TACHO); // ON TACH
                        DWELL_ACTIVE = 1;
                        Dwell_Timer_Start();
                        // Advance; if CYP1 is lost keep going 1-3-4-2,
                        // next CYP1 re-syncs anyway
                        NEXT_CYL = (pos >= 4) ? 1 : (uint8_t)(pos + 1);
                }
        }
}

int main(void)
{
        cli();

        // Watchdog may still be running after a WDT reset
        MCUSR = 0;
        wdt_disable();

        // Run at full 8 MHz regardless of CKDIV8 fuse
        CLKPR = _BV(CLKPCE);
        CLKPR = 0;

        NEXT_CYL = 0; // wait for CYP1
        DWELL_ACTIVE = 0;

        // Setup ports
        PORTA = 0;
        PORTB = 0;
        DDRB = _BV(IGNITION1) | _BV(IGNITION2) | _BV(IGNITION3);
        DDRA = _BV(TACHO) | _BV(LED) | _BV(IGNITION4);

        // Timer0: dwell safety limit, CTC, started per dwell
        TCCR0A = _BV(WGM01);
        TCCR0B = 0;
        OCR0A = (uint8_t)(DWELL_TICKS - 1);
        TIMSK0 = _BV(OCIE0A);

        // Timer1: free running, clk/1024. LED blink time base before sync,
        // CYP1 period measurement after
        CYP_PERIOD = 0;
        CYP_TIMER_VALID = 0;
        TCCR1A = 0;
        TCCR1B = _BV(CS12) | _BV(CS10);
        TCNT1 = 0;
        OCR1A = LED_BLINK_TICKS;
        TIMSK1 = _BV(OCIE1A);

        // Pin change interrupts
        PREV_PINA = PINA;
        PCMSK0 = _BV(PCINT0) | _BV(PCINT2);
        GIFR = _BV(PCIF0);
        GIMSK = _BV(PCIE0);

        // Enable WATCHDOG
        wdt_enable(WATCHDOG_DELAY);

        // Start Triggers
        sei();

        // Main loop
        while (1)
        {
                wdt_reset();
        }
}
