/*
 * openHCOP firmware simulation test bench (simavr, ATtiny44 @ 8 MHz)
 *
 * Feeds CYP1 (PA0) and IGN (PA2) like the ECU/distributor would and checks
 * coil outputs: firing order 1-3-4-2, dwell follows IGN, safety cut-off,
 * spark latency, CYP1 edge handling, watchdog stability.
 *
 * Usage: ./harness firmware.elf [-v]
 * Exit code: 0 = all tests passed, 1 = failure
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_avr.h"
#include "sim_elf.h"
#include "avr_ioport.h"

#define F_SIM 8000000.0
#define MAX_DWELL_MS 20.0   /* must match MAX_DWELL_US in src/main.c */
#define MAX_LATENCY_US 15.0 /* IGN rising edge -> coil OFF */

enum { CH_CYL1 = 1, CH_CYL2, CH_CYL3, CH_CYL4, CH_TACH, CH_LED };
static const char *names[] = {"?", "cyl1(PB0)", "cyl2(PB1)", "cyl3(PB2)", "cyl4(PA7)", "TACH(PA1)", "LED(PA3)"};

static avr_t *avr;
static avr_irq_t *in_cyp, *in_ign;
static int verbose = 0;
static int failures = 0;

static double ign_edge_ms = 0;
static double last_off_ms = 0, last_on_ms = 0;
static double max_latency_us = 0;
static int order[64], norder = 0;
static int led_toggles = 0;

static double now_ms(void) { return avr->cycle / F_SIM * 1000.0; }

static void out_cb(avr_irq_t *irq, uint32_t v, void *p)
{
        (void)irq;
        int id = (int)(intptr_t)p;
        if (now_ms() < 0.1)
                return; /* ignore initial port setup */
        if (id == CH_LED)
                led_toggles++;
        if (id >= CH_CYL1 && id <= CH_CYL4) {
                if (v) {
                        if (norder < 64)
                                order[norder++] = id;
                        last_on_ms = now_ms();
                } else {
                        last_off_ms = now_ms();
                        double lat = (now_ms() - ign_edge_ms) * 1000.0;
                        if (lat < 1000.0 && lat > max_latency_us)
                                max_latency_us = lat;
                }
        }
        if (verbose)
                printf("  %9.3f ms  %-10s %s  (%.1f us after last IGN edge)\n", now_ms(), names[id],
                       v ? "ON " : "OFF", (now_ms() - ign_edge_ms) * 1000.0);
}

static void run_ms(double ms)
{
        uint64_t target = avr->cycle + (uint64_t)(ms * F_SIM / 1000.0);
        while (avr->cycle < target) {
                int st = avr_run(avr);
                if (st == cpu_Done || st == cpu_Crashed) {
                        printf("CPU STOPPED (state %d)\n", st);
                        exit(1);
                }
        }
}

static void ign(int level) { ign_edge_ms = now_ms(); avr_raise_irq(in_ign, level); }
static void cyp(int level) { avr_raise_irq(in_cyp, level); }
static void reset_log(void) { norder = 0; led_toggles = 0; }

/* one ignition event: IGN low (dwell) then high (spark) */
static void spark(double dwell_ms, double rest_ms) { ign(0); run_ms(dwell_ms); ign(1); run_ms(rest_ms); }
static void cyp_pulse(double low_ms) { cyp(0); run_ms(low_ms); cyp(1); run_ms(low_ms); }

/*
 * Engine model: n ignition events with a constant period of dwell_ms + rest_ms.
 * A real CYP1 pulse comes before every 4th event (cylinder 1), inside the
 * preceding rest time. `phase` is the true position in the firing order.
 */
static int phase = 0;
static void engine(int n, double dwell_ms, double rest_ms, int with_cyp)
{
        for (int i = 0; i < n; i++) {
                ign(0);
                run_ms(dwell_ms);
                ign(1);
                phase = (phase + 1) % 4;
                if (phase == 0 && with_cyp) {
                        run_ms(rest_ms - 0.6);
                        cyp_pulse(0.3);
                } else {
                        run_ms(rest_ms);
                }
        }
}

/* cylinder number for channel: CH_CYLn -> n */
static void check(const char *name, int ok, const char *detail)
{
        printf("%-58s %s%s%s\n", name, ok ? "PASS" : "FAIL", detail ? "  " : "", detail ? detail : "");
        if (!ok)
                failures++;
}

static void check_order(const char *name, const int *exp, int n)
{
        char buf[256];
        int ok = (norder == n);
        int pos = snprintf(buf, sizeof buf, "got:");
        for (int i = 0; i < norder && pos < (int)sizeof buf - 4; i++)
                pos += snprintf(buf + pos, sizeof buf - pos, " %d", order[i]);
        for (int i = 0; ok && i < n; i++)
                ok = (order[i] == exp[i]);
        check(name, ok, ok ? NULL : buf);
}

int main(int argc, char **argv)
{
        if (argc < 2) {
                fprintf(stderr, "usage: %s firmware.elf [-v]\n", argv[0]);
                return 2;
        }
        verbose = (argc > 2 && !strcmp(argv[2], "-v"));

        elf_firmware_t fw;
        memset(&fw, 0, sizeof fw);
        if (elf_read_firmware(argv[1], &fw)) {
                fprintf(stderr, "cannot read %s\n", argv[1]);
                return 2;
        }
        avr = avr_make_mcu_by_name("attiny44");
        if (!avr) {
                fprintf(stderr, "simavr has no attiny44 core\n");
                return 2;
        }
        avr_init(avr);
        avr->frequency = (uint32_t)F_SIM;
        avr_load_firmware(avr, &fw);
        avr->log = verbose ? LOG_WARNING : LOG_ERROR;

        in_cyp = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), 0);
        in_ign = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), 2);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 0), out_cb, (void *)CH_CYL1);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 1), out_cb, (void *)CH_CYL2);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 2), out_cb, (void *)CH_CYL3);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), 7), out_cb, (void *)CH_CYL4);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), 1), out_cb, (void *)CH_TACH);
        avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('A'), 3), out_cb, (void *)CH_LED);
        cyp(1);
        ign(1);
        run_ms(10);

        static const int seq[] = {1, 3, 4, 2, 1, 3, 4, 2, 1, 3, 4, 2, 1, 3, 4, 2};
        char d[128];

        /* T1 */
        if (verbose) printf("--- T1\n");
        reset_log();
        for (int i = 0; i < 4; i++)
                spark(3, 12);
        run_ms(1100);
        check("T1 no CYP yet: coils stay off", norder == 0, NULL);
        snprintf(d, sizeof d, "toggles in ~1.16 s: %d", led_toggles);
        check("T1 no CYP yet: LED blinks every 0.5 s", led_toggles == 2, d);

        /* T2 */
        if (verbose) printf("--- T2\n");
        reset_log();
        cyp_pulse(0.3); /* first CYP1: sync, engine is at cylinder 1 */
        phase = 0;
        engine(8, 3, 7, 1);
        check_order("T2 3000 rpm: order 1-3-4-2", seq, 8);
        snprintf(d, sizeof d, "dwell %.3f ms", last_off_ms - last_on_ms);
        check("T2 dwell follows IGN (3 ms)", last_off_ms - last_on_ms > 2.95 && last_off_ms - last_on_ms < 3.05, d);

        /* T3: the CYP1 pulse of this cycle stays low across two IGN events */
        if (verbose) printf("--- T3\n");
        reset_log();
        engine(3, 3, 7, 1);
        ign(0); run_ms(3); ign(1); run_ms(6);
        cyp(0);
        run_ms(1);
        phase = 0;
        engine(2, 3, 7, 1);
        cyp(1);
        engine(2, 3, 7, 1);
        check_order("T3 CYP held low across IGN edges: no re-sync", seq, 8);

        /* T4 */
        if (verbose) printf("--- T4\n");
        reset_log();
        engine(8, 3, 7, 0);
        engine(4, 3, 7, 1);
        check_order("T4 CYP lost 2 cycles: order continues", seq, 12);

        /* T5: CYP1 edge inside the dwell of the last cylinder */
        if (verbose) printf("--- T5\n");
        reset_log();
        engine(3, 3, 7, 1);
        ign(0);
        run_ms(1);
        cyp_pulse(0.3);
        run_ms(1.4);
        ign(1);
        run_ms(7);
        phase = 0;
        engine(4, 3, 7, 1);
        check_order("T5 CYP during dwell: next coil is cyl1", seq, 8);

        /* T6 */
        if (verbose) printf("--- T6\n");
        reset_log();
        ign(0);
        run_ms(40);
        ign(1);
        run_ms(10);
        double cut = last_off_ms - last_on_ms;
        snprintf(d, sizeof d, "cut after %.2f ms", cut);
        check("T6 IGN stuck low: safety cut at MAX_DWELL", cut > MAX_DWELL_MS - 0.3 && cut < MAX_DWELL_MS + 0.3, d);
        phase = 1;
        engine(3, 3, 7, 1);
        check_order("T6 order continues after stuck IGN", seq, 4);

        /* T7 */
        if (verbose) printf("--- T7\n");
        reset_log();
        max_latency_us = 0;
        engine(16, 2, 1.75, 1);
        check_order("T7 8000 rpm: 16 sparks, order 1-3-4-2", seq, 16);
        snprintf(d, sizeof d, "max %.1f us", max_latency_us);
        check("T7 spark latency (IGN rise -> coil off)", max_latency_us < MAX_LATENCY_US, d);

        /* T8 */
        if (verbose) printf("--- T8\n");
        reset_log();
        run_ms(1200);
        snprintf(d, sizeof d, "LED toggles %d, coil events %d", led_toggles, norder);
        check("T8 1.2 s without signals: no watchdog reset", led_toggles == 0 && norder == 0, d);

        /* T9: false CYP1 pulse a quarter of the cycle after the real one */
        if (verbose) printf("--- T9\n");
        cyp_pulse(0.3); /* engine restarts at cylinder 1 */
        engine(8, 3, 7, 1);
        reset_log();
        ign(0); run_ms(3); ign(1); run_ms(3);
        cyp_pulse(0.3); /* noise */
        run_ms(3.4);
        phase = 1;
        engine(7, 3, 7, 1);
        check_order("T9 CYP noise early in the cycle: ignored", seq, 8);

        /* T10: ECU skips two ignition events (ignition cut), CYP1 must fix the order */
        if (verbose) printf("--- T10\n");
        reset_log();
        engine(2, 3, 7, 1);
        run_ms(19.4);
        cyp_pulse(0.3);
        phase = 0;
        engine(4, 3, 7, 1);
        static const int t10[] = {1, 3, 1, 3, 4, 2};
        check_order("T10 two IGN events lost: next CYP re-syncs at once", t10, 6);

        /* T11: false CYP1 late in the cycle is accepted, the real one must still win */
        if (verbose) printf("--- T11\n");
        engine(4, 3, 7, 1); /* period known again */
        reset_log();
        engine(2, 3, 7, 1);
        ign(0); run_ms(3); ign(1); run_ms(3);
        cyp_pulse(0.3); /* noise, ~3/4 of the cycle */
        run_ms(3.4);
        phase = 3;
        engine(5, 3, 7, 1);
        static const int t11[] = {1, 3, 4, 1, 1, 3, 4, 2};
        check_order("T11 late CYP noise: one wrong spark, real CYP re-syncs", t11, 8);

        printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "ALL PASSED", failures);
        return failures ? 1 : 0;
}
