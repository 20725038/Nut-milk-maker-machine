/**
 * Project : Nut Milk Maker firmware
 * Board   : ATmega328P (Arduino Nano V3), 16 MHz
 * Language: Pure C (avr-gcc, no Arduino core, no C++)
 *
 * Build   : avr-gcc -mmcu=atmega328p -DF_CPU=16000000UL -Os -Wall -o nutmilk.elf main.c -lm
 *           avr-objcopy -O ihex nutmilk.elf nutmilk.hex
 *
 * Feature orders
 *   1  Master Start/Stop (highest priority, stops ALL outputs at once)
 *   2  Timer selection 30..60 min, 5 min steps, wraps to 30
 *   3  Speed 50..100 % in 10 % steps, motor pulses 60 s ON / 30 s OFF
 *   4  Temperature control, 80 C target, heater ON < 77 C, OFF >= 82 C
 *   5  Completion alarm: buzzer 5 s, all outputs off, back to idle
 *   6  Program button: Manual -> Recipe 1 -> Recipe 2 -> Eco-Self Clean -> Manual
 *   7  EEPROM motor run-time counter, REQCLEAN after 10 h, clean cycle resets it
 *   8  Self-test: hold START 5 s -> PASS or FAIL: <module>
 *   9  Workaround modes (as written in User Guide section 10)
 *
 * Pin map
 *   D2 START/STOP   D3 TIME   D4 lid switch (LOW = closed)   D5 PROGRAM   D7 SPEED
 *   D8 heater relay/SSR (HIGH = on)   D9 motor PWM (OC1A, 20 kHz)   D10 buzzer
 *   A0 NTC 100k B3950 (5V -> NTC -> A0 -> 10k -> GND, see NTC_ON_HIGH_SIDE)
 *   A4/A5 I2C LCD 1602 (PCF8574 backpack, address 0x27)
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ======================================================================
 * PINS
 * ====================================================================== */
#define BTN_START_PIN    PD2
#define BTN_TIME_PIN     PD3
#define SAFETY_SW_PIN    PD4
#define BTN_PROGRAM_PIN  PD5
#define BTN_SPEED_PIN    PD7

#define HEATER_PIN       PB0
#define MOTOR_PWM_PIN    PB1
#define BUZZER_PIN       PB2

#define NTC_ADC_CHANNEL  0
#define I2C_SDA_PIN      PC4
#define I2C_SCL_PIN      PC5
#define LCD_I2C_ADDR     0x27

/* ======================================================================
 * PARAMETERS
 * ====================================================================== */
/* Order 2 */
#define TIMER_MIN_MIN        30
#define TIMER_MAX_MIN        60
#define TIMER_STEP_MIN       5
/* Order 3 */
#define SPEED_MIN_PCT        50
#define SPEED_MAX_PCT        100
#define SPEED_STEP_PCT       10
#define PULSE_RUN_MS         60000UL
#define PULSE_PAUSE_MS       30000UL
/* Order 4 (target, low = target - 3, high = target + 2) */
#define TEMP_MANUAL_C        80
/* Order 5 */
#define ALARM_MS             5000UL
/* Order 6 recipes */
#define R1_MIN 45
#define R1_SPD 70
#define R1_TMP 75
#define R2_MIN 50
#define R2_SPD 60
#define R2_TMP 80
/* Order 7 eco clean: 60 C, 25 %, 10 s ON / 50 s OFF, 10 cycles = 10 min */
#define ECO_MIN              10
#define ECO_SPD              25
#define ECO_TMP              60
#define ECO_RUN_MS           10000UL
#define ECO_PAUSE_MS         50000UL
#define CLEAN_LIMIT_MIN      600          /* 10 hours of motor run time */
#define COUNTER_SAVE_MIN     5            /* EEPROM write every 5 min of motor time */
#define EE_ADDR_RUNMIN       ((uint16_t *)0)
#define EE_ADDR_TESTCELL     ((uint8_t *)8)

/* Order 9 timed-heating workaround (NTC failed): 30 s ON, then 5 s ON / 15 s OFF */
#define TIMED_HEAT_FIRST_MS  30000UL
#define TIMED_HEAT_ON_MS     5000UL
#define TIMED_HEAT_OFF_MS    15000UL

/* Buttons */
#define DEBOUNCE_MS          20
#define HOLD_MS              5000UL       /* self-test and workaround activation */

/* NTC */
#define NTC_NOMINAL_OHM      100000.0f
#define NTC_NOMINAL_C        25.0f
#define NTC_B                3950.0f
#define NTC_SERIES_OHM       10000.0f
#define NTC_ON_HIGH_SIDE     1   /* 1: 5V-NTC-A0-10k-GND   0: 5V-10k-A0-NTC-GND */
#define NTC_FAULT_MS         1000UL

/* Self-test limits (calibrate on the prototype) */
#define VCC_MIN_MV           4500
#define VCC_MAX_MV           5500
#define HEATER_TEST_MS       3000UL
#define HEATER_TEST_RISE_C   0.2f

/* ======================================================================
 * TYPES AND STATE
 * ====================================================================== */
typedef enum { ST_IDLE, ST_RUNNING, ST_DONE } State_t;
typedef enum { MODE_MANUAL, MODE_RECIPE1, MODE_RECIPE2, MODE_ECO, MODE_COUNT } Mode_t;

/* Fault flags set by self-test or run-time checks */
#define F_NTC     0x01
#define F_MOTOR   0x02
#define F_SAFETY  0x04
#define F_HEATER  0x08
#define F_BUZZER  0x10
#define F_VCC     0x20
#define F_RAM     0x40
#define F_EEPROM  0x80

static volatile uint32_t g_ticks = 0;

static State_t  g_state = ST_IDLE;
static Mode_t   g_mode  = MODE_MANUAL;

static uint8_t  g_timer_min = TIMER_MIN_MIN;
static uint8_t  g_speed_pct = SPEED_MIN_PCT;
static int16_t  g_temp_low  = TEMP_MANUAL_C - 3;
static int16_t  g_temp_high = TEMP_MANUAL_C + 2;
static uint32_t g_run_ms    = PULSE_RUN_MS;
static uint32_t g_pause_ms  = PULSE_PAUSE_MS;

static uint32_t g_t_start = 0;
static uint32_t g_duration_ms = 0;
static uint32_t g_t_phase = 0;
static uint32_t g_t_done = 0;
static bool     g_motor_active = false;
static bool     g_heater_on = false;

static float    g_temp = 0.0f;
static bool     g_temp_ok = false;
static uint32_t g_ntc_bad_since = 0;
static bool     g_ntc_bad = false;

static uint8_t  g_faults = 0;
static bool     g_wa_timed_heat = false;   /* NTC workaround  */
static bool     g_wa_kettle = false;       /* motor workaround */

static uint16_t g_runmin = 0;              /* motor run minutes since last clean */
static uint32_t g_motor_acc_ms = 0;
static uint32_t g_motor_last = 0;
static bool     g_req_clean = false;

static bool     g_start_handled = false;
static char     g_msg1[17], g_msg2[17];
static uint32_t g_msg_until = 0;

static volatile bool     g_buzz_on = false;
static bool     g_buzz_timed = false;
static uint32_t g_buzz_off_at = 0;

static bool     g_lcd_ok = true;
static char     g_lcd_cache[2][17];

/* ======================================================================
 * TIME BASE: TIMER0, 1 kHz
 * ====================================================================== */
ISR(TIMER0_COMPA_vect) { g_ticks++; }

static uint32_t millis(void)
{
    uint32_t t;
    cli(); t = g_ticks; sei();
    return t;
}

/* ======================================================================
 * BUTTONS (debounced, press / release / hold events)
 * ====================================================================== */
#define EV_PRESS    0x01
#define EV_RELEASE  0x02
#define EV_HOLD     0x04

typedef struct {
    uint8_t  mask;
    bool     raw, down, hold_sent;
    uint32_t t_raw, t_down;
} Btn;

static Btn b_start   = { (1 << BTN_START_PIN),   false, false, false, 0, 0 };
static Btn b_time    = { (1 << BTN_TIME_PIN),    false, false, false, 0, 0 };
static Btn b_program = { (1 << BTN_PROGRAM_PIN), false, false, false, 0, 0 };
static Btn b_speed   = { (1 << BTN_SPEED_PIN),   false, false, false, 0, 0 };
static Btn b_lid     = { (1 << SAFETY_SW_PIN),   false, false, false, 0, 0 };  /* down = lid closed */

static uint8_t btn_update(Btn *b, uint32_t now, uint32_t hold_ms)
{
    uint8_t ev = 0;
    bool raw = !(PIND & b->mask);                 /* active LOW */

    if (raw != b->raw) { b->raw = raw; b->t_raw = now; }
    if (b->raw != b->down && (now - b->t_raw) >= DEBOUNCE_MS) {
        b->down = b->raw;
        if (b->down) { b->t_down = now; b->hold_sent = false; ev |= EV_PRESS; }
        else         { ev |= EV_RELEASE; }
    }
    if (hold_ms && b->down && !b->hold_sent && (now - b->t_down) >= hold_ms) {
        b->hold_sent = true;
        ev |= EV_HOLD;
    }
    return ev;
}

static void btn_init(Btn *b)
{
    b->raw = b->down = !(PIND & b->mask);
    b->hold_sent = false;
    b->t_raw = b->t_down = 0;
}

/* ======================================================================
 * SOFTWARE I2C + LCD1602 (PCF8574 backpack), open-drain
 * ====================================================================== */
static void sda_low(void)  { DDRC |=  (1 << I2C_SDA_PIN); PORTC &= ~(1 << I2C_SDA_PIN); }
static void sda_high(void) { DDRC &= ~(1 << I2C_SDA_PIN); PORTC |=  (1 << I2C_SDA_PIN); }
static void scl_low(void)  { DDRC |=  (1 << I2C_SCL_PIN); PORTC &= ~(1 << I2C_SCL_PIN); }
static void scl_high(void) { DDRC &= ~(1 << I2C_SCL_PIN); PORTC |=  (1 << I2C_SCL_PIN); }
static void i2c_delay(void) { _delay_us(5); }

static void i2c_start(void) { sda_high(); scl_high(); i2c_delay(); sda_low(); i2c_delay(); scl_low(); }
static void i2c_stop(void)  { sda_low(); i2c_delay(); scl_high(); i2c_delay(); sda_high(); i2c_delay(); }

static bool i2c_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) sda_high(); else sda_low();
        i2c_delay(); scl_high(); i2c_delay(); scl_low();
        data <<= 1;
    }
    sda_high(); i2c_delay(); scl_high(); i2c_delay();
    bool ack = !(PINC & (1 << I2C_SDA_PIN));
    scl_low();
    return ack;
}

static bool pcf8574_write(uint8_t data)
{
    i2c_start();
    bool ack = i2c_write_byte((LCD_I2C_ADDR << 1) | 0);
    if (ack) ack = i2c_write_byte(data | 0x08);       /* backlight always on */
    i2c_stop();
    return ack;
}

static void lcd_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t d = (nibble & 0xF0) | rs;
    pcf8574_write(d | 0x04);
    _delay_us(1);
    pcf8574_write(d & ~0x04);
    _delay_us(50);
}

static void lcd_byte(uint8_t v, uint8_t rs)
{
    lcd_nibble(v & 0xF0, rs);
    lcd_nibble((uint8_t)(v << 4), rs);
}

static void lcd_init(void)
{
    _delay_ms(50);
    lcd_nibble(0x30, 0); _delay_ms(5);
    lcd_nibble(0x30, 0); _delay_us(150);
    lcd_nibble(0x30, 0);
    lcd_nibble(0x20, 0);
    lcd_byte(0x28, 0);
    lcd_byte(0x0C, 0);
    lcd_byte(0x06, 0);
    lcd_byte(0x01, 0); _delay_ms(2);
    memset(g_lcd_cache, 0, sizeof g_lcd_cache);
}

/* Probe the display. Returns true if it answers. Re-initialises when it comes back. */
static bool lcd_probe(void)
{
    bool ok = pcf8574_write(0x00);
    if (ok && !g_lcd_ok) lcd_init();
    g_lcd_ok = ok;
    return ok;
}

/* Write one 16-character line, only when its text changed */
static void lcd_line(uint8_t row, const char *s)
{
    char buf[17];
    uint8_t i = 0;
    if (!g_lcd_ok) return;
    while (i < 16 && s[i]) { buf[i] = s[i]; i++; }
    while (i < 16) buf[i++] = ' ';
    buf[16] = 0;
    if (strcmp(buf, g_lcd_cache[row]) == 0) return;
    memcpy(g_lcd_cache[row], buf, 17);
    lcd_byte(0x80 | (row ? 0x40 : 0x00), 0);
    for (i = 0; i < 16; i++) lcd_byte((uint8_t)buf[i], 1);
}

static void show_msg(const char *l1, const char *l2, uint32_t ms)
{
    strncpy(g_msg1, l1, 16); g_msg1[16] = 0;
    strncpy(g_msg2, l2, 16); g_msg2[16] = 0;
    g_msg_until = millis() + ms;
    lcd_line(0, g_msg1);
    lcd_line(1, g_msg2);
}

/* ======================================================================
 * OUTPUTS: BUZZER (Timer2 ISR toggles D10 at ~2 kHz), MOTOR PWM, HEATER
 * ====================================================================== */
ISR(TIMER2_COMPA_vect) { PINB = (1 << BUZZER_PIN); }       /* toggle */

static void buzzer_on(void)
{
    TCCR2A = (1 << WGM21);
    TCCR2B = (1 << CS22);                                   /* clk/64 = 250 kHz */
    OCR2A  = 61;                                            /* 4 kHz toggle -> ~2 kHz tone */
    TCNT2  = 0;
    TIMSK2 |= (1 << OCIE2A);
    g_buzz_on = true;
}

static void buzzer_off(void)
{
    TIMSK2 &= ~(1 << OCIE2A);
    TCCR2B = 0;
    PORTB &= ~(1 << BUZZER_PIN);
    g_buzz_on = false;
    g_buzz_timed = false;
}

static void buzzer_beep(uint16_t ms)
{
    buzzer_on();
    g_buzz_timed = true;
    g_buzz_off_at = millis() + ms;
}

static void buzzer_service(uint32_t now)
{
    if (g_buzz_on && g_buzz_timed && (int32_t)(now - g_buzz_off_at) >= 0) buzzer_off();
}

static void set_motor_percent(uint8_t pct)
{
    if (pct > 100) pct = 100;
    OCR1A = (uint16_t)(((uint32_t)pct * 799UL) / 100UL);
}

static void heater_set(bool on)
{
    if (on) PORTB |= (1 << HEATER_PIN); else PORTB &= ~(1 << HEATER_PIN);
    g_heater_on = on;
}

/* ORDER 1: everything off, immediately */
static void all_outputs_off(void)
{
    heater_set(false);
    set_motor_percent(0);
    buzzer_off();
    g_motor_active = false;
}

/* ======================================================================
 * ADC: NTC TEMPERATURE, SUPPLY VOLTAGE
 * ====================================================================== */
static uint16_t adc_read(void)
{
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC)) { }
    return ADC;
}

static uint16_t adc_ntc_raw(void)
{
    ADMUX = (1 << REFS0) | (NTC_ADC_CHANNEL & 0x07);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 4; i++) sum += adc_read();
    return (uint16_t)(sum / 4);
}

/* returns false when the reading is out of range (open or shorted sensor) */
static bool read_temperature(float *out)
{
    uint16_t raw = adc_ntc_raw();
    if (raw < 8 || raw > 1015) return false;

#if NTC_ON_HIGH_SIDE
    float r = NTC_SERIES_OHM * ((1023.0f / (float)raw) - 1.0f);
#else
    float r = NTC_SERIES_OHM * ((float)raw / (1023.0f - (float)raw));
#endif
    float s = logf(r / NTC_NOMINAL_OHM) / NTC_B + 1.0f / (NTC_NOMINAL_C + 273.15f);
    *out = (1.0f / s) - 273.15f;
    return true;
}

static uint16_t read_vcc_mv(void)
{
    ADMUX = (1 << REFS0) | 0x0E;                            /* 1.1 V bandgap */
    _delay_ms(2);
    (void)adc_read();
    uint16_t raw = adc_read();
    ADMUX = (1 << REFS0) | (NTC_ADC_CHANNEL & 0x07);
    (void)adc_read();
    if (raw == 0) return 0;
    return (uint16_t)(1125300UL / raw);
}

static void sample_temperature(uint32_t now)
{
    static uint32_t last = 0;
    if (now - last < 100) return;
    last = now;

    float t;
    if (read_temperature(&t)) {
        g_temp = t; g_temp_ok = true; g_ntc_bad = false; g_ntc_bad_since = 0;
    } else {
        g_temp_ok = false;
        if (g_ntc_bad_since == 0) g_ntc_bad_since = now ? now : 1;
        if ((now - g_ntc_bad_since) >= NTC_FAULT_MS) g_ntc_bad = true;
    }
}

/* ======================================================================
 * ORDER 7: EEPROM RUN-TIME COUNTER
 * ====================================================================== */
static void counter_load(void)
{
    uint16_t v = eeprom_read_word(EE_ADDR_RUNMIN);
    if (v == 0xFFFF) v = 0;                                 /* blank EEPROM */
    g_runmin = v;
    g_req_clean = (g_runmin >= CLEAN_LIMIT_MIN);
}

static void counter_save(void)
{
    eeprom_update_word(EE_ADDR_RUNMIN, g_runmin);
    g_req_clean = (g_runmin >= CLEAN_LIMIT_MIN);
}

/* flush whole minutes still held in RAM (called on stop and finish) */
static void counter_flush(void)
{
    uint16_t add = (uint16_t)(g_motor_acc_ms / 60000UL);
    if (add) {
        g_runmin += add;
        g_motor_acc_ms -= (uint32_t)add * 60000UL;
        counter_save();
    }
}

static void counter_account(uint32_t now)
{
    uint32_t dt = now - g_motor_last;
    g_motor_last = now;
    if (g_state == ST_RUNNING && g_motor_active) {
        g_motor_acc_ms += dt;
        if (g_motor_acc_ms >= (uint32_t)COUNTER_SAVE_MIN * 60000UL) counter_flush();
    }
}

/* ======================================================================
 * PROGRAMS (ORDER 2, 3, 4, 6, 7)
 * ====================================================================== */
static const char *mode_name(Mode_t m)
{
    switch (m) {
        case MODE_RECIPE1: return "Recipe 1";
        case MODE_RECIPE2: return "Recipe 2";
        case MODE_ECO:     return "Eco-Self Clean";
        default:           return "Manual";
    }
}

static void set_temp_target(int16_t target)
{
    g_temp_low  = target - 3;
    g_temp_high = target + 2;
}

static void apply_mode(Mode_t m)
{
    g_mode = m;
    g_run_ms = PULSE_RUN_MS;
    g_pause_ms = PULSE_PAUSE_MS;
    switch (m) {
        case MODE_RECIPE1:
            g_timer_min = R1_MIN; g_speed_pct = R1_SPD; set_temp_target(R1_TMP); break;
        case MODE_RECIPE2:
            g_timer_min = R2_MIN; g_speed_pct = R2_SPD; set_temp_target(R2_TMP); break;
        case MODE_ECO:
            g_timer_min = ECO_MIN; g_speed_pct = ECO_SPD; set_temp_target(ECO_TMP);
            g_run_ms = ECO_RUN_MS; g_pause_ms = ECO_PAUSE_MS; break;
        default:
            g_timer_min = TIMER_MIN_MIN; g_speed_pct = SPEED_MIN_PCT; set_temp_target(TEMP_MANUAL_C); break;
    }
}

/* Changing time or speed by hand leaves the recipe and returns to Manual
 * (keeps the chosen time/speed, restores the manual 80 C band) */
static void leave_recipe_to_manual(void)
{
    if (g_mode == MODE_RECIPE1 || g_mode == MODE_RECIPE2) {
        g_mode = MODE_MANUAL;
        set_temp_target(TEMP_MANUAL_C);
    }
}

/* ======================================================================
 * RUN CONTROL
 * ====================================================================== */
static void stop_all(void)                                  /* ORDER 1 */
{
    all_outputs_off();
    counter_flush();
    g_state = ST_IDLE;
}

static void fail_text(uint8_t flag, char *out)              /* 16 chars max */
{
    const char *s = "FAIL: ?";
    switch (flag) {
        case F_NTC:    s = "FAIL: NTC";        break;
        case F_MOTOR:  s = "FAIL: MOTOR";      break;
        case F_SAFETY: s = "FAIL: SFTYSWTCH";  break;
        case F_HEATER: s = "FAIL: HEATRRELAY"; break;
        case F_BUZZER: s = "FAIL: BUZZER";     break;
        case F_VCC:    s = "FAIL: VCC";        break;
        case F_RAM:    s = "FAIL: RAM";        break;
        case F_EEPROM: s = "FAIL: EEPROM";     break;
    }
    strcpy(out, s);
}

static void try_start(uint32_t now)
{
    if (!b_lid.down) {                                      /* lid / jug switch must be closed */
        show_msg("CLOSE LID / JUG", "THEN PRESS START", 2500);
        buzzer_beep(150);
        return;
    }
    if ((g_faults & F_NTC) && !g_wa_timed_heat) {
        show_msg("FAIL: NTC", "USE WORKAROUND", 3000);
        return;
    }
    if ((g_faults & F_MOTOR) && !g_wa_kettle) {
        show_msg("FAIL: MOTOR", "USE WORKAROUND", 3000);
        return;
    }
    g_state = ST_RUNNING;
    g_t_start = now;
    g_duration_ms = (uint32_t)g_timer_min * 60000UL;
    g_motor_active = !g_wa_kettle;
    g_t_phase = now;
    g_motor_last = now;
    set_motor_percent(g_motor_active ? g_speed_pct : 0);
    buzzer_beep(100);
}

static void finish_run(uint32_t now)                        /* ORDER 5 */
{
    heater_set(false);
    set_motor_percent(0);
    g_motor_active = false;
    counter_flush();
    if (g_mode == MODE_ECO) {                               /* ORDER 7: clean cycle resets counter */
        g_runmin = 0;
        g_motor_acc_ms = 0;
        counter_save();
    }
    g_state = ST_DONE;
    g_t_done = now;
    buzzer_beep((uint16_t)ALARM_MS);
}

static void run_update(uint32_t now)
{
    /* NTC lost while running: stop heating and the whole run unless workaround is on */
    if (g_ntc_bad && !g_wa_timed_heat) {
        stop_all();
        g_faults |= F_NTC;
        show_msg("FAIL: NTC", "USE WORKAROUND", 4000);
        buzzer_beep(500);
        return;
    }

    if (now - g_t_start >= g_duration_ms) { finish_run(now); return; }

    /* ORDER 3: motor pulsing */
    if (g_wa_kettle) {
        set_motor_percent(0);
        g_motor_active = false;
    } else if (g_motor_active) {
        set_motor_percent(g_speed_pct);
        if (now - g_t_phase >= g_run_ms) {
            g_motor_active = false; g_t_phase = now; set_motor_percent(0);
        }
    } else {
        set_motor_percent(0);
        if (now - g_t_phase >= g_pause_ms) {
            g_motor_active = true; g_t_phase = now; set_motor_percent(g_speed_pct);
        }
    }

    /* ORDER 4 (or ORDER 9 timed heating when the NTC has failed) */
    if (g_wa_timed_heat) {
        uint32_t e = now - g_t_start;
        bool on;
        if (e < TIMED_HEAT_FIRST_MS) on = true;
        else on = (((e - TIMED_HEAT_FIRST_MS) % (TIMED_HEAT_ON_MS + TIMED_HEAT_OFF_MS)) < TIMED_HEAT_ON_MS);
        heater_set(on);
    } else if (g_temp_ok) {
        if (g_temp < (float)g_temp_low)        heater_set(true);
        else if (g_temp >= (float)g_temp_high) heater_set(false);
    } else {
        heater_set(false);                                  /* no valid reading: never heat blind */
    }
}

/* ======================================================================
 * ORDER 8: SELF-TEST (hold START 5 s)
 * ====================================================================== */
/* wait that can be cancelled by the master START/STOP button */
static bool wait_ms(uint32_t ms)
{
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        uint32_t now = millis();
        if (btn_update(&b_start, now, 0) & EV_PRESS) {
            all_outputs_off();
            g_start_handled = true;                         /* ignore the matching release */
            return false;
        }
        buzzer_service(now);
    }
    return true;
}

static bool self_test_ram(void)
{
    static volatile uint8_t buf[48];
    const uint8_t pat[2] = { 0x55, 0xAA };
    for (uint8_t p = 0; p < 2; p++) {
        for (uint8_t i = 0; i < sizeof buf; i++) buf[i] = pat[p];
        for (uint8_t i = 0; i < sizeof buf; i++) if (buf[i] != pat[p]) return false;
    }
    return true;
}

static bool self_test_eeprom(void)
{
    uint8_t keep = eeprom_read_byte(EE_ADDR_TESTCELL);
    bool ok = true;
    eeprom_update_byte(EE_ADDR_TESTCELL, 0xA5);
    if (eeprom_read_byte(EE_ADDR_TESTCELL) != 0xA5) ok = false;
    eeprom_update_byte(EE_ADDR_TESTCELL, 0x5A);
    if (eeprom_read_byte(EE_ADDR_TESTCELL) != 0x5A) ok = false;
    eeprom_update_byte(EE_ADDR_TESTCELL, keep);
    return ok;
}

static void run_self_test(void)
{
    uint8_t f = 0;
    float t0, t1;
    bool ok;

    all_outputs_off();
    show_msg("SELF-TEST", "RUNNING...", 60000);

    /* 1. 5 V supply */
    uint16_t vcc = read_vcc_mv();
    if (vcc < VCC_MIN_MV || vcc > VCC_MAX_MV) f |= F_VCC;
    /* 2. RAM */
    if (!self_test_ram()) f |= F_RAM;
    /* 3. EEPROM */
    if (!self_test_eeprom()) f |= F_EEPROM;
    /* 4. safety (lid / jug) switch must read closed */
    {
        uint32_t t = millis();
        while (millis() - t < 60) btn_update(&b_lid, millis(), 0);
        if (!b_lid.down) f |= F_SAFETY;
    }
    /* 5. NTC */
    ok = read_temperature(&t0);
    if (!ok || t0 < -20.0f || t0 > 120.0f) f |= F_NTC;
    /* 6. buzzer: the user confirms that the beep was heard */
    buzzer_beep(300);
    show_msg("BEEP HEARD?", "PROGRAM = YES", 60000);
    {
        bool heard = false, answered = false;
        uint32_t t = millis();
        while (!answered && millis() - t < 6000) {
            uint32_t now = millis();
            if (btn_update(&b_start, now, 0) & EV_PRESS) {
                all_outputs_off(); g_start_handled = true;
                show_msg("", "", 0);
                return;
            }
            if (btn_update(&b_program, now, 0) & EV_PRESS) { heard = true;  answered = true; }
            if (btn_update(&b_time,    now, 0) & EV_PRESS) { heard = false; answered = true; }
            buzzer_service(now);
        }
        if (!heard) f |= F_BUZZER;
    }
    show_msg("SELF-TEST", "RUNNING...", 60000);
    /* 7. heater relay: short pulse, the temperature must rise (needs NTC) */
    if (!(f & F_NTC)) {
        t0 = g_temp_ok ? g_temp : t0;
        heater_set(true);
        bool cont = wait_ms(HEATER_TEST_MS);
        heater_set(false);
        if (!cont) { show_msg("", "", 0); return; }
        if (read_temperature(&t1) && (t1 - t0) < HEATER_TEST_RISE_C) f |= F_HEATER;
    }
    /* 8. motor PWM: timer output must be running; spin 0.5 s only with lid closed */
    set_motor_percent(25);
    {
        uint16_t a = TCNT1; _delay_us(10); uint16_t b = TCNT1;
        if (a == b || OCR1A != (uint16_t)((25UL * 799UL) / 100UL) || !(TCCR1A & (1 << COM1A1))) f |= F_MOTOR;
    }
    if (!(f & F_SAFETY) && !(f & F_MOTOR)) {
        bool cont = wait_ms(500);
        set_motor_percent(0);
        if (!cont) { show_msg("", "", 0); return; }
    }
    set_motor_percent(0);

    g_faults = f;
    if (!(f & F_NTC))   g_wa_timed_heat = false;            /* sensor repaired */
    if (!(f & F_MOTOR)) g_wa_kettle = false;                /* motor repaired  */

    /* show result: PASS, or each failed module in turn */
    if (f == 0) {
        show_msg("SELF-TEST", "PASS", 4000);
        buzzer_beep(200);
    } else {
        char txt[17];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (f & (1 << bit)) {
                fail_text((uint8_t)(1 << bit), txt);
                show_msg("SELF-TEST", txt, 2500);
                if (!wait_ms(2500)) break;
            }
        }
    }
}

/* ======================================================================
 * ORDER 9: WORKAROUND ACTIVATION (Eco-Self Clean selected, then hold 5 s)
 * ====================================================================== */
static void activate_timed_heat(void)
{
    if (g_faults & F_NTC) {
        g_wa_timed_heat = true;
        show_msg("TIMED HEATING", "ON - STAY NEAR", 3000);
        buzzer_beep(200);
    } else {
        show_msg("NO NTC FAULT", "NOTHING CHANGED", 2000);
    }
}

static void activate_kettle(void)
{
    if (g_faults & F_MOTOR) {
        g_wa_kettle = true;
        show_msg("KETTLE ONLY MODE", "MOTOR DISABLED", 3000);
        buzzer_beep(200);
    } else {
        show_msg("NO MOTOR FAULT", "NOTHING CHANGED", 2000);
    }
}

/* ======================================================================
 * BUTTON HANDLING
 * ====================================================================== */
static void handle_buttons(uint32_t now)
{
    uint8_t ev;

    /* ---------- ORDER 1: master START/STOP is always served first ---------- */
    ev = btn_update(&b_start, now, HOLD_MS);
    if (ev & EV_PRESS) {
        if (g_state != ST_IDLE) {                           /* second press: stop everything NOW */
            stop_all();
            g_start_handled = true;
        } else {
            g_start_handled = false;
        }
    }
    if ((ev & EV_HOLD) && g_state == ST_IDLE && !g_start_handled) {
        g_start_handled = true;
        run_self_test();                                    /* ORDER 8 */
        return;
    }
    if (ev & EV_RELEASE) {
        if (!g_start_handled && !b_start.hold_sent && g_state == ST_IDLE) try_start(now);
        g_start_handled = false;
    }

    /* ---------- the other buttons ---------- */
    ev = btn_update(&b_program, now, 0);
    if ((ev & EV_PRESS) && g_state == ST_IDLE) {            /* ORDER 6 / 7 */
        apply_mode((Mode_t)((g_mode + 1) % MODE_COUNT));
        buzzer_beep(40);
    }

    ev = btn_update(&b_time, now, HOLD_MS);
    if (g_state == ST_IDLE) {
        if ((ev & EV_PRESS) && g_mode != MODE_ECO) {        /* ORDER 2 */
            leave_recipe_to_manual();
            g_timer_min += TIMER_STEP_MIN;
            if (g_timer_min > TIMER_MAX_MIN) g_timer_min = TIMER_MIN_MIN;
            buzzer_beep(40);
        }
        if ((ev & EV_HOLD) && g_mode == MODE_ECO) activate_timed_heat();
    }

    ev = btn_update(&b_speed, now, HOLD_MS);
    if (g_state == ST_IDLE) {
        if ((ev & EV_PRESS) && g_mode != MODE_ECO) {        /* ORDER 3 */
            leave_recipe_to_manual();
            g_speed_pct += SPEED_STEP_PCT;
            if (g_speed_pct > SPEED_MAX_PCT) g_speed_pct = SPEED_MIN_PCT;
            buzzer_beep(40);
        }
        if ((ev & EV_HOLD) && g_mode == MODE_ECO) activate_kettle();
    }

    (void)btn_update(&b_lid, now, 0);
}

/* ======================================================================
 * DISPLAY
 * ====================================================================== */
static void update_display(uint32_t now)
{
    char l1[24], l2[24], tt[6];

    if (!lcd_probe()) return;                               /* ORDER 9: no LCD -> keep working */
    if ((int32_t)(g_msg_until - now) > 0) {
        lcd_line(0, g_msg1);
        lcd_line(1, g_msg2);
        return;
    }

    if (g_temp_ok) snprintf(tt, sizeof tt, "%2d", (int)(g_temp + 0.5f));
    else           strcpy(tt, "--");

    if (g_state == ST_DONE) {
        lcd_line(0, "PROCESS FINISHED");
        lcd_line(1, "ALARM 5 SEC");
        return;
    }

    if (g_state == ST_IDLE) {
        lcd_line(0, mode_name(g_mode));
        if (g_req_clean && ((now / 500) % 2)) {             /* ORDER 7: blinking reminder */
            lcd_line(1, "REQCLEAN");
        } else if (g_wa_timed_heat && ((now / 1000) % 2)) {
            lcd_line(1, "WARN: NO NTC");
        } else if (g_wa_kettle && ((now / 1000) % 2)) {
            lcd_line(1, "KETTLE ONLY");
        } else {
            snprintf(l2, sizeof l2, "%2um %3u%% T:%sC", (unsigned)g_timer_min, (unsigned)g_speed_pct, tt);
            lcd_line(1, l2);
        }
        return;
    }

    /* running */
    uint32_t el = (now - g_t_start) / 1000UL;
    uint32_t tot = g_duration_ms / 1000UL;
    uint32_t rem = (tot > el) ? (tot - el) : 0;
    char tag = (g_mode == MODE_RECIPE1) ? '1' : (g_mode == MODE_RECIPE2) ? '2' : (g_mode == MODE_ECO) ? 'E' : 'M';
    snprintf(l1, sizeof l1, "%c %02u:%02u T:%sC", tag, (unsigned)(rem / 60), (unsigned)(rem % 60), tt);
    lcd_line(0, l1);

    if (g_req_clean && ((now / 500) % 2))              strcpy(l2, "REQCLEAN");
    else if (g_wa_timed_heat && ((now / 1000) % 2))    strcpy(l2, "WARN: NO NTC");
    else if (g_wa_kettle && ((now / 1000) % 2))        strcpy(l2, "KETTLE ONLY");
    else snprintf(l2, sizeof l2, "MOT:%s HEAT:%s", g_motor_active ? "ON " : "OFF", g_heater_on ? "ON" : "OFF");
    lcd_line(1, l2);
}

/* ======================================================================
 * HARDWARE INIT
 * ====================================================================== */
static void init_hardware(void)
{
    DDRB  |=  (1 << HEATER_PIN) | (1 << MOTOR_PWM_PIN) | (1 << BUZZER_PIN);
    PORTB &= ~((1 << HEATER_PIN) | (1 << MOTOR_PWM_PIN) | (1 << BUZZER_PIN));

    DDRD  &= ~((1 << BTN_START_PIN) | (1 << BTN_TIME_PIN) | (1 << SAFETY_SW_PIN) |
               (1 << BTN_PROGRAM_PIN) | (1 << BTN_SPEED_PIN));
    PORTD |=   (1 << BTN_START_PIN) | (1 << BTN_TIME_PIN) | (1 << SAFETY_SW_PIN) |
               (1 << BTN_PROGRAM_PIN) | (1 << BTN_SPEED_PIN);

    /* Timer0: 1 ms tick (16 MHz / 64 / 250) */
    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A  = 249;
    TIMSK0 |= (1 << OCIE0A);

    /* Timer1: fast PWM, TOP = ICR1 = 799 -> 20 kHz on OC1A (D9) */
    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1   = 799;
    OCR1A  = 0;

    /* ADC: AVcc reference, prescaler 128 */
    ADMUX  = (1 << REFS0) | (NTC_ADC_CHANNEL & 0x07);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    sei();

    g_lcd_ok = pcf8574_write(0x00);
    if (g_lcd_ok) lcd_init();

    btn_init(&b_start); btn_init(&b_time); btn_init(&b_program);
    btn_init(&b_speed); btn_init(&b_lid);
}

/* ======================================================================
 * MAIN
 * ====================================================================== */
int main(void)
{
    init_hardware();
    counter_load();
    apply_mode(MODE_MANUAL);
    g_state = ST_IDLE;
    g_motor_last = millis();

    for (;;) {
        uint32_t now = millis();

        handle_buttons(now);                                /* ORDER 1 first */
        buzzer_service(now);
        sample_temperature(now);

        /* lid / jug switch: any time it opens while running, stop everything */
        if (g_state == ST_RUNNING && !b_lid.down) {
            stop_all();
            show_msg("LID OPEN", "MACHINE STOPPED", 3000);
            buzzer_beep(300);
        }

        if (g_state == ST_RUNNING) {
            run_update(now);                                /* ORDER 3, 4, 5 */
        } else if (g_state == ST_DONE) {
            if (now - g_t_done >= ALARM_MS) { buzzer_off(); g_state = ST_IDLE; }
        }

        counter_account(now);                               /* ORDER 7 */

        static uint32_t last_lcd = 0;
        if (now - last_lcd >= 250) { last_lcd = now; update_display(now); }

        _delay_ms(5);
    }
    return 0;
}