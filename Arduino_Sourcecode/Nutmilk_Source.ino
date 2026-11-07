Here is the updated **pure C** firmware with **Order 6 (Program / Preset Recipe Button)** integrated.

---

### Hardware Pin Mapping Update

To accommodate the new **Program Button**, it is mapped to **`Pin D5` (`PD5`)** using the internal pull-up resistor.

| Component | Arduino Nano Pin | AVR Port/Pin | Connection Details |
| --- | --- | --- | --- |
| **Start / Stop Button** | `Pin D2` | `PD2` | Active LOW (`INPUT_PULLUP`) |
| **Timer Select Button** | `Pin D3` | `PD3` | Active LOW (`INPUT_PULLUP`) |
| **Safety Interlock Switch** | `Pin D4` | `PD4` | Active LOW (`INPUT_PULLUP`)

 |
| **Program (Recipe) Button** | **`Pin D5`** | **`PD5`** | **Active LOW (`INPUT_PULLUP`)** |
| **Speed Select Button** | `Pin D7` | `PD7` | Active LOW (`INPUT_PULLUP`)

 |
| **Heater Output** | `Pin D8` | `PB0` | Relay/SSR Output |
| **48V Motor Speed (PWM)** | `Pin D9` | `PB1` | Timer 1 PWM Output (20 kHz)

 |
| **Buzzer** | `Pin D10` | `PB2` | Timer 2 Tone Generator |
| **NTC Temp Sensor** | `Pin A0` | `PC0` | Analog Input (ADC Channel 0) |
| **I2C LCD 1602** | `Pin A4` / `A5` | `PC4` / `PC5` | Software I2C (`0x27`) |

---

### Key Program Button Logic (Order 6)

* **Button Press State Cycle**:
* **Default / Manual Mode**: Full manual control over Timer, Speed, and Temperature bounds.
* **Click 1 ("Recipe 1")**: Automatically locks settings to **Timer = 45 Mins**, **Speed = 70%**, **Target Temp = 75°C** (Hysteresis bounds: 72°C – 77°C).
* **Click 2 ("Recipe 2")**: Automatically locks settings to **Timer = 50 Mins**, **Speed = 60%**, **Target Temp = 80°C** (Hysteresis bounds: 77°C – 82°C).
* **Click 3**: Resets back to **Manual Mode**.



---

### Pure C Firmware (`main.c`)

```c
/**
 * Project: Nut Milk Maker Firmware
 * Board: ATmega328P (Arduino Nano V3)
 * Target Language: Pure C (AVR-GCC Toolchain)
 * 
 * Order 1: Master Power Toggle (Start/Stop with Highest Priority)
 * Order 2: Variable Timer Selection (30m to 60m in 5m steps)
 * Order 3: Speed Control (50%-100%) & Motor Pulsing (1m RUN / 30s PAUSE)
 * Order 4: Temperature Control (Target 80°C with 77°C - 82°C Hysteresis)
 * Order 5: Process Completion Signal (5-Second Alarm on Timer Timeout)
 * Order 6: Program / Preset Recipe Button (Recipe 1 & Recipe 2 Presets)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

// ==========================================
// PIN & PORT DEFINITIONS (AVR ATmega328P)
// ==========================================
// Inputs (PORTD)
#define BTN_START_PIN    PD2  // Pin D2
#define BTN_TIMER_PIN    PD3  // Pin D3
#define SAFETY_SW_PIN    PD4  // Pin D4
#define BTN_PROGRAM_PIN  PD5  // Pin D5 (Order 6: Program Button)
#define BTN_SPEED_PIN    PD7  // Pin D7

// Outputs (PORTB)
#define HEATER_PIN       PB0  // Pin D8
#define MOTOR_PWM_PIN    PB1  // Pin D9 (OC1A - Timer 1 PWM)
#define BUZZER_PIN       PB2  // Pin D10 (OC2B - Timer 2)

// Analog / I2C (PORTC)
#define NTC_ADC_CHANNEL  0    // Pin A0 (PC0)
#define I2C_SDA_PIN      PC4  // Pin A4
#define I2C_SCL_PIN      PC5  // Pin A5

#define LCD_I2C_ADDR     0x27

// ==========================================
// PARAMETERS & CONSTANTS
// ==========================================
#define TIMER_MIN        30
#define TIMER_MAX        60
#define TIMER_STEP       5

#define SPEED_MIN        50
#define SPEED_MAX        100
#define SPEED_STEP       10

#define MOTOR_RUN_TIME_MS   60000UL
#define MOTOR_PAUSE_TIME_MS 30000UL

#define THERMISTOR_NOMINAL  100000.0f
#define TEMPERATURE_NOMINAL 25.0f
#define B_COEFFICIENT       3950.0f
#define SERIES_RESISTOR     10000.0f

#define DEBOUNCE_DELAY_MS   200
#define ALARM_DURATION_MS   5000UL

// ==========================================
// STATE MACHINE & SYSTEM VARIABLES
// ==========================================
typedef enum {
    STATE_OFF,
    STATE_IDLE,
    STATE_RUNNING,
    STATE_COMPLETED
} SystemState_t;

typedef enum {
    MODE_MANUAL = 0,
    MODE_RECIPE_1 = 1,
    MODE_RECIPE_2 = 2
} ProgramMode_t;

volatile uint32_t g_system_ticks = 0;
volatile bool g_power_on = false;

SystemState_t g_current_state = STATE_OFF;
ProgramMode_t g_program_mode = MODE_MANUAL;

uint8_t g_selected_timer_min = TIMER_MIN;
uint8_t g_selected_speed_pct = SPEED_MIN;

float g_temp_target = 80.0f;
float g_temp_lower_bound = 77.0f;
float g_temp_upper_bound = 82.0f;

bool g_motor_pulsing_active = false;
uint32_t g_motor_pulse_timer = 0;

bool g_heater_state = false;

uint32_t g_process_start_time = 0;
uint32_t g_process_total_duration_ms = 0;
uint32_t g_completion_alarm_start_time = 0;

uint32_t g_last_btn_start_press = 0;
uint32_t g_last_btn_timer_press = 0;
uint32_t g_last_btn_speed_press = 0;
uint32_t g_last_btn_program_press = 0;
uint32_t g_last_lcd_update = 0;

uint8_t g_lcd_backlight = 0x08;

// ==========================================
// SYSTEM MILLISECOND TIMER (TIMER 0)
// ==========================================
ISR(TIMER0_COMPA_vect) {
    g_system_ticks++;
}

uint32_t get_millis(void) {
    uint32_t ms;
    cli();
    ms = g_system_ticks;
    sei();
    return ms;
}

// ==========================================
// SOFTWARE I2C & LCD DRIVER
// ==========================================
static void i2c_delay(void) { _delay_us(5); }

static void i2c_start(void) {
    DDRC |= (1 << I2C_SDA_PIN) | (1 << I2C_SCL_PIN);
    PORTC |= (1 << I2C_SDA_PIN) | (1 << I2C_SCL_PIN);
    i2c_delay();
    PORTC &= ~(1 << I2C_SDA_PIN);
    i2c_delay();
    PORTC &= ~(1 << I2C_SCL_PIN);
}

static void i2c_stop(void) {
    PORTC &= ~(1 << I2C_SDA_PIN);
    i2c_delay();
    PORTC |= (1 << I2C_SCL_PIN);
    i2c_delay();
    PORTC |= (1 << I2C_SDA_PIN);
    i2c_delay();
}

static bool i2c_write_byte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        if (data & 0x80) PORTC |= (1 << I2C_SDA_PIN);
        else PORTC &= ~(1 << I2C_SDA_PIN);
        i2c_delay();
        PORTC |= (1 << I2C_SCL_PIN);
        i2c_delay();
        PORTC &= ~(1 << I2C_SCL_PIN);
        data <<= 1;
    }
    DDRC &= ~(1 << I2C_SDA_PIN);
    PORTC |= (1 << I2C_SDA_PIN);
    i2c_delay();
    PORTC |= (1 << I2C_SCL_PIN);
    i2c_delay();
    bool ack = !(PINC & (1 << I2C_SDA_PIN));
    PORTC &= ~(1 << I2C_SCL_PIN);
    DDRC |= (1 << I2C_SDA_PIN);
    return ack;
}

static void pcf8574_write(uint8_t data) {
    i2c_start();
    i2c_write_byte((LCD_I2C_ADDR << 1) | 0);
    i2c_write_byte(data | g_lcd_backlight);
    i2c_stop();
}

static void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble & 0xF0) | mode | g_lcd_backlight;
    pcf8574_write(data | 0x04);
    _delay_us(1);
    pcf8574_write(data & ~0x04);
    _delay_us(50);
}

static void lcd_send_byte(uint8_t value, uint8_t mode) {
    lcd_send_nibble(value & 0xF0, mode);
    lcd_send_nibble((value << 4) & 0xF0, mode);
}

static void lcd_command(uint8_t cmd) { lcd_send_byte(cmd, 0); }
static void lcd_data(uint8_t data) { lcd_send_byte(data, 0x01); }

void lcd_init(void) {
    _delay_ms(50);
    lcd_send_nibble(0x30, 0);
    _delay_ms(5);
    lcd_send_nibble(0x30, 0);
    _delay_us(150);
    lcd_send_nibble(0x30, 0);
    lcd_send_nibble(0x20, 0);

    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_clear(void) {
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t offsets[] = {0x00, 0x40};
    lcd_command(0x80 | (col + offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_data((uint8_t)(*str++));
    }
}

// ==========================================
// HARDWARE INITIALIZATION
// ==========================================
void init_hardware(void) {
    DDRB |= (1 << HEATER_PIN) | (1 << MOTOR_PWM_PIN) | (1 << BUZZER_PIN);
    PORTB &= ~((1 << HEATER_PIN) | (1 << MOTOR_PWM_PIN) | (1 << BUZZER_PIN));

    DDRD &= ~((1 << BTN_START_PIN) | (1 << BTN_TIMER_PIN) | (1 << SAFETY_SW_PIN) | (1 << BTN_PROGRAM_PIN) | (1 << BTN_SPEED_PIN));
    PORTD |= (1 << BTN_START_PIN) | (1 << BTN_TIMER_PIN) | (1 << SAFETY_SW_PIN) | (1 << BTN_PROGRAM_PIN) | (1 << BTN_SPEED_PIN);

    TCCR0A = (1 << WGM01);
    TCCR0B = (1 << CS01) | (1 << CS00);
    OCR0A = 249;
    TIMSK0 |= (1 << OCIE0A);

    TCCR1A = (1 << COM1A1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
    ICR1 = 799;
    OCR1A = 0;

    ADMUX = (1 << REFS0) | (NTC_ADC_CHANNEL & 0x07);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    sei();

    lcd_init();
    g_lcd_backlight = 0x00;
    pcf8574_write(0x00);
}

// ==========================================
// BUZZER & MOTOR SPEED CONTROL
// ==========================================
void sound_buzzer(uint16_t frequency, uint16_t duration_ms) {
    if (frequency == 0) return;
    TCCR2A = (1 << COM2B0) | (1 << WGM21);
    uint32_t ocr = (16000000UL / (2UL * 256UL * frequency)) - 1;
    if (ocr > 255) ocr = 255;
    OCR2A = (uint8_t)ocr;
    TCCR2B = (1 << CS22) | (1 << CS21);

    uint32_t start = get_millis();
    while (get_millis() - start < duration_ms);

    TCCR2A = 0;
    TCCR2B = 0;
    PORTB &= ~(1 << BUZZER_PIN);
}

void sound_buzzer_start_continuous(uint16_t frequency) {
    TCCR2A = (1 << COM2B0) | (1 << WGM21);
    uint32_t ocr = (16000000UL / (2UL * 256UL * frequency)) - 1;
    if (ocr > 255) ocr = 255;
    OCR2A = (uint8_t)ocr;
    TCCR2B = (1 << CS22) | (1 << CS21);
}

void sound_buzzer_stop(void) {
    TCCR2A = 0;
    TCCR2B = 0;
    PORTB &= ~(1 << BUZZER_PIN);
}

void set_motor_speed_percent(uint8_t percent) {
    if (percent > 100) percent = 100;
    uint32_t duty = ((uint32_t)percent * 799) / 100;
    OCR1A = (uint16_t)duty;
}

// ==========================================
// NTC THERMISTOR READING
// ==========================================
float read_temperature(void) {
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));

    uint16_t raw_adc = ADC;
    if (raw_adc <= 5 || raw_adc >= 1018) return 0.0f;

    float resistance = SERIES_RESISTOR * ((1023.0f / (float)raw_adc) - 1.0f);
    float steinhart = resistance / THERMISTOR_NOMINAL;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);
    steinhart = 1.0f / steinhart;
    steinhart -= 273.15f;

    return steinhart;
}

// ==========================================
// EMERGENCY SHUTDOWN
// ==========================================
void emergency_shutdown(void) {
    PORTB &= ~(1 << HEATER_PIN);
    set_motor_speed_percent(0);
    sound_buzzer_stop();

    g_current_state = STATE_OFF;
    g_motor_pulsing_active = false;
    g_heater_state = false;

    if (!g_power_on) {
        g_lcd_backlight = 0x00;
        lcd_clear();
        pcf8574_write(0x00);
    }
}

// ==========================================
// DISPLAY MANAGEMENT
// ==========================================
void update_display(float current_temp) {
    if (!g_power_on) return;

    char buffer[17];
    lcd_set_cursor(0, 0);

    if (g_current_state == STATE_COMPLETED) {
        lcd_print(" PROCESS FINISH ");
        lcd_set_cursor(0, 1);
        lcd_print(" ALARM (5 SECS) ");
        return;
    }

    if (g_current_state == STATE_IDLE) {
        if (g_program_mode == MODE_RECIPE_1) {
            lcd_print("MODE: Recipe 1  ");
        } else if (g_program_mode == MODE_RECIPE_2) {
            lcd_print("MODE: Recipe 2  ");
        } else {
            lcd_print("MODE: Manual    ");
        }

        lcd_set_cursor(0, 1);
        snprintf(buffer, sizeof(buffer), "%2dm %2d%% T:%2dC/ Target:%2dC", 
                 g_selected_timer_min, g_selected_speed_pct, (int)current_temp, (int)g_temp_target);
        
        // Compact 16-char display line 2 format
        snprintf(buffer, sizeof(buffer), "%2dm %2d%% T:%2dC READY", 
                 g_selected_timer_min, g_selected_speed_pct, (int)current_temp);
        lcd_print(buffer);
    } else if (g_current_state == STATE_RUNNING) {
        uint32_t elapsed_sec = (get_millis() - g_process_start_time) / 1000;
        uint32_t total_sec = g_process_total_duration_ms / 1000;
        uint32_t remain_sec = (total_sec > elapsed_sec) ? (total_sec - elapsed_sec) : 0;

        if (g_program_mode == MODE_RECIPE_1) {
            snprintf(buffer, sizeof(buffer), "R1 %02ld:%02ld T:%2dC ", remain_sec / 60, remain_sec % 60, (int)current_temp);
        } else if (g_program_mode == MODE_RECIPE_2) {
            snprintf(buffer, sizeof(buffer), "R2 %02ld:%02ld T:%2dC ", remain_sec / 60, remain_sec % 60, (int)current_temp);
        } else {
            snprintf(buffer, sizeof(buffer), "R: %02ld:%02ld T:%2dC ", remain_sec / 60, remain_sec % 60, (int)current_temp);
        }
        lcd_print(buffer);

        lcd_set_cursor(0, 1);
        snprintf(buffer, sizeof(buffer), "%s %s", 
                 g_motor_pulsing_active ? "MOT:ON " : "MOT:OFF", 
                 g_heater_state ? "HEAT:ON " : "HEAT:OFF");
        lcd_print(buffer);
    }
}

// ==========================================
// USER INPUT HANDLERS
// ==========================================
void handle_master_start_stop(void) {
    uint32_t now = get_millis();

    if (!(PIND & (1 << BTN_START_PIN)) && (now - g_last_btn_start_press > DEBOUNCE_DELAY_MS)) {
        g_last_btn_start_press = now;
        g_power_on = !g_power_on;

        if (g_power_on) {
            g_lcd_backlight = 0x08;
            lcd_clear();
            lcd_set_cursor(0, 0);
            lcd_print(" Nut Milk Maker ");
            lcd_set_cursor(0, 1);
            lcd_print("   POWER ON    ");
            sound_buzzer(1500, 150);

            g_program_mode = MODE_MANUAL;
            g_selected_timer_min = TIMER_MIN;
            g_selected_speed_pct = SPEED_MIN;
            g_temp_target = 80.0f;
            g_temp_lower_bound = 77.0f;
            g_temp_upper_bound = 82.0f;

            g_current_state = STATE_RUNNING;
            g_process_start_time = get_millis();
            g_process_total_duration_ms = (uint32_t)g_selected_timer_min * 60000UL;
            g_motor_pulse_timer = get_millis();
            g_motor_pulsing_active = true;

            _delay_ms(1000);
            lcd_clear();
        } else {
            sound_buzzer(800, 300);
            emergency_shutdown();
        }
    }
}

// ORDER 6: PROGRAM BUTTON HANDLER
void handle_program_select(void) {
    uint32_t now = get_millis();

    if (!(PIND & (1 << BTN_PROGRAM_PIN)) && (now - g_last_btn_program_press > DEBOUNCE_DELAY_MS)) {
        g_last_btn_program_press = now;

        if (g_program_mode == MODE_MANUAL) {
            // ORDER 6: Click 1 -> Recipe 1 (45m, 70% Speed, 75°C)
            g_program_mode = MODE_RECIPE_1;
            g_selected_timer_min = 45;
            g_selected_speed_pct = 70;
            g_temp_target = 75.0f;
            g_temp_lower_bound = 72.0f;
            g_temp_upper_bound = 77.0f;
        } else if (g_program_mode == MODE_RECIPE_1) {
            // ORDER 6: Click 2 -> Recipe 2 (50m, 60% Speed, 80°C)
            g_program_mode = MODE_RECIPE_2;
            g_selected_timer_min = 50;
            g_selected_speed_pct = 60;
            g_temp_target = 80.0f;
            g_temp_lower_bound = 77.0f;
            g_temp_upper_bound = 82.0f;
        } else {
            // Click 3 -> Reset to Manual Mode
            g_program_mode = MODE_MANUAL;
            g_selected_timer_min = TIMER_MIN;
            g_selected_speed_pct = SPEED_MIN;
            g_temp_target = 80.0f;
            g_temp_lower_bound = 77.0f;
            g_temp_upper_bound = 82.0f;
        }

        sound_buzzer(1600, 100);
    }
}

void handle_timer_select(void) {
    uint32_t now = get_millis();

    if (!(PIND & (1 << BTN_TIMER_PIN)) && (now - g_last_btn_timer_press > DEBOUNCE_DELAY_MS)) {
        g_last_btn_timer_press = now;
        g_program_mode = MODE_MANUAL; // Switch to manual mode on override

        g_selected_timer_min += TIMER_STEP;
        if (g_selected_timer_min > TIMER_MAX) {
            g_selected_timer_min = TIMER_MIN;
        }

        sound_buzzer(1200, 80);
    }
}

void handle_speed_select(void) {
    uint32_t now = get_millis();

    if (!(PIND & (1 << BTN_SPEED_PIN)) && (now - g_last_btn_speed_press > DEBOUNCE_DELAY_MS)) {
        g_last_btn_speed_press = now;
        g_program_mode = MODE_MANUAL; // Switch to manual mode on override

        g_selected_speed_pct += SPEED_STEP;
        if (g_selected_speed_pct > SPEED_MAX) {
            g_selected_speed_pct = SPEED_MIN;
        }

        sound_buzzer(1400, 80);
    }
}

void update_motor_pulse_logic(void) {
    uint32_t now = get_millis();
    uint32_t pulse_elapsed = now - g_motor_pulse_timer;

    if (g_motor_pulsing_active) {
        set_motor_speed_percent(g_selected_speed_pct);

        if (pulse_elapsed >= MOTOR_RUN_TIME_MS) {
            g_motor_pulsing_active = false;
            g_motor_pulse_timer = now;
            set_motor_speed_percent(0);
        }
    } else {
        set_motor_speed_percent(0);

        if (pulse_elapsed >= MOTOR_PAUSE_TIME_MS) {
            g_motor_pulsing_active = true;
            g_motor_pulse_timer = now;
            set_motor_speed_percent(g_selected_speed_pct);
        }
    }
}

void update_heater_temperature_control(float current_temp) {
    if (current_temp < g_temp_lower_bound) {
        PORTB |= (1 << HEATER_PIN);
        g_heater_state = true;
    } else if (current_temp >= g_temp_upper_bound) {
        PORTB &= ~(1 << HEATER_PIN);
        g_heater_state = false;
    }
}

// ==========================================
// MAIN ENTRY POINT
// ==========================================
int main(void) {
    init_hardware();

    while (1) {
        // ORDER 1: Highest Priority Master Power Toggle
        handle_master_start_stop();

        if (!g_power_on) {
            emergency_shutdown();
            continue;
        }

        float current_temperature = read_temperature();

        if (g_current_state == STATE_IDLE) {
            handle_program_select(); // ORDER 6
            handle_timer_select();   // ORDER 2
            handle_speed_select();   // ORDER 3
        }

        if (g_current_state == STATE_RUNNING) {
            uint32_t elapsed_total = get_millis() - g_process_start_time;

            // ORDER 5: Timeout completion check
            if (elapsed_total >= g_process_total_duration_ms) {
                g_current_state = STATE_COMPLETED;
                g_completion_alarm_start_time = get_millis();

                PORTB &= ~(1 << HEATER_PIN);
                set_motor_speed_percent(0);
                g_heater_state = false;
                g_motor_pulsing_active = false;

                sound_buzzer_start_continuous(2000);
            } else {
                update_motor_pulse_logic();                            // ORDER 3
                update_heater_temperature_control(current_temperature); // ORDER 4
            }
        }

        // ORDER 5: Handle 5-Second Completion Alarm
        if (g_current_state == STATE_COMPLETED) {
            if (get_millis() - g_completion_alarm_start_time >= ALARM_DURATION_MS) {
                sound_buzzer_stop();
                g_current_state = STATE_IDLE;
            }
        }

        // Refresh Display
        if (get_millis() - g_last_lcd_update >= 250) {
            g_last_lcd_update = get_millis();
            update_display(current_temperature);
        }

        _delay_ms(10);
    }

    return 0;
}

```