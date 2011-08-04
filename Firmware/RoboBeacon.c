//Simple navigation beacon for Rovio.
//Paul J. Ste. Marie
//May 2011

#define F_CPU 16000000UL

#include <stdio.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>

// Beacon frequencies:

// clk0 has fewer prescalar options than clk1, so some fiddling is needed to get
// the best matching.  Prescalers and TCNT values, with the desired and actual frequencies,
// are:
//
// Pres0 OCR0A  DFreq0 AFreq0  Err0  Pres1 OCR1B  DFreq1 AFreq1  Err1
// 50Hz
// 64       61    2025 2016.1  0.44%    16   214    2325 2325.5  0.03%  Room 0 (charging station)
//  8      254    3925 3921.5 -0.09%    16   164    3025 3030.3  0.18%  Room 1
//  8      247    4025 4032.2  0.18%    16   159    3125 3125.0  0.00%  Room 2
//  8      241    4125 4132.2  0.18%    16   154    3225 3225.8  0.03%  Room 3
//  8      236    4225 4219.4 -0.13%    16   149    3325 3333.3  0.25%  Room 4
//  8      230    4325 4329.0  0.09%    16   145    3425 3424.6 -0.01%  Room 5
//  8      225    4425 4424.7  0.00%    16   141    3525 3521.1 -0.11%  Room 6
//  8      220    4525 4524.8  0.00%    16   137    3625 3623.1 -0.05%  Room 7
//  8      215    4625 4629.6  0.10%    16   133    3725 3731.3  0.17%  Room 8
//  8      211    4725 4716.9 -0.17%    16   130    3825 3816.7 -0.21%  Room 9
// 60Hz:
// 64       61    2010 2016.1  0.30%    16   164    3030 3030.3  0.01%  Room 0 (charging station)
// 64       59    2070 2083.3  0.64%    16   158    3150 3144.6 -0.17%  Room 1
//  8      239    4170 4166.6 -0.08%    16   155    3210 3205.1 -0.15%  Room 2
//  8      232    4290 4291.8  0.04%    16   149    3330 3333.3  0.10%  Room 3
//  8      226    4410 4405.2 -0.11%    16   144    3450 3448.2 -0.05%  Room 4
//  8      220    4530 4524.8 -0.11%    16   139    3570 3571.4  0.04%  Room 5
//  8      214    4650 4651.1  0.03%    16   135    3690 3676.4 -0.37%  Room 6
//  8      209    4770 4761.9 -0.17%    16   130    3810 3816.7  0.18%  Room 7
//  8      203    4890 4901.9  0.24%     8   253    3930 3937.0  0.18%  Room 8
//  8      199    5010 5000.0 -0.20%     8   246    4050 4048.5 -0.03%  Room 9

struct ClkSettings
{
    uint8_t cs0;
    uint8_t ocr0a;
    uint8_t cs1;
    uint8_t ocr1c;
};

uint16_t EEMEM nv_room_id = 0;
struct ClkSettings EEMEM nv_clock_settings[] = {

#ifdef FIFTY_HZ
//50Hz countries:

    {0b011,  61, 0b0101, 214},  // Room 0: cs0 = 011 for CLK/64, cs1 = 0101 for CLK/16
    {0b010, 254, 0b0101, 164},  // Room 1: cs0 = 010 for CLK/8
    {0b010, 247, 0b0101, 159},  // Room 2
    {0b010, 241, 0b0101, 154},  // Room 3
    {0b010, 236, 0b0101, 149},  // Room 4
    {0b010, 230, 0b0101, 145},  // Room 5
    {0b010, 225, 0b0101, 141},  // Room 6
    {0b010, 220, 0b0101, 137},  // Room 7
    {0b010, 215, 0b0101, 133},  // Room 8
    {0b010, 211, 0b0101, 130},  // Room 9

#else
// 60Hz countries

    {0b011,  61, 0b0101, 164},  // Room 0
    {0b011,  59, 0b0101, 158},  // Room 1
    {0b010, 239, 0b0101, 155},  // Room 2
    {0b010, 232, 0b0101, 149},  // Room 3
    {0b010, 226, 0b0101, 144},  // Room 4
    {0b010, 220, 0b0101, 139},  // Room 5
    {0b010, 214, 0b0101, 135},  // Room 6
    {0b010, 209, 0b0101, 130},  // Room 7
    {0b010, 203, 0b0100, 253},  // Room 8: cs1 = 0100 for CLK/8
    {0b010, 199, 0b0100, 246},  // Room 9
#endif
};

static uint16_t room_id;
static struct ClkSettings clk_settings;
static uint8_t ticks_per_millis;
static volatile uint32_t ticks;

// Count ticks on timer1.  These will allow computing time delays

ISR(TIM1_COMPA_vect)
{
    ++ticks;
}

// Load clock settings from EEPROM

void loadClockSettings(uint16_t id)
{
    if (id > 10) {
        id = 0;
    }
    room_id = id;
    eeprom_read_block(&clk_settings,
                      nv_clock_settings + room_id,
                      sizeof clk_settings);
//     micros_per_tick = clk_settings.ocr1c / (F_CPU / 1000000UL);
//     nanos_per_tick = (1000 * (clk_settings.ocr1c % (F_CPU / 1000000UL)))
// 				        / (F_CPU / 1000000UL);
    ticks_per_millis = (((F_CPU / 1000UL) >> (clk_settings.cs1 - 1))
                        + clk_settings.ocr1c / 2) / clk_settings.ocr1c;
}

// Time calc.  Note that rollover is icky--zero ticks before starting timer.

uint32_t millis(void)
{
    uint8_t oldSREG = SREG;
    cli();
    uint32_t lticks = ticks;
    SREG = oldSREG;
    return lticks / ticks_per_millis;
}

void clearMillis(void)
{
    uint8_t oldSREG = SREG;
    cli();
    ticks = 0;
    SREG = oldSREG;
}

void setPB2In(void)
{
    uint8_t oldSREG = SREG;
    cli();
    PORTB &= ~_BV(PB2);
    DDRB  &= ~_BV(PB2);
    SREG = oldSREG;
}

void setPB2OutHigh(void)
{
    uint8_t oldSREG = SREG;
    cli();
    DDRB  |= _BV(PB2);
    PORTB |= _BV(PB2);
    SREG = oldSREG;
}

void setPB2OutLow(void)
{
    uint8_t oldSREG = SREG;
    cli();
    DDRB  |= _BV(PB2);
    PORTB &= ~_BV(PB2);
    SREG = oldSREG;
}

void wait_for_button_press(void)
{
    setPB2In();
    int n = 0;
    while (n < 100) {
        if (PINB & _BV(PB2)) {
            ++n;
        }
        else {
            n = 0;
        }
        _delay_ms(1);
    }
}

void wait_for_button_release(void)
{
    setPB2In();
    int n = 0;
    while (n < 100) {
        if (PINB & _BV(PB2)) {
            n = 0;
        }
        else {
            n++;
        }
        _delay_ms(1);
    }
}

void blink(uint8_t n)
{
    while (n--) {
        _delay_ms(500);
        setPB2OutHigh();
        _delay_ms(500);
        setPB2OutLow();
    }
    setPB2In();
}

void setRoomID(void)
{
    blink(2);
    uint8_t newID = 0;
    for (;;) {
        wait_for_button_press();
        clearMillis();
        wait_for_button_release();
        if (millis() > 3000UL) {
            break;
        }
        ++newID;
        if (newID >= 10) {
            newID = 0;
        }
    }
    blink(2);
    _delay_ms(2000);
    blink(newID);
    _delay_ms(2000);
    blink(2);
    eeprom_update_word(&nv_room_id, newID);
    loadClockSettings(newID);
}

void shineLights(void)
{
    for (uint8_t i = 0; i < 250; ++i) {
        setPB2OutHigh();
        _delay_ms(120);
        setPB2In();
        if (PINB & _BV(2)) {
            wait_for_button_release();
            break;
        }
    }
    setPB2In();
}

// Setup.  Nothing fancy.

void init(void)
{
    loadClockSettings(eeprom_read_word(&nv_room_id));
    PORTB  = 0;
    DDRB   = _BV(PB0)|_BV(PB1); // pins PB0 (OC0A) and PB1 (OC0B) are outputs
    GTCCR  = 0;
    TCCR0A = _BV(COM0A0) | _BV(WGM01); // toggle OC0A on compare, OC0B
                                       // disconnected, CTC mode timer 0

    TCCR0B = clk_settings.cs0;  // Timer 0 prescaler, CTC mode
                                // timer 0 (WGM02 = 0)
    OCR0A = clk_settings.ocr0a; // Output half-period

    TCCR1 = _BV(CTC1)           // CTC mode timer 1,
        | _BV(COM1A0)           // toggle OC1A on compare
        | clk_settings.cs1;     // Timer 1 prescalar
    OCR1A = 1;                  // Toggle on match
    OCR1C = clk_settings.ocr1c; // Output half-period
    TIMSK = _BV(OCIE1A);        // enable interrupt for OC1A
    sei();
    blink(2);
    _delay_ms(2000);
    blink(room_id);
    _delay_ms(2000);
    blink(2);
}

int main(void)
{
    init();
    for (;;) {
        wait_for_button_press();
        clearMillis();
        wait_for_button_release();
        if (millis() > 3000UL) {
            setRoomID();
        }
        else {
            shineLights();
        }
    }
    return 0;
}

