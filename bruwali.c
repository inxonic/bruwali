#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>


#define BRIGHTNESS_STEP 40


volatile uint8_t mode __attribute__((section(".noinit")));

register uint8_t rot_pos_on __asm__("r2");
register uint8_t rot_pos_inc __asm__("r3");
register uint8_t rot_pos_dec __asm__("r4");


int main(void)
{
    if (++mode > 2 || MCUSR & _BV(PORF))
    {
        PORTB &= ~_BV(0);
        DDRB |= _BV(0);

        MCUSR &= ~_BV(PORF);
        mode = 0;

        cli();
        DDRD |= 0x00;
        TCCR2B &= ~(_BV(CS22) | _BV(CS21) | _BV(CS20));

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    }
    else
    {
        PORTB |= _BV(0);
        DDRB |= _BV(0);

        rot_pos_dec = 1<<0;
        rot_pos_on = 1<<1;
        rot_pos_inc = 1<<2;

        TCCR2B |= _BV(CS21);
        TIMSK2 |= _BV(OCIE2A) | _BV(TOIE2);
        sei();

        DDRD |= 0x1f;

        set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    }
    for (;;)
    {
        sleep_mode();
    }
}

ISR(TIMER2_COMPA_vect) {
    uint8_t mask = rot_pos_on | rot_pos_dec;
    PORTD = PORTD & ~0x1f | mask;
}

ISR(TIMER2_OVF_vect)
{
    uint8_t mask = rot_pos_on | rot_pos_inc;
    PORTD = PORTD & ~0x1f | mask;

    OCR2A += BRIGHTNESS_STEP;

    if ( OCR2A < BRIGHTNESS_STEP ) {
        rot_pos_dec = rot_pos_dec << 1 | (rot_pos_dec & 1<<4) >> 4;
        rot_pos_on = rot_pos_on << 1 | (rot_pos_on & 1<<4) >> 4;
        rot_pos_inc = rot_pos_inc << 1 | (rot_pos_inc & 1<<4) >> 4;
    }
}