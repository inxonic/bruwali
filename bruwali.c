#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define AUTO_OFF_TIME 600

#define BRIGHTNESS_STEP_A 40
#define BRIGHTNESS_STEP_B 32

#define FLASH_COUNT 3
#define FLASH_REPEAT 43

static volatile uint8_t mode __attribute__((section(".noinit")));
static uint16_t auto_off_timer;

// https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
uint8_t mcusr_mirror __attribute__((section(".noinit")));

void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void) {
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

int main(void) {
  // Turn off everything we don't need to save energy
  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) |
        _BV(PRADC);

  mode++;

  // Only accept external reset source to activate the device
  if (mcusr_mirror & (_BV(WDRF) | _BV(PORF))) {
    mode = 255;
  }

  // Handle operational modes
  if (mode <= 4) {
    // Enable the step-up converter
    PORTB |= _BV(PORTB0);
    DDRB |= _BV(DDB0);

    auto_off_timer = AUTO_OFF_TIME / 8;

    wdt_reset();
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0);

    cli();

    // Prescaler at 8 lets the timer overflow at 62.5 Hz
    TCCR2B = _BV(CS21);

    if (mode == 1 || mode == 3) {
      GPIOR0 = 0b01100011;
      GPIOR2 |= _BV(0);
      TIMSK2 |= _BV(OCIE2A) | _BV(TOIE2);
    } else {
      GPIOR0 = 0;
      GPIOR2 &= ~_BV(0);
      TIMSK2 |= _BV(TOIE2);
    }

    if (mode == 1 || mode == 2) {
      GPIOR1 = 0b11111001;
      GPIOR2 |= _BV(1);
      TIMSK2 |= _BV(OCIE2B) | _BV(TOIE2);
    } else {
      GPIOR1 = 0;
      GPIOR2 &= ~_BV(1);
      TIMSK2 |= _BV(TOIE2);
    }

    sei();

    DDRB |= 0b11000110;
    DDRC |= 0b00111000;
    DDRD |= 0b11111111;

    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
  }

  // Every other mode value puts the device to the power down mode
  else {
    // Ensure everything is off, though redundant after a reset
    cli();
    TIMSK2 = 0;
    TCCR2B = 0;

    DDRB &= ~_BV(DDB0);
    DDRC = 0;
    DDRD = 0;

    PORTB &= ~_BV(PORTB0);
    PORTC = 0;
    PORTD = 0;

    // Disable the step-up converter
    DDRB = _BV(DDB0);
    PORTB &= ~_BV(PORTB0);

    // Next external reset will start first operational mode
    mode = 0;

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  }

  for (;;) {
    sleep_mode();
  }
}

ISR(WDT_vect) {
  if (--auto_off_timer == 0) {
    // Let the next watchdog timeout reset the device
    wdt_enable(WDTO_15MS);
  }
}

ISR(TIMER2_COMPA_vect) {
  uint8_t a = GPIOR0;

  if (GPIOR2 & _BV(0)) {
    PORTD = a;
    PORTB = PORTB & 0b00111111 | a << 3 & 0b11000000;
  }
}

ISR(TIMER2_COMPB_vect) {
  uint8_t a = GPIOR1;

  PORTB = PORTB & 0b11111001 | a & 0b00000110;
  PORTC = PORTC & 0b11000111 | a & 0b00111000;
}

ISR(TIMER2_OVF_vect) {
  uint8_t a = GPIOR0;

  if (GPIOR2 & _BV(0)) {
    uint8_t b = a << 1 | a >> 4 & 1 << 0;

    PORTD = b;
    PORTB = PORTB & 0b00111111 | b << 3 & 0b11000000;

    if ((OCR2A += BRIGHTNESS_STEP_A) < BRIGHTNESS_STEP_A) {
      GPIOR0 = b;
    }
  } else {
    if (a >> 4 == 6 || (a & 1 << 0) == 0) {
      if (a >> 4 == 6) {
        PORTD = 0;
        PORTB &= ~_BV(PORTB6);
        PORTB &= ~_BV(PORTB7);
        a += 1 << 4;
      }

      if (a >> 7 == 0) {
        if ((a & 1 << 1) == 0) {
          PORTD |= 0b00011111;
        } else {
          PORTD &= ~0b00011111;
        }
      } else {
        if ((a & 1 << 1) == 0) {
          PORTD |= 0b11100000;
          PORTB |= _BV(PORTB6);
          PORTB |= _BV(PORTB7);
        } else {
          PORTD &= ~0b11100000;
          PORTB &= ~_BV(PORTB6);
          PORTB &= ~_BV(PORTB7);
        }
      }
    }
    a += 1 << 4;
    a++;
    GPIOR0 = a;
  }

  uint8_t k = GPIOR1;

  if (GPIOR2 & _BV(1)) {
    uint8_t l = k << 1 | k >> 4 & 1 << 1;

    PORTB = PORTB & 0b11111001 | l & 0b00000110;
    PORTC = PORTC & 0b11000111 | l & 0b00111000;

    if ((OCR2B += BRIGHTNESS_STEP_B) < BRIGHTNESS_STEP_B) {
      GPIOR1 = l;
    }
  } else {
    if ((k & 1 << 0) == 0 && (k >> 2) < FLASH_COUNT) {
      if ((k >> 1 & 1 << 0) == 0) {
        PORTB &= ~0b00000110;
        PORTC &= ~0b00111000;
      } else {
        PORTB |= 0b00000110;
        PORTC |= 0b00111000;
      }
    }
    k++;
    if (k == FLASH_REPEAT) {
      k = 0;
    }
    GPIOR1 = k;
  }
}
