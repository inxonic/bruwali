#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

// This is the time in seconds after which the device turns off automatically.
static const int kAutoOffTime = 600;

// These are the step sizes by which the brightness is changed on every timer
// overflow for the tractor and the trailer in revolving light mode. A higher
// value results in a faster rotation speed.
static const int kTractorBrightnessStep = 40;
static const int kTrailerBrightnessStep = 32;

// This is the number of times the trailer light flashes in flashing mode.
static const int kTrailerFlashCount = 3;
// This is the number of timer overflows after which the trailer light starts to
// flash again in flashing mode. Prime numbers should give good results.
static const int kTrailerFlashInterval = 43;

static volatile uint8_t mode __attribute__((section(".noinit")));
static uint16_t auto_off_timer;

// We need to turn off the watchdog during early program startup as it remains
// active after a system reset
// [https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html].
uint8_t mcusr_mirror __attribute__((section(".noinit")));

void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void) {
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

int main(void) {
  // We turn off everything we don't need to save energy.
  PRR = _BV(PRTWI) | _BV(PRTIM0) | _BV(PRTIM1) | _BV(PRSPI) | _BV(PRUSART0) |
        _BV(PRADC);

  mode++;

  // After a watchdog or power on reset the device enters the off state.
  if (mcusr_mirror & (_BV(WDRF) | _BV(PORF))) {
    mode = UINT8_MAX;
  }

  // These are the operational modes.
  if (mode <= 4) {
    // Enable the step-up converter.
    PORTB |= _BV(PORTB0);
    DDRB |= _BV(DDB0);

    // Use the watchdog interrupt with an 8 seconds interval to
    // automatically turn off the device after the specified time.
    auto_off_timer = kAutoOffTime / 8;
    wdt_reset();
    WDTCSR |= _BV(WDCE) | _BV(WDE);
    WDTCSR = _BV(WDIE) | _BV(WDP3) | _BV(WDP0);

    cli();

    // A prescaler value of 8 lets the timer overflow at 62.5 Hz.
    TCCR2B = _BV(CS21);

    if (mode == 1 || mode == 3) {
      // Revolving light on the tractor.
      GPIOR0 = 0b01100011;
      GPIOR2 |= _BV(0);
      TIMSK2 |= _BV(OCIE2A) | _BV(TOIE2);
    } else {
      // Flashing light on the tractor.
      GPIOR0 = 0;
      GPIOR2 &= ~_BV(0);
      TIMSK2 |= _BV(TOIE2);
    }

    if (mode == 1 || mode == 2) {
      // Revolving light on the trailer.
      GPIOR1 = 0b11111001;
      GPIOR2 |= _BV(1);
      TIMSK2 |= _BV(OCIE2B) | _BV(TOIE2);
    } else {
      // Flashing light on the trailer.
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

  // Every other mode value puts the device into the off state.
  else {
    // Ensure everything is off, though this is redundant after a reset.
    cli();
    TIMSK2 = 0;
    TCCR2B = 0;

    DDRB &= ~_BV(DDB0);
    DDRC = 0;
    DDRD = 0;

    PORTB &= ~_BV(PORTB0);
    PORTC = 0;
    PORTD = 0;

    // Disable the step-up converter.
    DDRB = _BV(DDB0);
    PORTB &= ~_BV(PORTB0);

    // The next external reset will put the device into the first operational
    // mode.
    mode = 0;

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  }

  for (;;) {
    sleep_mode();
  }
}

ISR(WDT_vect) {
  if (--auto_off_timer == 0) {
    // The next watchdog timeout will reset the device.
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
    // Revolving light on the tractor.
    uint8_t b = a << 1 | a >> 4 & 1 << 0;

    PORTD = b;
    PORTB = PORTB & 0b00111111 | b << 3 & 0b11000000;

    if ((OCR2A += kTractorBrightnessStep) < kTractorBrightnessStep) {
      GPIOR0 = b;
    }
  } else {
    // Flashing light on the tractor.
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
    GPIOR0 = a + (1 << 4) + 1;
  }

  uint8_t k = GPIOR1;

  if (GPIOR2 & _BV(1)) {
    // Revolving light on the trailer.
    uint8_t l = k << 1 | k >> 4 & 1 << 1;

    PORTB = PORTB & 0b11111001 | l & 0b00000110;
    PORTC = PORTC & 0b11000111 | l & 0b00111000;

    if ((OCR2B += kTrailerBrightnessStep) < kTrailerBrightnessStep) {
      GPIOR1 = l;
    }
  } else {
    // Flashing light on the trailer.
    if ((k & 1 << 0) == 0 && (k >> 2) < kTrailerFlashCount) {
      if ((k >> 1 & 1 << 0) == 0) {
        PORTB &= ~0b00000110;
        PORTC &= ~0b00111000;
      } else {
        PORTB |= 0b00000110;
        PORTC |= 0b00111000;
      }
    }
    k++;
    if (k == kTrailerFlashInterval) {
      k = 0;
    }
    GPIOR1 = k;
  }
}
