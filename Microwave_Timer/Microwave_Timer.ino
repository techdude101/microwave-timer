/*
 * Author: Mark McKee
 * Description: 30 Second Timer  
 * Version: 2.0
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>

const int button_30s = 3;
const int button_60s = 4;
const int led_green = 2;
const int buzzer_pin = 0;

enum states {
  IDLE,
  TIMER_RUNNING,
  NOTIFYING
};


volatile uint8_t button_pressed = 0;
volatile enum states state;

// Interrupt vectors
ISR(PCINT0_vect) {
  
}

ISR(WDT_vect) {

}

// End of interrupt vectors

// Function definitions

void sleep_for_1s() {
  setup_watchdog(6);
  enter_sleep();
}

void flashLed(uint16_t time_in_ms, uint8_t led_pin) {
  digitalWrite(led_pin, HIGH);
  delay(time_in_ms);
  digitalWrite(led_pin, LOW);
  delay(time_in_ms);
}

void activeBuzzer(uint16_t time_in_us, uint8_t buzzer_pin, uint16_t duration_in_ms) {
  unsigned long start_time = millis();
  unsigned long end_time = start_time + duration_in_ms;
  while (millis() < end_time) {
    digitalWrite(buzzer_pin, HIGH);
    delayMicroseconds(time_in_us);
    digitalWrite(buzzer_pin, LOW);
    delayMicroseconds(time_in_us);
  }
}

void enter_sleep() {
  // Enable interrupts before entering sleep mode
  GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
  PCMSK |= _BV(PCINT3);                   // Use PB3 as interrupt pin
  PCMSK |= _BV(PCINT4);                   // Use PB4 as interrupt pin
  
  sleep_enable();
  ADCSRA &= ~_BV(ADEN);                   // ADC off ## IMPORTANT ## Saves ~280uA
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  
  sei();                                  // Enable interrupts
  sleep_cpu();
  
  // Wake up from sleep here
  
  uint8_t b1 = PINB & (1 << button_30s);
  uint8_t b2 = PINB & (1 << button_60s);
  if (b1 == 0) {
    button_pressed = 1;
    if (state == IDLE) {
      state = TIMER_RUNNING; 
    } else if (state == TIMER_RUNNING) {
      state = IDLE; 
    } else if (state == NOTIFYING) {
      state = IDLE; 
    }
  } else if (b2 == 0) {
    button_pressed = 2;
    if (state == IDLE) {
      state = TIMER_RUNNING; 
    } else if (state == TIMER_RUNNING) {
      state = IDLE; 
    } else if (state == NOTIFYING) {
      state = IDLE; 
    }
  }
}

// 0 = 16ms, 1 = 32ms,2 = 64ms,3 = 128ms,4 = 250ms,5 = 500ms
// 6 = 1 secs, 7 = 2 secs, 8 = 4 secs, 9 = 8secs
void setup_watchdog(int ii) {
  byte bb;
  int ww;

  if (ii > 9) ii = 9;

  bb = ii & 7;

  if (ii > 7)  {
    bb |= (1 << 5);
  }
  bb |= (1 << WDCE);
  ww = bb;
  MCUSR = 0x00;
  // start timed sequence
  WDTCR |= (1 << WDCE) | (1 << WDE);
  
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// End of function definitions

void setup() {
  pinMode(led_green, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(button_30s, INPUT_PULLUP);
  pinMode(button_60s, INPUT_PULLUP);

  // Beep on startup
  activeBuzzer(250, buzzer_pin, 100);
  digitalWrite(led_green, LOW);
}

void loop() {
  state = IDLE;
  uint8_t delay_time_in_seconds = 0;

  digitalWrite(led_green, LOW);
  activeBuzzer(500, buzzer_pin, 100);
  activeBuzzer(250, buzzer_pin, 100);
  enter_sleep();

  if (button_pressed == 1) {
    activeBuzzer(500, buzzer_pin, 100);
    delay_time_in_seconds = 30;
    digitalWrite(led_green, LOW);
  }
  if (button_pressed == 2) {
    activeBuzzer(250, buzzer_pin, 200);
    delay_time_in_seconds = 60;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(led_green, LOW);
  }

  for (uint8_t i = 0; i < delay_time_in_seconds; i++) {
    sleep_for_1s();
    if (state != TIMER_RUNNING) {
      break;
    }
  }
  // Timer is finished
  state = NOTIFYING;
  
  // Beep until a button is pressed
  while (state == NOTIFYING) {
    digitalWrite(led_green, HIGH);
    activeBuzzer(500, buzzer_pin, 200);
    sleep_for_1s();
  }
  // Disable the watchdog timer
  WDTCR = 0x00;
}
