#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

// Register Masks 

// traffic light bit definitions
#define RED_BIT    (1 << PB0) 
#define YEL_BIT    (1 << PB1)
#define GRN_BIT    (1 << PB2)
#define PED_BIT    (1 << PB3)

// 7 segment definitions
#define SEG_F      (1 << PB4)
#define SEG_G      (1 << PB5)
#define SEG_MASK_D 0xF8       // PD3 - PD7 -> segments a - e

// lcd definitions
#define RS_BIT        (1 << PC0)
#define EN_BIT        (1 << PC1)
#define LCD_DATA_MASK 0x3C          // PC2 - PC5 -> D4 - D7

// button definition
#define PEDBTN_BIT (1 << PD2)

// Bare-Metal LCD Driver
void lcd_wait_us(uint16_t us) { 
// function wait the requested number of microseconds
  
    // record starting time
    unsigned long start = micros();
  
  	// continuously check elapsed time (busy-wait)
    while ((unsigned long)(micros() - start) < us) {}
}
void lcd_wait_ms(uint16_t ms) { 
  while (ms--) lcd_wait_us(1000); 
}

void lcd_pulse_en(void) {
    PORTC |= EN_BIT;
    asm volatile("nop\n nop\n nop\n nop\n"); 
    PORTC &= ~EN_BIT;
}

void lcd_send_nibble(uint8_t nibble_top4) {
    uint8_t remapped = (nibble_top4 >> 2) & LCD_DATA_MASK;
    PORTC = (PORTC & ~LCD_DATA_MASK) | remapped;
    lcd_pulse_en();
}
void lcd_command(uint8_t cmd) {
    PORTC &= ~RS_BIT;
    lcd_send_nibble(cmd & 0xF0);
    lcd_send_nibble((cmd << 4) & 0xF0);
    if (cmd == 0x01 || cmd == 0x02) lcd_wait_ms(2); 
    else lcd_wait_us(40);
}
void lcd_data(uint8_t b) {
    PORTC |= RS_BIT;
    lcd_send_nibble(b & 0xF0);
    lcd_send_nibble((b << 4) & 0xF0);
    lcd_wait_us(40);
}
void lcd_init(void) {
    DDRC |= RS_BIT | EN_BIT | LCD_DATA_MASK;
    PORTC &= ~(RS_BIT | EN_BIT);
    lcd_wait_ms(50);
    lcd_send_nibble(0x30); lcd_wait_ms(5);
    lcd_send_nibble(0x30); lcd_wait_us(150);
    lcd_send_nibble(0x30); lcd_wait_us(150);
    lcd_send_nibble(0x20); lcd_wait_us(150);
    lcd_command(0x28);
    lcd_command(0x08);
    lcd_command(0x01);
    lcd_command(0x06);
    lcd_command(0x0C);
}
void lcd_setCursor(uint8_t col, uint8_t row) {
    uint8_t addr = (row == 0 ? 0x00 : 0x40) + col;
    lcd_command(0x80 | addr);
}
void lcd_print(const char *s) { while (*s) lcd_data((uint8_t)*s++); }

// 7-Segment Countdown 
const uint8_t segD[10] PROGMEM = {0xF8,0x30,0xD8,0x78,0x30,0x68,0xE8,0x38,0xF8,0x78};
const uint8_t segB[10] PROGMEM = {0x10,0x00,0x20,0x20,0x30,0x30,0x30,0x00,0x30,0x30};

void showDigit(uint8_t d) {
    if (d > 9) { 
      PORTD &= ~SEG_MASK_D; PORTB &= ~(SEG_F | SEG_G); 
      return; 
    }
    uint8_t dv = pgm_read_byte(&segD[d]);
    uint8_t bv = pgm_read_byte(&segB[d]);
    PORTD = (PORTD & ~SEG_MASK_D) | dv;
    PORTB = (PORTB & ~(SEG_F | SEG_G)) | bv;
}

// Traffic State Machine
enum TState { S_GREEN, S_YELLOW, S_RED, S_PED_CROSS };
void enterState(TState s);         

volatile bool pedRequest = false;

TState state = S_GREEN;
unsigned long stateStart = 0;
const unsigned long T_GREEN = 6000, T_YELLOW = 2500, T_RED = 5000, T_PED = 4000;

unsigned long lastRow1Update = 0;
unsigned long pedBlinkPrev = 0;

void enterState(TState s) {
    state = s;
    stateStart = millis();
    PORTB &= ~(RED_BIT | YEL_BIT | GRN_BIT | PED_BIT);
    switch (s) {
        case S_GREEN:  PORTB |= GRN_BIT; 
                       lcd_setCursor(0, 0); 
                       lcd_print("TRAFFIC: GO  "); 
                       break;
        case S_YELLOW: 
                       PORTB |= YEL_BIT; lcd_setCursor(0, 0); 
                       lcd_print("TRAFFIC: SLOW"); 
                       break;
        case S_RED:    PORTB |= RED_BIT; 
                       lcd_setCursor(0, 0); 
                       lcd_print("TRAFFIC: STOP"); 
                       break;
        case S_PED_CROSS:
                       PORTB |= RED_BIT;                        // Red holds through the crossing
                       lcd_setCursor(0, 0); 
                       lcd_print("PED: CROSSING");
                       break;
    }
}

//  Pedestrian request ISR 
ISR(INT0_vect) {
    if (state != S_PED_CROSS) 
      pedRequest = true;   // ignore requests during active crossing
}

void setup_pedestrian_interrupt(void) {
    DDRD  &= ~PEDBTN_BIT; // configure pedestrian bit as input
    PORTD |=  PEDBTN_BIT; // enable internal pull-up resistor for the pedestrian bit
    // EICRA: ISC01=1, ISC00=0 -> INT0 triggers on falling edge
    EICRA |=  (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    // EIMSK: enable INT0
    EIMSK |= (1 << INT0);
}

void setup() {
  	// configure these PORTB pins as outputs
    DDRB |= RED_BIT | YEL_BIT | GRN_BIT | PED_BIT | SEG_F | SEG_G;
    DDRD |= SEG_MASK_D;

    lcd_init();
    setup_pedestrian_interrupt();
    sei();
    enterState(S_GREEN);
}

void loop() {
    unsigned long now = millis();
    unsigned long elapsed = now - stateStart;

    // State transition logic 
    switch (state) {
        case S_GREEN:
            if (elapsed >= T_GREEN && !pedRequest) {
                enterState(S_YELLOW);
            } else if (pedRequest && elapsed >= 3000) {
                // two cases: 1. request arrived after 3s (immediate),
                // and        2. request arrived before 3s (simply wait here until
                //               elapsed crosses 3000ms, then fire on the very next pass
                pedRequest = false;
                enterState(S_YELLOW);
            }
            break;

        case S_YELLOW:
            if (elapsed >= T_YELLOW) enterState(S_RED);
            break;

        case S_RED:
            if (elapsed >= T_RED) {
                if (pedRequest) {
                    pedRequest = false;
                    enterState(S_PED_CROSS);
                } else {
                    enterState(S_GREEN);
                }
            }
            break;

        case S_PED_CROSS:
            if (elapsed >= T_PED) enterState(S_GREEN);
            break;
    }

    // Pedestrian LED blink 2.5Hz during crossing 
    if (state == S_PED_CROSS) {
        if (now - pedBlinkPrev >= 200) {  // 400ms period -> 200ms half-cycle
            pedBlinkPrev = now;
            PORTB ^= PED_BIT;
        }
    }

    //  LCD row1 + 7-seg countdown every 200ms
    if (now - lastRow1Update >= 200) {
        lastRow1Update = now;
        unsigned long duration = (state == S_GREEN)  ? T_GREEN  :
                                  (state == S_YELLOW) ? T_YELLOW :
                                  (state == S_RED)    ? T_RED    : T_PED;
        long remain = (long)duration - (long)elapsed;
        if (remain < 0) remain = 0;

        int whole = remain / 1000;
        int tenth = (remain % 1000) / 100;
        char buf[17];
        snprintf(buf, sizeof(buf), "Rem Time: %d.%ds  ", whole, tenth);
        lcd_setCursor(0, 1);
        lcd_print(buf);

        uint8_t digit = (uint8_t)((remain * 10) / (long)duration);
        if (digit > 9) digit = 9;
        showDigit(digit);
    }
}
