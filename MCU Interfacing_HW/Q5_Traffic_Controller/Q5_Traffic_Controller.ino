#include <avr/io.h>
#include <avr/interrupt.h>

#define RED_LED _BV(PB0)
#define YELLOW_LED _BV(PB1)
#define GREEN_LED _BV(PB2)
#define PED_LED _BV(PB3)
#define LCD_RS _BV(PC0)
#define LCD_EN _BV(PC1)

enum State
{
  GREEN_STATE,
  YELLOW_STATE,
  RED_STATE,
  PED_CROSS_STATE
};

volatile bool pedRequest = false;
volatile State state = GREEN_STATE;
unsigned long stateStartedAt = 0;
unsigned long pedBlinkAt = 0;
bool pedLedState = false;

ISR(INT0_vect)
{
  // Requests during an active crossing are ignored.
  if (state != PED_CROSS_STATE)
  {
    pedRequest = true;
  }
}

static void waitUs(unsigned long us)
{
  unsigned long start = micros();

  while (micros() - start < us)
  {
  }
}

static void pulseEnable()
{
  PORTC |= LCD_EN;
  waitUs(1); // Safely exceed the LCD's 450 ns minimum pulse width.
  PORTC &= ~LCD_EN;
}

static void sendNibble(uint8_t nibble)
{
  PORTD = (PORTD & 0x0F) | (nibble & 0xF0);
  pulseEnable();
}

static void sendByte(uint8_t value, bool isData)
{
  if (isData)
  {
    PORTC |= LCD_RS;
  }
  else
  {
    PORTC &= ~LCD_RS;
  }

  sendNibble(value);
  sendNibble(value << 4);
  waitUs(45);
}

static void lcdCommand(uint8_t command)
{
  sendByte(command, false);

  if (command < 3)
  {
    waitUs(1700);
  }
}

static void lcdData(uint8_t value)
{
  sendByte(value, true);
}

static void lcdPosition(byte column, byte row)
{
  lcdCommand(0x80 + (row ? 0x40 : 0x00) + column);
}

static void lcdPrint(const char *text)
{
  while (*text)
  {
    lcdData(*text++);
  }
}

static void lcdInit()
{
  DDRC |= LCD_RS | LCD_EN;
  DDRD |= 0xF0;

  // Standard 4-bit LCD initialization.
  waitUs(15000);
  sendNibble(0x30);
  waitUs(4100);
  sendNibble(0x30);
  waitUs(100);
  sendNibble(0x30);
  sendNibble(0x20);
  lcdCommand(0x28);
  lcdCommand(0x08);
  lcdCommand(0x01);
  lcdCommand(0x06);
  lcdCommand(0x0C);
}

static unsigned long stateDuration()
{
  if (state == GREEN_STATE)
  {
    return 6000;
  }

  if (state == YELLOW_STATE)
  {
    return 2500;
  }

  if (state == RED_STATE)
  {
    return 5000;
  }

  return 4000;
}

// Enter a state, set its LEDs, and update LCD row 0.
static void enterState(State nextState)
{
  state = nextState;
  stateStartedAt = millis();
  PORTB &= ~(RED_LED | YELLOW_LED | GREEN_LED | PED_LED);

  if (state == GREEN_STATE)
  {
    PORTB |= GREEN_LED;
  }
  else if (state == YELLOW_STATE)
  {
    PORTB |= YELLOW_LED;
  }
  else
  {
    PORTB |= RED_LED;
  }

  lcdPosition(0, 0);

  if (state == GREEN_STATE)
    lcdPrint("TRAFFIC: GO     ");
  else if (state == YELLOW_STATE)
    lcdPrint("TRAFFIC: SLOW   ");
  else if (state == RED_STATE)
    lcdPrint("TRAFFIC: STOP   ");
  else
    lcdPrint("PED: CROSSING   ");
}

void setup()
{
  DDRB |= RED_LED | YELLOW_LED | GREEN_LED | PED_LED;

  // D2 input with internal pull-up.
  DDRD &= ~_BV(PD2);
  PORTD |= _BV(PD2);

  lcdInit();

  // Configure INT0 for a falling edge.
  EICRA = (EICRA & ~_BV(ISC00)) | _BV(ISC01);
  EIMSK |= _BV(INT0);
  sei();

  enterState(GREEN_STATE);
}

void loop()
{
  unsigned long now = millis();
  unsigned long elapsed = now - stateStartedAt;
  unsigned long duration = stateDuration();

  bool requestCopy;
  cli();
  requestCopy = pedRequest;
  sei();

  // Perform non-blocking state transitions.
  if (state == GREEN_STATE && requestCopy && elapsed >= 3000)
  {
    cli();
    pedRequest = false;
    sei();
    enterState(YELLOW_STATE);
  }
  else if (state == GREEN_STATE && elapsed >= duration)
  {
    enterState(YELLOW_STATE);
  }
  else if (state == YELLOW_STATE && elapsed >= duration)
  {
    enterState(RED_STATE);
  }
  else if (state == RED_STATE && elapsed >= duration)
  {
    cli();
    requestCopy = pedRequest;
    pedRequest = false;
    sei();
    enterState(requestCopy ? PED_CROSS_STATE : GREEN_STATE);
  }
  else if (state == PED_CROSS_STATE && elapsed >= duration)
  {
    enterState(GREEN_STATE);
  }

  // A 200 ms toggle produces a 2.5 Hz blink.
  if (state == PED_CROSS_STATE && now - pedBlinkAt >= 200)
  {
    pedBlinkAt = now;
    pedLedState = !pedLedState;

    if (pedLedState)
      PORTB |= PED_LED;
    else
      PORTB &= ~PED_LED;
  }

  static unsigned long lastRefresh = 0;

  if (now - lastRefresh >= 200)
  {
    lastRefresh = now;
    elapsed = now - stateStartedAt;
    duration = stateDuration();
    unsigned long remaining = elapsed < duration ? duration - elapsed : 0;

    char line[17];
    snprintf(line, sizeof(line), "Rem Time:%2lu.%lus",
             remaining / 1000, (remaining % 1000) / 100);
    lcdPosition(0, 1);
    lcdPrint(line);
  }
}
