#include <avr/io.h>
#include <avr/interrupt.h>

// Set to 0 for Part A or 1 for Part B.
#define USE_RAW_INTERRUPTS 1
#define LCD_RS _BV(PB0)
#define LCD_EN _BV(PB1)

volatile unsigned long pulseCount = 0;
volatile unsigned long lastInt0 = 0;
volatile unsigned long lastInt1 = 0;
volatile bool countEvent = false;
volatile bool resetEvent = false;

static void waitUs(unsigned long us)
{
  unsigned long start = micros();

  while (micros() - start < us)
  {
  }
}

static void pulseEnable()
{
  PORTB |= LCD_EN;
  waitUs(1); // Safely exceed the LCD's 450 ns minimum pulse width.
  PORTB &= ~LCD_EN;
}

static void sendNibble(unsigned char nibble)
{
  PORTD = (PORTD & 0x0F) | (nibble & 0xF0);
  pulseEnable();
}

static void sendByte(unsigned char value, bool isData)
{
  if (isData)
  {
    PORTB |= LCD_RS;
  }
  else
  {
    PORTB &= ~LCD_RS;
  }

  sendNibble(value);
  sendNibble(value << 4);
  waitUs(45);
}

static void lcdCommand(unsigned char command)
{
  sendByte(command, false);

  if (command < 3)
  {
    waitUs(1700);
  }
}

static void lcdData(unsigned char value)
{
  sendByte(value, true);
}

static void lcdPosition(unsigned char column, unsigned char row)
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
  DDRB |= LCD_RS | LCD_EN;
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

// INT0 increments after a 50 ms debounce guard.
static void handleInt0()
{
  unsigned long now = millis();

  if (now - lastInt0 >= 50)
  {
    lastInt0 = now;
    pulseCount++;
    countEvent = true;
  }
}

// INT1 resets after a 50 ms debounce guard.
static void handleInt1()
{
  unsigned long now = millis();

  if (now - lastInt1 >= 50)
  {
    lastInt1 = now;
    pulseCount = 0;
    resetEvent = true;
  }
}

#if USE_RAW_INTERRUPTS
ISR(INT0_vect)
{
  handleInt0();
}

ISR(INT1_vect)
{
  handleInt1();
}
#else
void apiInt0()
{
  handleInt0();
}

void apiInt1()
{
  handleInt1();
}
#endif

void setup()
{
  lcdInit();
  Serial.begin(9600);

  // D2 and D3 inputs with internal pull-ups.
  DDRD &= ~(_BV(PD2) | _BV(PD3));
  PORTD |= _BV(PD2) | _BV(PD3);

#if USE_RAW_INTERRUPTS
  // Falling edge: ISC01:00=10 and ISC11:10=10.
  EICRA = (EICRA & ~(_BV(ISC00) | _BV(ISC10))) |
          _BV(ISC01) | _BV(ISC11);
  EIMSK |= _BV(INT0) | _BV(INT1);
  sei();
#else
  attachInterrupt(digitalPinToInterrupt(2), apiInt0, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), apiInt1, FALLING);
#endif
}

void loop()
{
  static unsigned long displayedCount = ~0UL;
  unsigned long countCopy;
  bool countEventCopy;
  bool resetEventCopy;

  // Atomically copy and clear ISR-shared data.
  cli();
  countCopy = pulseCount;
  countEventCopy = countEvent;
  resetEventCopy = resetEvent;
  countEvent = false;
  resetEvent = false;
  sei();

  if (countCopy != displayedCount)
  {
    displayedCount = countCopy;
    char line[17];
    snprintf(line, sizeof(line), "Count: %-9lu", countCopy);
    lcdPosition(0, 0);
    lcdPrint(line);
  }

  // Serial output stays outside the ISRs.
  if (countEventCopy)
  {
    Serial.print("INT0 count: ");
    Serial.println(countCopy);
  }

  if (resetEventCopy)
  {
    Serial.println("INT1 reset");
  }
}
