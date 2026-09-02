#include <avr/io.h>

#define RS_BIT _BV(PB0)
#define EN_BIT _BV(PB1)
#define LCD_DATA_MASK 0xF0

// Wait only for timing required by the LCD.
static void waitUs(unsigned long us)
{
  unsigned long start = micros();

  while ((unsigned long)(micros() - start) < us)
  {
  }
}

// Generate the LCD enable pulse.
static void pulseEN()
{
  PORTB |= EN_BIT;
  waitUs(1); // Safely exceed the LCD's 450 ns minimum pulse width.
  PORTB &= ~EN_BIT;
}

// Write one nibble and preserve PD0-PD3.
static void sendNibble(uint8_t nibble)
{
  PORTD = (PORTD & ~LCD_DATA_MASK) | (nibble & LCD_DATA_MASK);
  pulseEN();
}

static void sendByte(uint8_t value, bool isData)
{
  if (isData)
  {
    PORTB |= RS_BIT;
  }
  else
  {
    PORTB &= ~RS_BIT;
  }

  sendNibble(value & 0xF0);
  sendNibble(value << 4);
  waitUs(45);
}

void lcd_command(uint8_t command)
{
  sendByte(command, false);

  // Clear and home need extra execution time.
  if (command == 0x01 || command == 0x02)
  {
    waitUs(1700);
  }
}

void lcd_data(uint8_t value)
{
  sendByte(value, true);
}

void lcd_setCursor(uint8_t column, uint8_t row)
{
  lcd_command(0x80 + (row ? 0x40 : 0x00) + column);
}

void lcd_print(const char *text)
{
  while (*text)
  {
    lcd_data(*text++);
  }
}

void lcd_init()
{
  DDRB |= RS_BIT | EN_BIT;
  DDRD |= LCD_DATA_MASK;
  PORTB &= ~(RS_BIT | EN_BIT);

  // Standard HD44780 4-bit cold-start sequence.
  waitUs(15000);
  sendNibble(0x30);
  waitUs(4100);
  sendNibble(0x30);
  waitUs(100);
  sendNibble(0x30);
  waitUs(45);
  sendNibble(0x20);
  waitUs(45);

  lcd_command(0x28);
  lcd_command(0x08);
  lcd_command(0x01);
  lcd_command(0x06);
  lcd_command(0x0C);
}

void setup()
{
  lcd_init();
  lcd_setCursor(0, 0);
  lcd_print("Algernon - 123XXXX");
}

void loop()
{
  static unsigned long lastRefresh = 0;
  unsigned long now = millis();

  // Refresh without stopping other tasks.
  if ((unsigned long)(now - lastRefresh) >= 100)
  {
    lastRefresh = now;

    char line[17];
    snprintf(line, sizeof(line), "Runtime:%8lums", now);
    lcd_setCursor(0, 1);
    lcd_print(line);
  }
}
