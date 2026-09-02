// Minimal Timer-compatible scheduler embedded for Tinkercad.
class Timer
{
  struct Job
  {
    unsigned long period;
    unsigned long lastRun;
    void (*callback)();
    unsigned char pin;
    bool oscillates;
    bool used;
    bool state;
  };

  Job jobs[8] = {};

  static void configureOutput(unsigned char pin)
  {
    if (pin <= 7)
      DDRD |= _BV(pin);
    else if (pin <= 13)
      DDRB |= _BV(pin - 8);
    else
      DDRC |= _BV(pin - 14);
  }

  static void writeOutput(unsigned char pin, bool state)
  {
    if (pin <= 7)
    {
      if (state)
        PORTD |= _BV(pin);
      else
        PORTD &= ~_BV(pin);
    }
    else if (pin <= 13)
    {
      if (state)
        PORTB |= _BV(pin - 8);
      else
        PORTB &= ~_BV(pin - 8);
    }
    else
    {
      if (state)
        PORTC |= _BV(pin - 14);
      else
        PORTC &= ~_BV(pin - 14);
    }
  }

public:
  int every(unsigned long period, void (*callback)())
  {
    for (unsigned char i = 0; i < 8; i++)
    {
      if (!jobs[i].used)
      {
        jobs[i].period = period;
        jobs[i].lastRun = millis();
        jobs[i].callback = callback;
        jobs[i].oscillates = false;
        jobs[i].used = true;
        return i;
      }
    }

    return -1;
  }

  int oscillate(unsigned char pin, unsigned long period,
                unsigned char startState)
  {
    configureOutput(pin);
    writeOutput(pin, startState != 0);

    for (unsigned char i = 0; i < 8; i++)
    {
      if (!jobs[i].used)
      {
        jobs[i].period = period;
        jobs[i].lastRun = millis();
        jobs[i].callback = 0;
        jobs[i].pin = pin;
        jobs[i].oscillates = true;
        jobs[i].used = true;
        jobs[i].state = startState != 0;
        return i;
      }
    }

    return -1;
  }

  // Change a live job without deleting or recreating it.
  void setPeriod(int jobId, unsigned long newPeriod)
  {
    if (jobId >= 0 && jobId < 8 && jobs[jobId].used)
    {
      jobs[jobId].period = newPeriod;
    }
  }

  void update()
  {
    unsigned long now = millis();

    for (unsigned char i = 0; i < 8; i++)
    {
      if (!jobs[i].used || now - jobs[i].lastRun < jobs[i].period)
      {
        continue;
      }

      jobs[i].lastRun += jobs[i].period;

      if (jobs[i].oscillates)
      {
        jobs[i].state = !jobs[i].state;
        writeOutput(jobs[i].pin, jobs[i].state);
      }
      else if (jobs[i].callback != 0)
      {
        jobs[i].callback();
      }
    }
  }
};

#define LCD_RS _BV(PB0)
#define LCD_EN _BV(PB1)

Timer ledTimer;
Timer pulseTimer;
Timer tickTimer;

int ledJobId = -1;
int pulseJobId = -1;
int tickJobId = -1;

volatile unsigned long ticks = 0;
bool pulseActive = false;
bool pulseOccurred = false;
bool fastMode = false;
unsigned long pulseStartedAt = 0;
unsigned long fastModeStartedAt = 0;
unsigned long pulseActiveTime = 150;
unsigned long pulseMessageTime = 600;
unsigned long lcdRefreshTime = 250;

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

  // Enter 4-bit mode and configure the display.
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

// Start an LED2 pulse using the current mode's duration.
void triggerPulse()
{
  pulseActive = true;
  pulseOccurred = true;
  pulseStartedAt = millis();
  pulseActiveTime = fastMode ? 75 : 150;
  pulseMessageTime = fastMode ? 300 : 600;
  PORTC |= _BV(PC1);
}

void incrementTick()
{
  ticks++;
}

// Halve or restore all timer periods without replacing timer jobs.
void applyTimerPeriods(bool useFastMode)
{
  fastMode = useFastMode;

  ledTimer.setPeriod(ledJobId, fastMode ? 125 : 250);
  pulseTimer.setPeriod(pulseJobId, fastMode ? 1500 : 3000);
  tickTimer.setPeriod(tickJobId, fastMode ? 500 : 1000);
  lcdRefreshTime = fastMode ? 125 : 250;
}

void setup()
{
  lcdInit();
  DDRC |= _BV(PC0) | _BV(PC1);

  // D3 is the bonus-mode button with internal pull-up.
  DDRD &= ~_BV(PD3);
  PORTD |= _BV(PD3);

  ledJobId = ledTimer.oscillate(A0, 250, HIGH);
  pulseJobId = pulseTimer.every(3000, triggerPulse);
  tickJobId = tickTimer.every(1000, incrementTick);
}

void loop()
{
  ledTimer.update();
  pulseTimer.update();
  tickTimer.update();

  unsigned long now = millis();

  // Debounce the active-low bonus button without blocking.
  static bool rawButton = false;
  static bool stableButton = false;
  static unsigned long buttonChangedAt = 0;
  bool buttonSample = !(PIND & _BV(PD3));

  if (buttonSample != rawButton)
  {
    rawButton = buttonSample;
    buttonChangedAt = now;
  }

  if (rawButton != stableButton && now - buttonChangedAt >= 40)
  {
    stableButton = rawButton;

    // A new press starts or restarts the three-second fast window.
    if (stableButton)
    {
      fastModeStartedAt = now;
      applyTimerPeriods(true);
    }
  }

  if (fastMode && now - fastModeStartedAt >= 3000)
  {
    applyTimerPeriods(false);
  }

  // End LED2 pulse after its active interval.
  if (pulseActive && now - pulseStartedAt >= pulseActiveTime)
  {
    pulseActive = false;
    PORTC &= ~_BV(PC1);
  }

  static unsigned long lastRefresh = 0;

  if (now - lastRefresh >= lcdRefreshTime)
  {
    lastRefresh = now;

    char line[17];
    snprintf(line, sizeof(line), "Ticks: %-9lu", ticks);
    lcdPosition(0, 0);
    lcdPrint(line);
    lcdPosition(0, 1);

    if (pulseActive)
    {
      lcdPrint("PULSE: ACTIVE   ");
    }
    else if (pulseOccurred && now - pulseStartedAt < pulseMessageTime)
    {
      lcdPrint("PULSE: EXPIRED  ");
    }
    else
    {
      lcdPrint("                ");
    }
  }
}
