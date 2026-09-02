# Q2 technical note

PB0–PB4 are outputs; PD2 is an input with its PORT bit set for the internal pull-up. Each LED
has its own timestamp and asymmetric interval. Button input must remain unchanged for 40 ms
before the stable state changes. A press toggles D; a hold over 2000 ms switches D every 50 ms.
The bonus comparison is one masked read: `(PORTB & 0x07) == 0x07`.

## Tinkercad simulation

**Link:** _Paste the Tinkercad simulation link here._
