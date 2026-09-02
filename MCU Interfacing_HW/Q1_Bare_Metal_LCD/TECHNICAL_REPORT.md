# Q1 technical note

PB0/PB1 drive RS/EN; PD4–PD7 form the masked 4-bit bus. `PORTD=(PORTD&0x0F)|(n&0xF0)`
preserves PD0–PD3. Cold boot sends 0x3 three times, then 0x2, followed by function set,
display control, clear, entry mode, and display-on commands. `micros()` busy waits are used
only for LCD-required settling; the runtime update uses a 100 ms `millis()` schedule.

## Tinkercad simulation

**Link:** _Paste the Tinkercad simulation link here._
