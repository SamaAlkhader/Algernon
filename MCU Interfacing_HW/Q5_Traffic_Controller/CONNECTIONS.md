# Q5 connections

- D8, D9, D10, D11 → individual 220 Ω resistors → Red, Yellow, Green, Walk LEDs → GND.
- Pedestrian button: D2 → button → GND (internal pull-up).
- LCD `DB4`, `DB5`, `DB6`, `DB7` → Arduino D4, D5, D6, D7.
- LCD **`RS`→A0 and `E`→A1**; `RW`→GND; LCD `DB0`–`DB3` remain disconnected.
- Connect LCD `GND`, `VCC`, `VO/CON`, and backlight exactly as listed in Q1.

The LCD control remap is required because PB0/PB1 are already the red/yellow outputs.
The optional seven-segment bonus is omitted: PORTD D4–D7 is occupied by the LCD and D2 by INT0.
