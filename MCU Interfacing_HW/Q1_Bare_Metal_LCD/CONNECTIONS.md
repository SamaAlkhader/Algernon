# Q1 connections (exact Tinkercad labels)

| Tinkercad LCD pin | Connect to Arduino/circuit |
|---|---|
| `GND` | Arduino GND |
| `VCC` | Arduino 5 V |
| `VO` / `CON` | Middle pin of 10 kΩ potentiometer |
| `RS` | Arduino D8 |
| `RW` | Arduino GND |
| `E` | Arduino D9 |
| `DB0`, `DB1`, `DB2`, `DB3` | Leave disconnected |
| `DB4` | Arduino D4 |
| `DB5` | Arduino D5 |
| `DB6` | Arduino D6 |
| `DB7` | Arduino D7 |
| `LED` / backlight anode | Arduino 5 V through 220 Ω |
| Backlight cathode, if separately shown | Arduino GND |

Connect one outside potentiometer pin to 5 V and the other to GND. If the display shows solid
blocks or blank characters, turn the potentiometer to adjust contrast. Replace
`YOUR NAME - ID` in the sketch.
