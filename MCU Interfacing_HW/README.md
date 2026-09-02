# ENCS4380 Homework 1 - Tinkercad Pack

Open each named `.ino` as a separate Arduino Uno project. Each folder contains a short
`CONNECTIONS.md` and `TECHNICAL_REPORT.md`. Replace the name/ID placeholders in Q1.

## Question folders

- `Q1_Bare_Metal_LCD` — HD44780 LCD driver and runtime counter
- `Q2_Non_Blocking_Patterns` — LED schedules and button debounce
- `Q3_Dynamic_Task_Scheduling` — timer coordination and LCD status
- `Q4_External_Interrupts` — API and register-level interrupt counter
- `Q5_Traffic_Controller` — integrated traffic/pedestrian state machine

## Important assignment conflicts

- Q3 reuses PB0/PB1 (D8/D9) for both LCD RS/EN and LEDs. The supplied build moves the
  two LEDs to PC0/PC1 (A0/A1).
- Q5 reuses PB0/PB1 for both LCD RS/EN and traffic LEDs. The supplied build moves LCD
  RS/EN to PC0/PC1 (A0/A1).
- Tinkercad may not provide third-party `Timer.h`. Q3 therefore embeds its small compatible
  timer scheduler directly in the `.ino` file; no extra library is required.

All timing is rollover-safe (`now - previous >= interval`) and no `delay()`,
`digitalWrite()`, `digitalRead()`, or `pinMode()` is used.
