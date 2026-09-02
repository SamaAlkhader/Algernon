# Q3 - Dynamic Task Scheduling: Tinkercad Connections

## Parts

- Arduino Uno
- 16x2 LCD
- 10 kohm potentiometer
- 2 LEDs
- 2 resistors, 220 ohm each
- 1 pushbutton for the bonus mode
- Breadboard and jumper wires

## 1. Connect power rails

1. Connect Arduino `5V` to the breadboard positive rail.
2. Connect Arduino `GND` to the breadboard negative rail.
3. All GND connections below go to this same negative rail.

## 2. Connect the LCD

Use the exact labels printed on the Tinkercad LCD:

| Tinkercad LCD pin | Connect to |
|---|---|
| `GND` | GND rail |
| `VCC` | 5 V rail |
| `VO` or `CON` | Potentiometer middle pin |
| `RS` | Arduino D8 |
| `RW` | GND rail |
| `E` | Arduino D9 |
| `DB0` | Leave disconnected |
| `DB1` | Leave disconnected |
| `DB2` | Leave disconnected |
| `DB3` | Leave disconnected |
| `DB4` | Arduino D4 |
| `DB5` | Arduino D5 |
| `DB6` | Arduino D6 |
| `DB7` | Arduino D7 |
| Backlight `LED+` | 5 V through a 220 ohm resistor |
| Backlight `LED-` | GND rail |

Some Tinkercad LCDs show only `LED` near the last pins. The backlight anode goes toward
5 V through a resistor, and the backlight cathode goes to GND.

## 3. Connect the contrast potentiometer

The potentiometer has three pins:

1. Connect one outside pin to 5 V.
2. Connect the other outside pin to GND.
3. Connect the middle pin to LCD `VO` or `CON`.

If the LCD is blank or shows solid blocks, rotate the potentiometer during the simulation.

## 4. Connect LED 1

1. Connect Arduino `A0` to one 220 ohm resistor.
2. Connect the resistor to LED 1's long leg (anode).
3. Connect LED 1's short leg (cathode) to GND.

LED 1 uses A0 as a digital output. It should change state every 250 ms.

## 5. Connect LED 2

1. Connect Arduino `A1` to one 220 ohm resistor.
2. Connect the resistor to LED 2's long leg (anode).
3. Connect LED 2's short leg (cathode) to GND.

LED 2 produces one short 150 ms pulse every 3 seconds.

## Compact diagram

```text
LCD RS  ---------- D8
LCD E   ---------- D9
LCD RW  ---------- GND
LCD DB4 ---------- D4
LCD DB5 ---------- D5
LCD DB6 ---------- D6
LCD DB7 ---------- D7

A0 ----- 220 ohm ----- LED 1 long leg
                           LED 1 short leg ----- GND

A1 ----- 220 ohm ----- LED 2 long leg
                           LED 2 short leg ----- GND

D3 -------- bonus pushbutton -------- GND
```

## 5. Connect the bonus mode button

1. Place a pushbutton across the breadboard center gap.
2. Connect one side of the button to Arduino **D3**.
3. Connect the opposite side to **GND**.
4. Do not add a resistor; the code enables the internal pull-up.

Use legs from opposite sides of a four-leg pushbutton. Its two legs on the same side are
already connected internally.

Pressing this button activates double-speed mode for three seconds. Pressing it again while
fast mode is active restarts the three-second window.

## Important pin correction

The assignment assigns D8/D9 to both the LCD control lines and the two LEDs. Those jobs
cannot share the same pins. This build keeps the mandatory LCD on D8/D9 and moves the LEDs
to A0/A1 (AVR PC0/PC1).

## Code setup

Copy the entire `Q3_Dynamic_Task_Scheduling.ino` file into Tinkercad's text-code space.
The timer scheduler is already included at the top of that file. No `Timer.h`, extra tab,
or external library is required.

## Expected simulation result

- LED 1 alternates ON/OFF every 250 ms.
- LCD row 0 shows `Ticks:` and increases once per second.
- Every 3 seconds, LED 2 turns on for only 150 ms, so watch closely.
- During that pulse, LCD row 1 shows `PULSE: ACTIVE`.
- Afterward, LCD row 1 shows `PULSE: EXPIRED` for 450 ms, then clears.
- Press the D3 bonus button: for three seconds, LED1 toggles every 125 ms, LED2 pulses every
  1.5 seconds for 75 ms, ticks increment every 500 ms, and the LCD refreshes every 125 ms.
- After three seconds, all periods return to their normal values automatically.
