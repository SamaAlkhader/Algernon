# Q4 - External Interrupt Counter: Tinkercad Connections

## Parts

- Arduino Uno
- 16x2 LCD
- 10 kohm potentiometer
- 2 pushbuttons
- 1 resistor, 220 ohm, for the LCD backlight
- Breadboard and jumper wires

No LEDs are required for Q4.

## 1. Connect power rails

1. Connect Arduino `5V` to the breadboard positive rail.
2. Connect Arduino `GND` to the breadboard negative rail.
3. Use these same rails for the LCD, potentiometer, and buttons.

## 2. Connect the LCD

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
| Backlight `LED+` | 5 V through 220 ohm resistor |
| Backlight `LED-` | GND rail |

## 3. Connect the contrast potentiometer

1. Connect one outside potentiometer pin to 5 V.
2. Connect the other outside pin to GND.
3. Connect the middle pin to LCD `VO` or `CON`.
4. Turn the potentiometer if the text is blank or only blocks appear.

## 4. Connect Button 1 - increment

1. Place the pushbutton across the breadboard center gap.
2. Connect one side of Button 1 to Arduino **D2** (`INT0`).
3. Connect the opposite side to **GND**.
4. Do not add a resistor; the code enables the internal pull-up.

Pressing Button 1 increases the displayed counter by one.

## 5. Connect Button 2 - reset

1. Place the second pushbutton across the breadboard center gap.
2. Connect one side of Button 2 to Arduino **D3** (`INT1`).
3. Connect the opposite side to **GND**.
4. Do not add a resistor; the code enables the internal pull-up.

Pressing Button 2 resets the displayed counter to zero.

Important: on a four-leg pushbutton, the two legs on the same side are internally connected.
Use legs from opposite sides of the button.

## Compact diagram

```text
BUTTON 1:  D2 -------- pushbutton -------- GND
BUTTON 2:  D3 -------- pushbutton -------- GND

LCD RS  -> D8          LCD DB4 -> D4
LCD E   -> D9          LCD DB5 -> D5
LCD RW  -> GND         LCD DB6 -> D6
                       LCD DB7 -> D7
```

## Select Part A or Part B

At the top of `Q4_External_Interrupts.ino`, use:

```cpp
#define USE_RAW_INTERRUPTS 0
```

for **Part A**, which uses `attachInterrupt()`.

Use:

```cpp
#define USE_RAW_INTERRUPTS 1
```

for **Part B**, which uses `EICRA`, `EIMSK`, `ISR(INT0_vect)`, and `ISR(INT1_vect)`.

The current file is set to Part B.

## Expected simulation behavior

1. Start the simulation and adjust LCD contrast if necessary.
2. The LCD initially displays `Count: 0`.
3. Press Button 1 once; the LCD should display `Count: 1`.
4. Press Button 1 again; it should display `Count: 2`.
5. Press Button 2; it should return to `Count: 0`.
6. Open Serial Monitor at **9600 baud**. It reports each accepted increment or reset.
7. Very fast contact changes within 50 ms are ignored by the debounce guard.
