# Q2 - Non-Blocking LED Patterns: Tinkercad Connections

## Parts

- Arduino Uno
- 5 LEDs (the fifth LED is the bonus)
- 5 resistors, 220 ohm each
- 1 pushbutton
- Breadboard and jumper wires

## LED connections

An LED has two legs:

- **Long leg = anode (+)**: connect toward the Arduino output through a resistor.
- **Short leg = cathode (-)**: connect to GND.
- In Tinkercad, the bent/flat side of the LED body marks the cathode.

Connect each LED as follows:

| Purpose | Arduino pin | Complete connection |
|---|---:|---|
| LED A | D8 | D8 - 220 ohm resistor - LED long leg; LED short leg - GND |
| LED B | D9 | D9 - 220 ohm resistor - LED long leg; LED short leg - GND |
| LED C | D10 | D10 - 220 ohm resistor - LED long leg; LED short leg - GND |
| LED D | D11 | D11 - 220 ohm resistor - LED long leg; LED short leg - GND |
| LED E (bonus) | D12 | D12 - 220 ohm resistor - LED long leg; LED short leg - GND |

The resistor can be before or after the LED, but each LED must have its own resistor.

## Pushbutton connection

1. Place the pushbutton across the breadboard's center gap.
2. Connect **one side** of the button to Arduino **D2**.
3. Connect the **opposite side** of the button to Arduino **GND**.
4. Do not add an external resistor. The code enables the Arduino's internal pull-up.

Important: the two legs on the same side of a four-leg pushbutton are already connected
internally. Use legs from opposite sides; otherwise the button will appear permanently pressed.

## Ground wiring

1. Connect one Arduino `GND` pin to the breadboard negative rail.
2. Connect all five LED short legs to that negative rail.
3. Connect the button's ground side to the same negative rail.

## Compact diagram

```text
D8  --- 220 ohm --- LED A ---> GND
D9  --- 220 ohm --- LED B ---> GND
D10 --- 220 ohm --- LED C ---> GND
D11 --- 220 ohm --- LED D ---> GND
D12 --- 220 ohm --- LED E ---> GND   (bonus)

D2  ------- pushbutton -------> GND
```

## Expected behavior

- LED A: 200 ms ON, 800 ms OFF.
- LED B: changes state every 350 ms.
- LED C: starts ON for 150 ms, then stays OFF for 650 ms.
- A normal button press toggles LED D.
- Holding the button longer than 2 seconds makes LED D flash rapidly.
- Bonus LED E turns on only while A, B, and C are all on.
- Open the Serial Monitor at **9600 baud** for the status message every 2.5 seconds.
