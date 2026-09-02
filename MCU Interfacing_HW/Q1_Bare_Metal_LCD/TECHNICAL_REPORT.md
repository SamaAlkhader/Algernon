# Question 1 — Bare-Metal HD44780 LCD Driver

## Register Configuration

* The LCD control pins are connected as follows:

  * **RS → PB0**
  * **EN → PB1**
* PB0 and PB1 are configured as outputs using `DDRB`.
* The LCD data pins **D4–D7** are connected to **PD4–PD7**.
* Only the upper four bits of `PORTD` are used for LCD data.
* Bit masking is used to make sure that **PD0–PD3 are not modified**.
* The **RW pin is connected to GND**, so the LCD operates in write-only mode.

## Timing Calculations

The LCD operates in **4-bit mode**, so each byte is transmitted in two parts:

1. Upper 4-bit nibble
2. Lower 4-bit nibble

The **EN pin is pulsed** every time a nibble is sent.

The required LCD timing is handled without using `delay()`:

* Normal LCD commands wait at least **40 µs**.
* The clear-display command waits at least **1.64 ms**.
* The runtime counter is updated using `millis()`.

Using `millis()` allows the program to update the LCD without blocking the execution of other tasks.

## System Behavior

After the LCD is initialized:

* **Row 1** displays the student's name and ID.
* **Row 2** continuously displays the system runtime in milliseconds.

Separate functions are used for:

* Sending LCD commands
* Sending characters/data
* Setting the cursor position
* Printing strings

This keeps the LCD driver organized while using direct register access and bit masking.


## Tinkercad simulation

**Link:** _[Q1 tinkercad](https://www.tinkercad.com/things/hib5CWZqjwU/editel?returnTo=%2Fdashboard&sharecode=5ecKqhgPnlr7C_JtBIThkuo8CR9qMXseZ3hgMkcg4MY)_
