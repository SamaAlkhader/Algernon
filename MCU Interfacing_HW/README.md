# ENCS4380 — Microcontroller Interfacing Homework

This project contains the solutions for the five questions of the ENCS4380 Microcontroller Interfacing homework.

The project focuses on direct AVR register programming, non-blocking timing, interrupts, LCD control, and state-machine design using the ATmega328P / Arduino Uno.

## Project Structure

* `Q1_Bare_Metal_LCD`
  Bare-metal HD44780 LCD driver using direct register access and a runtime counter.

* `Q2_Non_Blocking_Patterns`
  Multiple LED timing patterns, button debouncing, long-press detection, and Serial monitoring.

* `Q3_Dynamic_Task_Scheduling`
  Timer-based task scheduling with LEDs, system tick counting, and LCD status updates.

* `Q4_External_Interrupts`
  External interrupt handling using both `attachInterrupt()` and direct AVR interrupt registers.

* `Q5_Traffic_Controller`
  Non-blocking traffic light and pedestrian crossing controller using a state machine.

## Main Features

The project demonstrates:

* Direct use of AVR registers such as `DDRB`, `PORTB`, `PIND`, `EICRA`, and `EIMSK`
* Bit masking for controlling individual pins
* Non-blocking timing using `millis()`
* LCD control without using an LCD library
* Button debouncing
* External hardware interrupts
* Multi-task scheduling
* State-machine based control

## Files

Each question folder contains:

* Arduino `.ino` source code
* Circuit connection information
* Simulation screenshot
* Technical report or technical note

## Simulation

The circuits were designed and tested using **TinkerCAD with an Arduino Uno**.

The code avoids blocking operations where required and mainly uses direct register access instead of high-level Arduino functions.

## Notes

Some questions use the same Arduino pins for different components in the original assignment. Where necessary, alternative pins were used to avoid hardware conflicts while keeping the required functionality.


