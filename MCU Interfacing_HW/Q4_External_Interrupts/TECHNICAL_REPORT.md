# Q4 technical note

For falling edges, INT0 uses ISC01:ISC00=`10` and INT1 uses ISC11:ISC10=`10`.
Read-modify-write clears ISC00/ISC10 and sets ISC01/ISC11; EIMSK sets INT0/INT1.
Each ISR accepts an edge only after 50 ms and sets an event flag. Serial and LCD work occurs
in `loop()`, never inside an ISR. The shared 32-bit count is copied atomically with interrupts off.

## Tinkercad simulation

**Link:** _Paste the Tinkercad simulation link here._
