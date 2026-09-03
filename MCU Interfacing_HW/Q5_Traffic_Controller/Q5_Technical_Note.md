# Q5 technical note

PB0–PB3 are masked traffic outputs. INT0 falling-edge configuration is ISC01:ISC00=`10`;
the ISR only records a request and ignores active crossing. A green request is served at or
after 3000 ms; a red request is served after the red interval. Pedestrian blinking toggles every
200 ms (2.5 complete cycles/s). LCD remaining time refreshes every 200 ms. PC0/PC1 replace
the conflicting LCD control pins. The stated seven-segment bonus also conflicts with the LCD
data bus, so it needs a shift register or a separately assigned port on larger hardware.

## Tinkercad simulation

[**Link:** Q4 TinkerCAD_](https://www.tinkercad.com/things/kg6LFf8DQTm/editel?sharecode=wB9QFxOGk0uZg1wIdU91OD3RVpoDlmyBmMnQV7_iYRQ&fbclid=IwcGRvZgNleHRuA2FlbQIxMQBzcnRjBmFwcF9pZA80Mzc2MjYzMTY5NzM3ODgAAR5OJUTRMjs8CkHwkaM4oSKNqzkqt2nWj7M79lKCObd9ubHeAfTai4fTA7fD7g_aem_F19B-JKWwvGKM47YtV151A)
