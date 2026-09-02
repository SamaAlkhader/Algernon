# Q3 technical note

The local Timer-compatible scheduler services the 250 ms oscillator, 3000 ms pulse trigger,
and 1000 ms tick callback. LED2 is cleared 150 ms after its timestamp. The expired message
continues until 600 ms after the trigger (150 ms active + 450 ms expired). LCD refresh is a
separate 250 ms timestamp. A0/A1 replace the contradictory PB0/PB1 LED assignment. The
debounced PD3 bonus button changes the existing timer jobs in place for three seconds: periods
become 125, 1500, and 500 ms, while pulse and LCD intervals are also halved. Job IDs and active
output states are preserved, then the original periods are restored automatically.

## Tinkercad simulation

**Link:** _[Q3 tinkercad - not done yet](https://www.tinkercad.com/things/dQUHsafOqpk/editel?returnTo=%2Fdashboard&sharecode=Lu0zAVMG1I4Uex-t71aERj3TdcQ88Yk6vEzv-pv_fEI)_
