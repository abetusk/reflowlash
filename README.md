reflowlash
==========

PID + bitlash = reflowlash, a command line driven reflow oven controller

Uses the MAX31855 thermocouple amplifier (I used Adafruits breakout board for it) and a solid state
relay to operate the reflow oven.

PID_v1 is modified to allow internal state to be displayed from the bitlash command line.

By default, Arduino pins are:

  * MAX31855 CLK  3
  * MAX31855  CS  4
  * MAX31855  DO  5

  * Relay Pin     6


