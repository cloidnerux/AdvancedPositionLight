# AdvancedPositionLight

This is an position light system intended to be used on a quad-copter.
4 strips of 8 WS2812 LEDs are used.
Two just blink to give an indication where the front is. One shows the current battery level and blinks, if it gets critical. The last strip shows the GPS status and how many GPS sats are seen, but it does not show more than 8 statellites.

This uses an Arduino pro micro.

There is also the support for DS18B20 temperature sensors on pin 2. If there is any temeprature exceeding 85°C, a front LED will blink red, also both front leds will blink twice as fast, to indicate over temperature event.

All necessary librarays to build this project can be found in the "APL-librarys.zip"