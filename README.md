# T-Beam GPS referenced 10 MHz frequency source
It is not a GPSDO but you get a ~10 MHz signal which is measured with GPS accuracy. That measurement than can be used to adjust the frequency of radio receivers.
Due to its squarewaviness and lack of filtering you get many harmonics, which can be used to adjust on many higher frequencies.

The software calculates and shows 3 frequencies:
1. 10 MHz (for shortwave receivers or equipment with 10 MHz reference input)
2. 430 MHz (for 70 cm receivers)
3. 1280 MHz (for 23 cm receivers)

# Features

* Generates a 10 MHz square wave on GPIO 0 (Use a short wire as an antenna)
* Using GPS Timepulse via interrupt to measure the exact frequency of the 10 MHz output
* It is using every 10th pulse to increase accuracy. Calculation of the sliding average uses the last 10 measurements (~100 secs)
* Additionally it shows the 43th and 128th harmonic of the frequency so you can adjust your 70 and 23 cm receivers, too.


![image of board](https://github.com/formtapez/T-Beam_GPS_10MHz_Reference/blob/main/pics/board.jpg?raw=true)
![signal on receiver](https://github.com/formtapez/T-Beam_GPS_10MHz_Reference/blob/main/pics/rx.png?raw=true)


# Dependencies

1. [Arduino software](https://www.arduino.cc)
2. [AXP202X_Library](https://github.com/lewisxhe/AXP202X_Library) (available at Arduino library manager)
3. [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library) (available at Arduino library manager)
4. [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306) library (available at Arduino library manager)


# Buttons

Button | Function
:-----:|:--------
PWR short | power off
PWR long | power on
IO38 | nothing
RST | Reset

# Thanks

I used much code from some amazing dudes:
Thanks to [Bjoerns-TB](https://github.com/Bjoerns-TB/Lora-TTNMapper-T-Beam) who forked [DeuxVis](https://github.com/DeuxVis/Lora-TTNMapper-T-Beam) who forked [sbiermann](https://github.com/sbiermann/Lora-TTNMapper-ESP32), and [smaksimowicz](https://github.com/hottimuc/Lora-TTNMapper-T-Beam) for his great GPS-Autobauding.

