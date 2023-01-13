# ISL28022
This project is a Python class supporting The Curious Electric Co.'s DC Power Monitor implementation of a Renesas ISL28022 chip. This class *is not* written as an installable class, yet.
My use of the Curious Electric board is in a water managment system that I built using a Raspberry PI 3B+, an AdaFruit Feather, and a Seeed XIAO. 

The Feather and XAIO are remote sensor collectors. I use the ISL28022 to monitor the total 12v DC power to the recirculation pump and other devices. I also monitor the 5v DC power with an Adafruit INA260, now obsolete. The difference is the 12v DC supply draws much more power than the Adafruit INA260 is rated.

Links:

https://www.curiouselectric.co.uk/products/dc-power-sensor-i2c

https://github.com/curiouselectric/ISL28022_breakout

https://www.renesas.com/eu/en/document/dst/isl28022-datasheet
