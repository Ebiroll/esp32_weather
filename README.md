# esp32_weather
Collect data from bosh/sparkfun BME280 i2c and internal temperature sensor. Send data to thingspeak.com and go back to sleep.

Here are the reuslts.
https://thingspeak.com/channels/209116
For the dataseries I opened the window, closed the window, used a hair-drier and then just let is sit with the window closed.
Obviously the BME280 driver is not correct. 
https://github.com/BoschSensortec/BME280_driver

I used the si720 temperature humidity board.

https://cdn-learn.adafruit.com/assets/assets/000/035/931/original/Support_Documents_TechnicalDocs_Si7021-A20.pdf

#The si720 board
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-si7021-temperature-plus-humidity-sensor.pdf


#The weather data board
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library