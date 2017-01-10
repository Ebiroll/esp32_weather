# esp32_weather
Collect data from bosh/sparkfun BME280 i2c and internal esp32 temperature sensor. Send data to thingspeak.com and go back to sleep.

Here are the reuslts.
https://thingspeak.com/channels/209116
For the dataseries I opened the window, closed the window, used a hair-drier and then just let is sit with the window closed.

After this test i solved the problem with bootcount being lost.
    esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
That statement was missing in the ecample. Now I will let the board run on a 7800mAh battery and continue 
to update this thingspeak channel https://thingspeak.com/channels/212260
I also added pressure to this feed.

I used the adafruit si7021 temperature humidity board & Sparkfun BME280 board.

https://cdn-learn.adafruit.com/assets/assets/000/035/931/original/Support_Documents_TechnicalDocs_Si7021-A20.pdf

#The si7021 board
https://cdn-learn.adafruit.com/downloads/pdf/adafruit-si7021-temperature-plus-humidity-sensor.pdf


#The BME280 weather data board
https://github.com/sparkfun/SparkFun_BME280_Arduino_Library
https://github.com/BoschSensortec/BME280_driver
