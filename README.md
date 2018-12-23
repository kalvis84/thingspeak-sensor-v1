# thingspeak-sensor-v1
sensor send data directly to the thing speak channel.

## Changes

## ToDo
* Add switch for switching to SmartConfig (remove automatic switching when WiFi apsence)
* Add power switch to reconect battery (DF robot FireBeatle has problem with reseting ESP32 correctly)
* Add long deepsleep when battery drops to 3.3V. (Add big capacitor to VCC witch will prevent brownout)
* Modify temperature difference required to send data. Change to 0.5°C. It is 2 times resolution
