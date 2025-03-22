## Connection to ESP

To connect to the server to download the files on the SD card please :

1. Connect to the wifi "Kite" with the "12345678"
2. On your web browser, put the following url adress : "http://192.168.4.1/"

## Logging

To start logging:
1.  turn on the electronics by clipping the magnet in the ON-position (the clip closest to box outline, where there is a golden button beneath)
2. Put the round magnet into the circular spot on the lid, for 1 or 2 seconds, then remove the magnet.
3. The logging starts, the buzzer beeps short every 5 seconds
4. To stop, place the magnet again into the circular spot.

If the buzzer is not loud enough, one can connect to the ESP and check that a new file is created when starting to log.

## Calibration

Calibration happens when turning on the system.
Place the board flat so that the IMU has a correct reference plane. Nothing except straps should be on the "platines"
If a new calibration is needed, turn off the system and then turn it on again,

## Buzzer

The buzzer indicates wether the GPS has a fix and if the system is logging.

If there is no fix, it buzzes every 2 seconds with a long beep.
When logging, it buzzes short every 5 seconds.

If the system is logging while the GPS has no fix, both pattern are played at the same time: long beep every 2 seconds and a longer beep following by a short one every 5 seconds.

## Charging

To charge, place the induction charger on the electronic box where it is delimited with the 3D print, cable towards the inside of the board.

As there is no feedback on the voltage level of the battery, recharge until full after each use.

### LED meaning
- Orange: charging
- Green: battery full
- Red: low voltage, need to recharge
- Blue: The system is on