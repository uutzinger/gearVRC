# gearVRC
Samsung Gear VR Controller client in python

## Summary
This is an attempt to read Gear VR Controller using bluetooth LE.

Check bleak/gearVRC.py or gatt/gearVRC.py

The bleak version runs also on Windows, however befure each use the gear device will need to be manually removed from the systyem. 

This work includes the following additions to other's implementations:

- Reading of device information including battery status and notification
- Reading of temperature and time stamps
- Correct reading of magnetometer
- Skeleton for sensor calibration
- Attitude estimation based on AHRS

This work needs additional software to calibrate the sensor:
- Reading of the accelerometer and magnetometer and storing of data
- Calibration, e.g. fitting of data onto ellipse

### Based On

- AHRS https://ahrs.readthedocs.io/en/latest/index.html
- Reverse Engineering https://jsyang.ca/hacks/gear-vr-rev-eng/
- GATT: https://github.com/rdady/gear-vr-controller-linux/

### Calibration
Check out https://github.com/makerportal/mpu92-calibration

### Windows
BLE Console from https://github.com/sensboston/BLEConsole 
Please make sure Windows does not have knowledge of the gear device, otherwise connection/pairing will fail.