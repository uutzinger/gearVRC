# gearVRC
Samsung Gear VR Controller client in python

## Summary
This is attempt to read Gear VR Controller using bluetooth LE.

Check gatt/gearVRC.py

Currently only runs on unix, until conversion to bleak is completed.

This work includes the following additions to other's implementations:

- Reading of device information including battery status and notification
- Reading of temperature and time stamps
- Correct reading of magnetometer
- Skeleton for sensor calibration
- Attitude estimation based on AHRS

This work needs additional software to calibrate the sensor:

- Reading of the accelerometer and magnetometer and storing of data
- Fitting of data onto ellipse
- Providing min/max of the axis, offset of the axis and cross correlation between the axis.
- runtime gyroscpe bias estimation

### Based On

- AHRS https://ahrs.readthedocs.io/en/latest/index.html
- Reverse Engineering https://jsyang.ca/hacks/gear-vr-rev-eng/
- GATT: https://github.com/rdady/gear-vr-controller-linux/

### Calibration
Check out https://github.com/makerportal/mpu92-calibration

### Windows
BLE Console from https://github.com/sensboston/BLEConsole 

#00: GenericAccess
#01: GenericAttribute
#02: Battery
#03: DeviceInformation
#04: 6265
#05: Custom Service: 4f63756c-7573-2054-6872-65656d6f7465
#06: 65269