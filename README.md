# gearVRC

Samsung Gear VR Controller client implementation in python.
There are two implemenations:
- bleak/gearVRC.py
- gatt/gearVRC.py

The bleak implementation is the latest and has most features. It also "runs" on Windows.

The gearVR Controller disconnects after inactivity. Usually a short press on the Home key will reconnect it. Disconnection occurs regardless wether you send keep alive commands to the device. At this time I don't know if disconnection also occurs when the device is just moved around or if buttons were pushed regularly or if it detects being unused.

Using gear VR Controller on Windows is cumbersome as one needs to remove the device from the system each time before using it. After inactivity disconnection one needs to also remove the device. If this process can be automated, the controller might be usable on Windows also.

gearVR Controller runs on Raspian but might cause issues when it disconnects because of a signal loss. You should attempt to implement a manual disconnection mechanism for example by responding to the user pushing the home button three times.

gearVRC Controller can enter into a pairing stage that it will remain in and refuse any pairing attempts from Raspian or Windows. On Raspian, a system reboot might help. On Windows you need to remove the device so that the OS has no prior knowledge of the controller.

If you are stuck, attempt unpairing and removing device in bluetoothctl. Remove battery from gear VR Controller, reboot the system and try fresh again until the entry in bluetoothctl matches the one listed below.

At this time I do not know how to automate the solution for a system that no longer pairs with the controller.

## Installation

Download the archive and `'pip3 install .'`  . You can also add the `-e` switch to create a link to a local folder.

## Prerequisites
You will either need to install bleak or gatt: e.g. `pip install bleak`

This software uses
- asyncio
- pyserial_asyncio
- uvloop (on Windows uvloop is not loaded)
- more_itertools
- re
- zmq and msgpack
- pyIMU

### Windows
On Windows you will need to delete any records of a previous installation in Settings prior to reading from the device. 

```
Settings->Bluetooth & devices->Select the Gear VR Controller entry->right click and remove the device
```

Then turn on/off Bluetooth and if the device reappears remove it again. After a second removal it usually is successfully removed.

You can use BLE Console from https://github.com/sensboston/BLEConsole 
to test the sensor connection without using the provided programs.

### Unix
On Unix use the `bluetoothctl` command line tool. Executing the `info` command will need to provide the following output:

```
[bluetooth]# info 2C:BA:BA:2E:17:DB
Device 2C:BA:BA:2E:17:DB (public)
	Name: Gear VR Controller(17DB)
	Alias: Gear VR Controller(17DB)
	Appearance: 0x03c0
	Paired: yes
	Bonded: yes
	Trusted: yes
	Blocked: no
	Connected: no
	LegacyPairing: no
	UUID: Generic Access Profile    (00001800-0000-1000-8000-00805f9b34fb)
	UUID: Generic Attribute Profile (00001801-0000-1000-8000-00805f9b34fb)
	UUID: Device Information        (0000180a-0000-1000-8000-00805f9b34fb)
	UUID: Battery Service           (0000180f-0000-1000-8000-00805f9b34fb)
	UUID: Unknown                   (00001879-0000-1000-8000-00805f9b34fb)
	UUID: Dialog Semiconductor GmbH (0000fef5-0000-1000-8000-00805f9b34fb)
	UUID: Vendor specific           (4f63756c-7573-2054-6872-65656d6f7465)
	ManufacturerData Key: 0x0075
	ManufacturerData Value:
  01 00 02 00 fb 01 02 0e 03 76 72 73 65 74 75 70  .........vrsetup
  77 69 7a 61 72 64 10                             wizard.         
	AdvertisingFlags:
  00                                               . 
```

It is not entirely clear how this state can be created repeatedly with bluetoothctl but try:
- power on
- pairable on
- scan on
- info 2C:BA:BA:2E:17:DB
- pair 2C:BA:BA:2E:17:DB
- trust 2C:BA:BA:2E:17:DB

Usually after pairing succeeded the device is connected. Accept the pop ups appearing on the desktop. The ones without user input buttons you need to close. Usually after pairing completed successfully the device is connected, otherwise try `connect 2C:BA:BA:2E:17:DB` and within a minute it will tell you wether it was successful. 

You will need to `disconnect 2C:BA:BA:2E:17:DB` because: 

***You can not run this software if bluetoothctl or any other device is already connected to the gearVR Controller!***

### Calibration
For calibration please check freeIMUCal in my repositories.
The magnetometer is unusable without calibration. If you just need the touchpad and input keys, no calibration is needed. 
The gyroscope has drift and future version will attempt detecting when the device does not move and calibrate the drift while it runs.

## pyIMU
pyIMU is used to fuse IMU sensor data and calculates the device pose. The pose is best understood qw Roll Pitch and Yaw terms but its internally computed using quaternions.

Using pyIMU it is possible (but not implemented) to calculate acceleration and velocity of the controller.
