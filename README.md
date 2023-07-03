# gearVRC

Samsung Gear VR Controller client implementation in python.
There are two implemenations:
- bleak/gearVRC.py
- gatt/gearVRC.py

The bleak implementation is the latest and has most features. It also "runs" on Windows.

The gearVR Controller disconnects after inactivity. Usually a short press on the Home key will reconnect it. Disconnection occurs regardless wether you send keep alive commands to the device. At this time I don't know if disconnection also occurs when the device is just moved around or if buttons were pushed regularly or if it detects being unused.

Using gear VR Controller on Windows is cumbersome as one needs to remove the device from the system each time before using it. After inactivity disconnection one needs to also remove the device. If this process can be automated, the controller might be usable on Windows also.

gearVR Controller runs on Raspian but might cause issues when it disconnects because of a signal loss. In this implementation, when yuo press the home button three times within 2 seocnds, the programs exits..

gearVRC Controller can enter into a pairing stage that it will remain in and refuse any pairing attempts from Raspian or Windows. On Raspian, a system reboot might help. On Windows you need to remove the device so that the OS has no prior knowledge of the controller.

If you are stuck, attempt unpairing and removing device in bluetoothctl. Remove battery from gear VR Controller, reboot the system and try fresh again until the entry in bluetoothctl matches the one listed below.

At this time I do not know how to automate the solution for a system that no longer pairs with the controller.

## Usage
gearVRC.py has the following options
```
  -h, --help            show this help message and exit
  -n <name>, --name <name>
                        the name of the bluetooth device to connect to
  -a <address>, --address <address>
                        the mac address of the bluetooth device to connect to
  -vr, --vrmode         sets the vrmode, sensor mode is used otherwise, [does
                        not work]
  -d, --debug           sets the log level from info to debug
  -r <report>, --report <report>
                        report level: 0(None), 1(Rate only), 2(Regular)
  -f, --fusion          turns on IMU data fusion
  -v, --virtual         turns on virtual wheel and touchpad
  -s <serial>, --serial <serial>
                        serial port for reporting, e.g '/tmp/ttyV0' when you
                        are using virtual ports 'socat -d -d
                        pty,rawer,echo=0,link=/tmp/ttyV0
                        pty,rawer,echo=0,link=/tmp/ttyV1&'
  -b <baud>, --baud <baud>
                        serial baud rate, e.g. 115200
  -z <zmqport>, --zmq <zmqport>
                        port used by ZMQ, e.g. 5556
```
- ```-n``` by default is ```Gear VR Controller(17DB)```
- ```-a``` vy default is ```2C:BA:BA:2E:17:DB```
- ```-vr``` is not working, supposed to start sensor in VR Mode
- ```-d``` switches loggin output to DEBUG
- ```-r``` set debuggin to rate display (1), or full display (2), default is no reporting
- ```-f``` enables AHRS sensor fusion
- ```-v``` enables wheel simulation by sliding finger along the rim of touchpad, also enables scrolling by adding movements to previous position on touch pad
- ```-s``` provide serial port so that freeIMUCal can obtain data from the sensors
- ```-b``` us baudrate with default of 115200
- ```-z``` for ZMQ data anouncement

### Serial
You can start virtual serial pors with null modem connection between the two ports:

```
socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1
```
The you can start gearVRC with
```
python gearVRC -s /tmp/ttyV0 -b 115200
```

You can test the serial output with
```
putty
```
and open serial port ```/tmp/ttyV1``` with baurate of ```115200```

In Putty you can type ```v``` followed by ```CTRL-J``` abd response should be VRC version string.

You can type ```b10``` followed by ```CTRL-J``` and 10 lines of hex encoded acc,gyr,mag data should appear in terminal

### ZMQ
When starting ZMQ with ```-z 5556``` you will have ```tcp://localhost:5556``` available to listen to ZMQ messages.

There are 4 message types emitted:
- system
- raw
- virtual
- fusion

That data within the messages is serialized with messagepack.

system message contains:
```
	data.data_rate 		Update rate of sensor readings
	data.virtual_rate 	
	data.fusion_rate 
	data.zmq_rate
	data.serial_rate 
	data.reporting_rate
```

raw message contains:
```
	data.accX			Accelration
	data.accY
	data.accZ
	data.gyrX			Gyration
	data.gyrY 
	data.gyrZ
	data.magX			Magnetic field
	data.magY
	data.magZ
	data.temp			Temperature
	data.battery		Battery Level
	data.trigger		Trigger button
	data.touch			Touchpad pressed
	data.back
	data.home
	data.volume_up
	data.volume_down
	data.noButton
	data.touchX			Location on Touchpad
	data.touchY
	data.sensorTime		Time when reading was taken on sensor in seconds
	data.aTime
	data.bTime
```
virtual Message contains:
```
	data.absX		Absolute Position on TouchPad
	data.absY
	data.dirUp		Finger is moving up
	data.dirDown
	data.dirLeft
	data.dirRigh
	data.wheelPos	Wheel is touched at postion
	data.top		Wheel is touched at top quadrant
	data.bottom		
	data.left		Wheel is touched at left quadrant
	data.right
	data.center		Touchpad is touched in the center
	data.isRotating	Finger is rotating along the rim
	data.clockwise  Finger is rotating clockwise if True
```

fusion Message contains
```
	data.acc 		Vector3D of acceleration
	data.gyr		Vector3D of gyration
	data.mag		Vector3D of mangnetic field
	data.rpy 		Vector3d of roll, pitch and yaw
	data.heading	Heading
	data.q			Quanternion of Pose
```
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

This state can be created with bluetoothctl but try:
- power on
- pairable on
- scan on

After gearVRC appears on 2C:BA:BA:2E:17:DB
- info 2C:BA:BA:2E:17:DB

This should create reponse:\
``` 
Device 2C:BA:BA:2E:17:DB (public)
	Name: Gear VR Controller(17DB)
	Alias: Gear VR Controller(17DB)
	Paired: no
	Bonded: no
	Trusted: no
	Blocked: no
	Connected: no
	LegacyPairing: no
	ManufacturerData Key: 0x0075
	ManufacturerData Value:
  01 00 02 00 fb 01 02 0e 03 76 72 73 65 74 75 70  .........vrsetup
  77 69 7a 61 72 64 10                             wizard.         
	RSSI: -55
	AdvertisingFlags:
  06 
```

Then execute
- trust 2C:BA:BA:2E:17:DB

This should results in:
```
[CHG] Device 2C:BA:BA:2E:17:DB Trusted: yes
Changing 2C:BA:BA:2E:17:DB trust succeeded
[CHG] Device 00:3C:7F:F0:F0:0A LegacyPairing: no
```

Then
- pair 2C:BA:BA:2E:17:DB

After pairing succeeded the device is connected. Accept the pop ups appearing on the desktop. The ones without user input buttons you need to close. 

Since the device is connected now you will need to `disconnect 2C:BA:BA:2E:17:DB` because: 

***You can not run this software if bluetoothctl or any other device is already connected to the gearVR Controller!***

A module was added that detects wether the gearVR Controller is already connected in the system. It runs bluetoothctl info command and executes disconnect command if necessary. Bluetoothctl is not available on Windows.

### Calibration
For calibration please check freeIMUCal in my repositories.
The magnetometer is unusable without calibration. If you just need the touchpad and input keys, no calibration is needed. 
The gyroscope has drift and future version will attempt detecting when the device does not move and calibrate the drift while it runs.

## pyIMU
pyIMU is used to fuse IMU sensor data and calculates the device pose. The pose is best understood qw Roll Pitch and Yaw terms but its internally computed using quaternions.

Using pyIMU it is possible (but not implemented) to calculate acceleration and velocity of the controller.
