# gearVRC 

**gearVRC.py** Samsung Gear VR Controller client implementation in python .

This client reads
- buttons
- touchpad
- accelerometer
- gyroscope
- magnetometer

and uses **AHRS** to fuse the data into a pose. The convention I use is x points forward, y points to the east and z down.

The software provides a virtual wheel implementation: when sliding a finger along the rim of the touchpad the rotation direction and position is calculated.

# Table of Contents

- [gearVRC](#gearvrc)
- [Table of Contents](#table-of-contents)
  * [Usage](#usage)
  * [Installation](#installation)
  * [Pre Requisites](#pre-requisites)
  * [bleak Issues](#bleak-issues)
  * [Behaviors of the gearVR Controller](#behaviors-of-the-gearvr-controller)
  * [Serial](#serial)
  * [ZMQ](#zmq)
    + [Windows](#windows)
    + [Unix](#unix)
  * [Calibration](#calibration)
  * [pyIMU](#pyimu)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

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
  -m, --motion			attempts calculating velocity and position
  -s <serial>, --serial <serial>
                        serial port for reporting, e.g '/tmp/ttyV0' when you
                        are using virtual ports 'socat -d -d
                        pty,rawer,echo=0,link=/tmp/ttyV0
                        pty,rawer,echo=0,link=/tmp/ttyV1&'
  -b <baud>, --baud <baud>
                        serial baud rate, e.g. 115200
  -z <zmqport>, --zmq <zmqport>
                        port used by ZMQ, e.g. 5556
  -e <escreps>, --escreps <escreps>
						number of consecutive home button presses to terminate program
						e.g. 3
  -t <esctimeoput>, --esctimeout <esctimeout>
						timeout for counting the home button presses in seconds, e.g. 2.0
```
- ```-n``` by default is ```Gear VR Controller(17DB)```
- ```-a``` vy default is ```2C:BA:BA:2E:17:DB```
- ```-vr``` is not working, supposed to start sensor in VR Mode
- ```-d``` switches loggin output to DEBUG
- ```-r``` set debuggin to rate display (1), or full display (2), default is no reporting
- ```-f``` enables AHRS sensor fusion
- ```-m``` enables motion calculation
- ```-v``` enables wheel simulation by sliding finger along the rim of touchpad, also enables scrolling by adding movements to previous position on touch pad
- ```-s``` provide serial port so that freeIMUCal can obtain data from the sensors
- ```-b``` us baudrate with default of 115200
- ```-z``` for ZMQ data announcement

## Installation

Download the archive and `'pip3 install .'`  . You can also add the `-e` switch to create a link to the local folder.

## Pre Requisites
You will either need to install bleak: `pip install bleak`

This software uses
- asyncio
- pyserial_asyncio
- uvloop (on Windows uvloop is not loaded)
- zmq and msgpack
- pyIMU (repository from me)


## bleak Issues
This implementation uses **bleak** for BLE communication. 

**bleak** expects a confirmation reply after a device is connected and some devices do not conform with this BLE specs. For gearVRC to work well you need to edit the bleak client code in the bluezdbus backend and change ```assert reply is not None``` to ```if reply is not None:``` and indent the code until the line before ```assert_reply(reply).

## Behaviors of the gearVR Controller  

The gearVR Controller **disconnects** after **inactivity**. A short press on the Home key will reconnect it. Disconnection occurs regardless wether you send keep alive commands to the device or not. Disconnection also occurs when the device is just moved around. If buttons are pushed regularly the device does not disconnect.

Using gear VR Controller on **Windows** is **cumbersome** as one needs to remove the device from the system each time before using it. After inactivity disconnection, one needs to also remove the device. If this process can be automated, the controller might be usable on Windows also.

In this implementation, when you press the Home button three times within 2 seconds, the program exits.

**gearVRC Controller** can enter into a **pairing stage** that it will remain in and **refuse any pairing** attempts from Raspian or Windows. On Raspian, a system reboot helps. On Windows you need to remove the device so that the OS has no prior knowledge of the controller. On some Windows system bluetooth devices appear in the list and after removal the list is not cleared.

If you are stuck, attempt unpairing and removing device in bluetoothctl. Remove battery from gear VR Controller, reboot the system and try fresh again until the entry in bluetoothctl matches the one listed below.

The update rate from the controller is between **45 and 72 Hz**. If you restart the program it will switch in between those two numbers. It is not yet clear to me how to make sure we always get the same update rate. Since I use asyncio to signal to the ZMQ task that new data is available, the ZMQ update rate is slower than the fusion rate. This is due to the system timer resolution. If you need full data rate you need to move the ZMQ handling to the data notification routine.

The gearVRC sensors does not provide accurate data during the first couple readings after the system boots. You should not use that data for AHRS data fusion because both magnetometer and accelerometer are affected and AHRS makes an initial guess of its pose from the first reading.

## Serial
You can start virtual serial ports with virtual null modem connection between the two ports to transmit data between the client and a listener:

```
socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1
```
Then you can start gearVRC with
```
python gearVRC -s /tmp/ttyV0 -b 115200
```

You can test the serial output with
```
putty
```
and open serial port ```/tmp/ttyV1``` with baurate of ```115200```

In Putty you can type ```v``` followed by ```CTRL-J``` and response should be VRC version string.

You can type ```b10``` followed by ```CTRL-J``` and 10 lines of hex encoded acc,gyr,mag data should appear in terminal. Routines to decode and encode hex are in gearVRC.py.

## ZMQ
When starting ZMQ with ```-z 5556``` you will have ```tcp://localhost:5556``` available to listen to ZMQ messages.

There are 6 message types emitted as multi part message. You can subscribe to any of them:
- ```system```
- ```imu```
- ```button```
- ```touch```
- ```virtual```
- ```fusion```

That data within the messages is serialized to binary with messagepack.

Routines needed to decode that data and create python object out of them are provided.

system message contains:
```
	data.temperature
	data.battery_level
	data.data_rate 		Update rate of sensor readings
	data.virtual_rate 	
	data.fusion_rate 
	data.zmq_rate
	data.serial_rate 
	data.reporting_rate
```

imu message contains:
```
	data.time			Sensor time
	data.acc			Accelration
	data.gyr			Gyration
	data.mag			Magnetic field
```

button message contains:
```
	data.trigger		Trigger button
	data.touch			Touchpad pressed
	data.back
	data.home
	data.volume_up
	data.volume_down
	data.noButton
	data.touchX
	data.touchY
```

touch pad message contains
```
	data.time
	data.touchX			Location on Touchpad
	data.touchY
```

virtual Message contains:
```
	data.time
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
	data.time
	data.acc 		Vector3D of acceleration
	data.gyr		Vector3D of gyration
	data.mag		Vector3D of mangnetic field
	data.rpy 		Vector3d of roll, pitch and yaw
	data.heading	Heading
	data.q			Quanternion of Pose
```
### Windows
On Windows you will need to delete any records of a previous installation in Settings prior to reading from the device. 

```
Settings->Bluetooth & devices->Select the Gear VR Controller entry->right click and remove the device
```

Then turn on/off Bluetooth and if the device reappears remove it again. After a second removal it usually is successfully removed.

You can use BLE Console from https://github.com/sensboston/BLEConsole 
to test the sensor connection without using the provided programs.

### Unix
On Unix use the `bluetoothctl` command line tool. Executing the `info` command below will need to provide the following output:

```
[bluetooth]# info 2C:BA:BA:2E:17:DB
```
```
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

This state can be created with bluetoothctl. Try:
- power on
- pairable on
- scan on

After gearVRC appears on 2C:BA:BA:2E:17:DB
- info 2C:BA:BA:2E:17:DB

should create reponse:\
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

Since the device is now connected, you will need to `disconnect 2C:BA:BA:2E:17:DB` because: 

***You can not run this software if bluetoothctl or any other device is already connected to the gearVR Controller!***

A module was added that detects wether the gearVR Controller is already connected in the system. It runs bluetoothctl info command and executes disconnect command if necessary. Bluetoothctl is not available on Windows.

## Calibration
For AHRS, calibration data is expected that ensures proper offset and scaling of the sensor measurements. These calibration data are specific for each sensor but one can also set offset to 0 and scales to 1 in the provided json files.

For calibration please check freeIMUCal in my repositories.
The magnetometer is unusable without calibration. If you just need the touchpad and input keys, no calibration is needed. 
The gyroscope has drift and the code attempts detecting when the device does not move and calibrates for the drift while it runs.

## pyIMU
pyIMU is used to fuse IMU sensor data and calculates the device pose. The pose is best understood in Roll Pitch and Yaw terms but its internally computed using quaternions. 

Using pyIMU it is possible to calculate acceleration and velocity of the controller. The accuracy is insufficient to estimate the position.
