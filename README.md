# gearVRC
Samsung Gear VR Controller client implementation in python.
There are two implemetnations:
- bleak/gearVRC.py
- gatt/gearVRC.py

The bleak imlementation is the latest. I also runs on Windows.

## Installation

Download the archive and `pip3 install .` . You can also add the `-e` switch to create a link to local folder.

## Prerequisits
You will either need to install bleak or gatt: e.g. `pip install bleak`

The gearVR Controller has **issues with connecting** in Windows or Linux when attempting to add it through the operting systems `add device` functionality.

### Windows
On Windows you will need to delete any records of a pervious installation in Settings prior to reading from the device. 

```
Settings->Bluetooth & devices->Select the Gear VR Controller entry->right click and remove the device
```

Then turn on/off Bluetooth and if the device reapears remove it again.

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

### Calibration
Check out https://github.com/makerportal/mpu92-calibration

