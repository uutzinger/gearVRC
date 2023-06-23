#!/usr/bin/env python
from __future__ import print_function

import binascii
import pygatt

import logging

logging.basicConfig()
logging.getLogger('pygatt').setLevel(logging.DEBUG)


MACADDRESS = '2C:BA:BA:2E:17:DB'
# Many devices, e.g. Fitbit, use random addressing - this is required to connect.
ADDRESS_TYPE = pygatt.BLEAddressType.random

adapter = pygatt.GATTToolBackend()
adapter.start()
device = adapter.connect(MACADDRESS)

for uuid in device.discover_characteristics().keys():
    try: 
        print("Read UUID %s: %s" % (uuid, binascii.hexlify(device.char_read(uuid))))
    except:
        print("Read UUID %s: can not read" % (uuid))


# Device Information
# [2C:BA:BA:2E:17:DB]     Service [0000180a-0000-1000-8000-00805f9b34fb]
# Read UUID 00002a24-0000-1000-8000-00805f9b34fb: b'45542d594f333234'                       # Model number
# Read UUID 00002a25-0000-1000-8000-00805f9b34fb: b'313233343536'                           # Serial number
# Read UUID 00002a26-0000-1000-8000-00805f9b34fb: b'd4d4d4d4'                               # Firmware version
# Read UUID 00002a27-0000-1000-8000-00805f9b34fb: b'526576302e30'                           # Hardware revision
# Read UUID 00002a28-0000-1000-8000-00805f9b34fb: b'594f3332345858553041514434'             # Software revision
# Read UUID 00002a29-0000-1000-8000-00805f9b34fb: b'53616d73756e67'                         # Manufacturer name
# Read UUID 00002a50-0000-1000-8000-00805f9b34fb: b'00000000000100'                         # PnP ID

# General
# [2C:BA:BA:2E:17:DB]     Service [00001801-0000-1000-8000-00805f9b34fb]                    # Generic Attribute
# Read UUID 00002a00-0000-1000-8000-00805f9b34fb: b'4765617220565220436f6e74726f6c6c657228' # Device Name
# Read UUID 00002a01-0000-1000-8000-00805f9b34fb: b'c003'                                   # Appearance
# Read UUID 00002a04-0000-1000-8000-00805f9b34fb: b'0b000b000000e803'                       # Peripheral Preferred Connection Parameters

# Battery
# [2C:BA:BA:2E:17:DB]     Service [0000180f-0000-1000-8000-00805f9b34fb]                    # Battery Servuce
# Read UUID 00002a19-0000-1000-8000-00805f9b34fb: b'64'                                     # Battery level

# Keyboard Information
# [2C:BA:BA:2E:17:DB]     Service [00001879-0000-1000-8000-00805f9b34fb]
# Read UUID 00002a22-0000-1000-8000-00805f9b34fb: b'0000000000000000'                       # "Boot Keyboard Input Report"
# Read UUID 00002a32-0000-1000-8000-00805f9b34fb: b'0000000000000000'                       # "Boot Keyboard Output Report"
# Read UUID 00002a4a-0000-1000-8000-00805f9b34fb: b'00000000'                               # "HID Information"
# Read UUID 00002a4b-0000-1000-8000-00805f9b34fb: b'05010906a1018501050719e029e71500250175' # "Report Map"
# Read UUID 00002a4c-0000-1000-8000-00805f9b34fb: can not read                              # "HID Control Point"
# Read UUID 00002a4d-0000-1000-8000-00805f9b34fb: b'0000'                                   # "Report"
# Read UUID 00002a4e-0000-1000-8000-00805f9b34fb: b'01'                                     # "Protocol Mode"

# Controller Service                                                                  
# [2C:BA:BA:2E:17:DB]     Service [4f63756c-7573-2054-6872-65656d6f7465]                    # Sensor Settings and Data
# Read UUID c8c51726-81bc-483b-a052-f7a14ea3d281: b'61a4cd04cefe8a01ea07f7fff7ffdfff3ab7cd' # Notify
# Read UUID c8c51726-81bc-483b-a052-f7a14ea3d282: b'0000'                                   # Write Settings

# Custom, perhaps firmware update
# [2C:BA:BA:2E:17:DB]     Service [0000fef5-0000-1000-8000-00805f9b34fb]
# Read UUID 8082caa8-41a6-4021-91c6-56f9b954cc34: can not read
# Read UUID 724249f0-5ec3-4b5f-8804-42345af08651: can not read
# Read UUID 6c53db25-47a1-45fe-a022-7c92fb334fd4: can not read
# Read UUID 9d84b9a3-000c-49d8-9183-855b673fda31: can not read
# Read UUID 457871e8-d516-4ca1-9116-57d0b17b9cb2: can not read
# Read UUID 5f78df94-798c-46f5-990a-b3eb6a065c88: b'00'
# Read UUID 61c8849c-f639-4765-946e-5c3419bebb2a: b'8100'
# Read UUID 64b4e8b5-0de5-401b-a21d-acc8db3b913a: b'0d'
# Read UUID 42c3dfdd-77be-4d9c-8454-8f875267fb3b: b'f400'
# Read UUID b7de1eea-823d-43bb-a3af-c4903dfce23c: b'1700'



