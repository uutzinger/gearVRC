import asyncio
import logging
from bleak import BleakClient, BleakScanner

# Services:
# 00001800-0000-1000-8000-00805f9b34fb (Handle: 1):  Generic Access Profile
# 00001801-0000-1000-8000-00805f9b34fb (Handle: 8):  Generic Attribute Profile
# 0000180f-0000-1000-8000-00805f9b34fb (Handle: 9):  Battery Service
# 0000180a-0000-1000-8000-00805f9b34fb (Handle: 13): Device Information
# 00001879-0000-1000-8000-00805f9b34fb (Handle: 28): Vendor specific
# 4f63756c-7573-2054-6872-65656d6f7465 (Handle: 48): Unknown
# 0000fef5-0000-1000-8000-00805f9b34fb (Handle: 54): Dialog Semiconductor GmbH

logging.basicConfig(level=logging.DEBUG)

DEVICE_NAME = 'Gear VR Controller(17DB)'

async def main():
    myDevice = None
    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == DEVICE_NAME:
            print('{}: {}, Found it!'.format(d.address, d.name))
            myDevice = d
        else:
            print('{}: {}'.format(d.address, d.name))
            
    if myDevice !=None:
        async with BleakClient(myDevice, winrt=dict(address_type='public', use_cached_services=False)) as client:        
            print("Services:")
            for service in client.services:
                print("Service: {}".format(service))

                for char in service.characteristics:
                    if "read" in char.properties:
                        try:
                            value = await client.read_gatt_char(char.uuid)
                            print("  Characteristic {} {}, Value: {}".format(char, char.properties, value))
                        except Exception as e:
                            print("  Characteristic {} {}, Error: {}".format(char, char.properties, e))
                    else:
                        print("  Characteristic {} {}".format(char, char.properties))

                    for descriptor in char.descriptors:
                        try:
                            value = await client.read_gatt_descriptor(descriptor.handle)
                            print("    Descriptor {}, Value: {}".format(descriptor, value))
                        except Exception as e:
                            print("    Descriptor {}, Value: {}".format(descriptor, e))

asyncio.run(main())

