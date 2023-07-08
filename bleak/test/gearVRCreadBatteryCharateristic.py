import asyncio
import logging
from bleak import BleakClient, BleakScanner

# To remove a device from the cache:
# Removing does not work YET
#
# $device = Get-PnpDevice -FriendlyName "Gear VR Controller(17DB)"
# pnputil.exe /remove-device $device.InstanceId
# devcon remove $device.InstanceId
# Restart-Service -Name bthserv -Force
# Update-HostStorageCache

# logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.INFO)

DEVICE_NAME         = 'Gear VR Controller(17DB)'
DEVICE_MAC          = '2C:BA:BA:2E:17:DB'
SERVICE_NAME        = 'Battery Service'
CHARACTERISTIC_NAME = 'Battery Level'

async def main():

    myDevice = None
    myService = None
    myCharacteristic = None
    
    print("Scanning for Devices...")
    myDevice = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=10.0)
    # myDevice = await BleakScanner.find_device_by_address(DEVICE_MAC, timeout=10.0)
            
    if myDevice !=None:
        print('Found Device {}'.format(DEVICE_NAME))

        client = BleakClient(myDevice, winrt=dict(address_type='public', use_cached_services=False))
                
        if not client.is_connected:
            print("Connecting...") 
            await client.connect()
            print('{} is {}'.format(DEVICE_NAME, 'connected' if client.is_connected else 'not connected'))
            paired = await client.pair(protection_level=2)
            print("{} is {}".format(DEVICE_NAME, 'paired' if paired else 'not paired'))
        else:
            print("Already connected...") 
        
        # Scan for services
        print("Scanning for Services...")
        for service in client.services:
            if service.description == SERVICE_NAME:
                print('Found Service! {}'.format(service.description))
                myService = service
        
        if myService != None:
            print("Scanning for Characteristic...")
            for char in myService.characteristics:
                if char.description == CHARACTERISTIC_NAME:
                    myCharacteristic = char
                    print('Found Characteristic! {}: {}'.format(char.description, char.uuid))
        
        if myCharacteristic != None:

            print("Reading Value...")
            if not client.is_connected: await client.connect()                    
            print('{} is {}'.format(DEVICE_NAME, 'connected' if client.is_connected else 'not connected'))                
            try: 
                value = await client.read_gatt_char(myCharacteristic)
                print('{}: ({}), Value {}'.format(myCharacteristic,myCharacteristic.properties, int.from_bytes(value,byteorder='big')))                
            except Exception as e:
                print('{}: ({}), Error {}'.format(myCharacteristic,myCharacteristic.properties, e))

                
            print("Reading Descriptor...")
            if not client.is_connected: await client.connect()                    
            print('{} is {}'.format(DEVICE_NAME, 'connected' if client.is_connected else 'not connected'))
            for descriptor in myCharacteristic.descriptors:
                try:
                    value = await client.read_gatt_descriptor(descriptor.handle)
                    print("[Descriptor] {}, Value: {}".format(descriptor, value))
                except Exception as e:
                    print("[Descriptor] {}, Error: {}".format(descriptor, e))

        # print("Unpairing...")
        # paired = await client.unpair()
        # print("{} is {}".format(DEVICE_NAME, 'not paired' if paired else 'paired'))

        print("Disconnecting...")
        await client.disconnect()
        print('{} is {}'.format(DEVICE_NAME, 'connected' if client.is_connected else 'not connected'))

    else:
        print('Could not find Device {}'.format(DEVICE_NAME))
        
asyncio.run(main(), debug=False)
