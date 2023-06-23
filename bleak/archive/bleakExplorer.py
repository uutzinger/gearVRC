import asyncio
from bleak import BleakScanner, BleakClient

# https://github.com/hbldh/bleak

async def discover():
    myDevice = None

    devices = await BleakScanner.discover()
    for d in devices:
        if d.name == 'Gear VR Controller(17DB)':
            myDevice = d
            print('{}: {}, Found it!'.format(d.address, d.name))
        else:
            print(d)

    if myDevice != None:
        async with BleakClient(myDevice) as client:
            print("Services:")
            for service in client.services:
                print(service)
                    
        device_name = await client.read_gatt_char(DEVICE_NAME_UUID)
        print("Device Name: {0}".format("".join(map(chr, device_name))))

        manufacturer = await client.read_gatt_char(MANUFACTURER_UUID)
        print("Manufacturer: {0}".format("".join(map(chr, manufacturer))))

        model_number = await client.read_gatt_char(MODEL_NUMBER_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

        software_revision = await client.read_gatt_char(SOFTWARE_REVISION_UUID)
        print("Software Revision: {0}".format("".join(map(chr, software_revision))))

asyncio.run(discover())
