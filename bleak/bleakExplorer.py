import asyncio
from bleak import BleakScanner, BleakClient

# https://github.com/hbldh/bleak

MACADDRESS              = "2C:BA:BA:2E:17:DB"
DEVICE_NAME_UUID        = "00002a00-0000-1000-8000-00805f9b34fb"
PNP_ID_UUID             = "00002a50-0000-1000-8000-00805f9b34fb"
SOFTWARE_REVISION_UUID  = "00002a28-0000-1000-8000-00805f9b34fb"
FIRMWARE_REVISION_UUID  = "00002a26-0000-1000-8000-00805f9b34fb"
HARDWARE_REVISION_UUID  = "00002a27-0000-1000-8000-00805f9b34fb"
SERIAL_NUMBER_UUID      = "00002a25-0000-1000-8000-00805f9b34fb"
MODEL_NUMBER_UUID       = "00002a24-0000-1000-8000-00805f9b34fb"
MANUFACTURER_UUID       = "00002a29-0000-1000-8000-00805f9b34fb"


async def discover():
    devices = await BleakScanner.discover()
    for d in devices:
        print(d)

async def readinfo():
    async with BleakClient(MACADDRESS) as client:

        device_name = await client.read_gatt_char(DEVICE_NAME_UUID)
        print("Device Name: {0}".format("".join(map(chr, device_name))))

        manufacturer = await client.read_gatt_char(MANUFACTURER_UUID)
        print("Manufacturer: {0}".format("".join(map(chr, manufacturer))))

        model_number = await client.read_gatt_char(MODEL_NUMBER_UUID)
        print("Model Number: {0}".format("".join(map(chr, model_number))))

        software_revision = await client.read_gatt_char(SOFTWARE_REVISION_UUID)
        print("Software Revision: {0}".format("".join(map(chr, software_revision))))

asyncio.run(discover())
asyncio.run(readinfo())
