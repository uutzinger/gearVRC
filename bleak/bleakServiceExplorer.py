"""
Service Explorer
----------------
An example showing how to access and print out the services, characteristics and
descriptors of a connected GATT server.
Created on 2019-03-25 by hbldh <henrik.blidh@nedomkull.com>
"""

import argparse
import asyncio
import logging

from bleak import BleakClient, BleakScanner

logger = logging.getLogger(__name__)


async def main(args: argparse.Namespace):
    logger.info("starting scan...")

    if args.address:
        device = await BleakScanner.find_device_by_address(
            args.address, cb=dict(use_bdaddr=args.macos_use_bdaddr)
        )
        if device is None:
            logger.error("could not find device with address '%s'", args.address)
            return
    else:
        device = await BleakScanner.find_device_by_name(
            args.name, cb=dict(use_bdaddr=args.macos_use_bdaddr)
        )
        if device is None:
            logger.error("could not find device with name '%s'", args.name)
            return

    logger.info("connecting to device...")

    async with BleakClient(
        device,
        services=args.services,
    ) as client:
        logger.info("connected")

        for service in client.services:
            logger.info("[Service] %s", service)

            for char in service.characteristics:
                if "read" in char.properties:
                    try:
                        value = await client.read_gatt_char(char.uuid)
                        logger.info(
                            "  [Characteristic] %s (%s), Value: %r",
                            char,
                            ",".join(char.properties),
                            value,
                        )
                    except Exception as e:
                        logger.error(
                            "  [Characteristic] %s (%s), Error: %s",
                            char,
                            ",".join(char.properties),
                            e,
                        )

                else:
                    logger.info(
                        "  [Characteristic] %s (%s)", char, ",".join(char.properties)
                    )

                for descriptor in char.descriptors:
                    try:
                        value = await client.read_gatt_descriptor(descriptor.handle)
                        logger.info("    [Descriptor] %s, Value: %r", descriptor, value)
                    except Exception as e:
                        logger.error("    [Descriptor] %s, Error: %s", descriptor, e)

        logger.info("disconnecting...")

    logger.info("disconnected")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    device_group = parser.add_mutually_exclusive_group(required=True)

    device_group.add_argument(
        "--name",
        metavar="<name>",
        help="the name of the bluetooth device to connect to",
    )
    device_group.add_argument(
        "--address",
        metavar="<address>",
        help="the address of the bluetooth device to connect to",
    )

    parser.add_argument(
        "--macos-use-bdaddr",
        action="store_true",
        help="when true use Bluetooth address instead of UUID on macOS",
    )

    parser.add_argument(
        "--services",
        nargs="+",
        metavar="<uuid>",
        help="if provided, only enumerate matching service(s)",
    )

    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="sets the log level to debug",
    )

    args = parser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(name)-8s %(levelname)s: %(message)s",
    )

    asyncio.run(main(args))


# $ sudo python3 bleakServiceExplore.py --address "2C:BA:BA:2E:17:DB"
# 2023-05-16 09:47:34,039 __main__ INFO: starting scan...
# 2023-05-16 09:47:34,436 __main__ INFO: connecting to device...
# 2023-05-16 09:47:39,196 __main__ INFO: connected
# 2023-05-16 09:47:39,196 __main__ INFO: [Service] 00001801-0000-1000-8000-00805f9b34fb (Handle: 8): Generic Attribute Profile
# 2023-05-16 09:47:39,197 __main__ INFO: [Service] 0000180a-0000-1000-8000-00805f9b34fb (Handle: 13): Device Information
# 2023-05-16 09:47:39,360 __main__ INFO:   [Characteristic] 00002a26-0000-1000-8000-00805f9b34fb (Handle: 22): Firmware Revision String (read), Value: bytearray(b'\xd4\xd4\xd4\xd4')
# 2023-05-16 09:47:39,539 __main__ INFO:   [Characteristic] 00002a24-0000-1000-8000-00805f9b34fb (Handle: 16): Model Number String (read), Value: bytearray(b'ET-YO324')
# 2023-05-16 09:47:39,719 __main__ INFO:   [Characteristic] 00002a27-0000-1000-8000-00805f9b34fb (Handle: 20): Hardware Revision String (read), Value: bytearray(b'Rev0.0')
# 2023-05-16 09:47:39,899 __main__ INFO:   [Characteristic] 00002a25-0000-1000-8000-00805f9b34fb (Handle: 18): Serial Number String (read), Value: bytearray(b'123456')
# 2023-05-16 09:47:40,079 __main__ INFO:   [Characteristic] 00002a28-0000-1000-8000-00805f9b34fb (Handle: 24): Software Revision String (read), Value: bytearray(b'YO324XXU0AQD4')
# 2023-05-16 09:47:40,259 __main__ INFO:   [Characteristic] 00002a29-0000-1000-8000-00805f9b34fb (Handle: 14): Manufacturer Name String (read), Value: bytearray(b'Samsung')
# 2023-05-16 09:47:40,439 __main__ INFO:   [Characteristic] 00002a50-0000-1000-8000-00805f9b34fb (Handle: 26): PnP ID (read), Value: bytearray(b'\x00\x00\x00\x00\x00\x01\x00')
# 2023-05-16 09:47:40,439 __main__ INFO: [Service] 00001879-0000-1000-8000-00805f9b34fb (Handle: 28): Vendor specific
# 2023-05-16 09:47:40,620 __main__ INFO:   [Characteristic] 00002a4d-0000-1000-8000-00805f9b34fb (Handle: 39): Report (read,write,notify), Value: bytearray(b'\x00\x00')
# 2023-05-16 09:47:40,799 __main__ INFO:     [Descriptor] 00002908-0000-1000-8000-00805f9b34fb (Handle: 42): Report Reference, Value: bytearray(b'\x03\x01')
# 2023-05-16 09:47:40,979 __main__ INFO:     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 41): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
# 2023-05-16 09:47:41,159 __main__ INFO:   [Characteristic] 00002a4e-0000-1000-8000-00805f9b34fb (Handle: 31): Protocol Mode (read,write-without-response), Value: bytearray(b'\x01')
# 2023-05-16 09:47:41,339 __main__ INFO:   [Characteristic] 00002a22-0000-1000-8000-00805f9b34fb (Handle: 43): Boot Keyboard Input Report (read,notify), Value: bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
# 2023-05-16 09:47:41,519 __main__ INFO:     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 45): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
# 2023-05-16 09:47:41,520 __main__ INFO:   [Characteristic] 00002a4c-0000-1000-8000-00805f9b34fb (Handle: 37): HID Control Point (write-without-response)
# 2023-05-16 09:47:41,701 __main__ INFO:   [Characteristic] 00002a4b-0000-1000-8000-00805f9b34fb (Handle: 33): Report Map (read), Value: bytearray(b'\x05\x01\t\x06\xa1\x01\x85\x01\x05\x07\x19\xe0)\xe7\x15\x00%\x01u\x01\x95\x08\x81\x02\x95\x01u\x08\x81\x03\x95\x05u\x01\x05\x08\x19\x01)\x05\x91\x02\x95\x01u\x03\x91\x03\x95\x06u\x08\x15\x00%e\x05\x07\x19\x00)e\x81\x00\xc0\x06\x00\xff\t\x02\xa1\x01\x85\x02u\x08\x95\x01\x15\x01%d\t \x81\x00\xc0\x05\x0c\t\x01\xa1\x01\x85\x03u\x10\x95\x01\x15\x01&\xff\x02\x19\x01*\xff\x02\x81`\xc0')
# 2023-05-16 09:47:41,879 __main__ INFO:   [Characteristic] 00002a32-0000-1000-8000-00805f9b34fb (Handle: 46): Boot Keyboard Output Report (read,write-without-response,write), Value: bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
# 2023-05-16 09:47:42,059 __main__ INFO:   [Characteristic] 00002a4a-0000-1000-8000-00805f9b34fb (Handle: 35): HID Information (read), Value: bytearray(b'\x00\x00\x00\x00')
# 2023-05-16 09:47:42,059 __main__ INFO: [Service] 4f63756c-7573-2054-6872-65656d6f7465 (Handle: 48): Unknown
# 2023-05-16 09:47:42,239 __main__ INFO:   [Characteristic] c8c51726-81bc-483b-a052-f7a14ea3d282 (Handle: 52): Unknown (read,write), Value: bytearray(b'')
# 2023-05-16 09:47:42,419 __main__ INFO:   [Characteristic] c8c51726-81bc-483b-a052-f7a14ea3d281 (Handle: 49): Unknown (read,notify), Value: bytearray(b'')
# 2023-05-16 09:47:42,599 __main__ INFO:     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 51): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
# 2023-05-16 09:47:42,599 __main__ INFO: [Service] 0000fef5-0000-1000-8000-00805f9b34fb (Handle: 54): Dialog Semiconductor GmbH
# 2023-05-16 09:47:42,779 __main__ INFO:   [Characteristic] 724249f0-5ec3-4b5f-8804-42345af08651 (Handle: 57): Unknown (read,write), Value: bytearray(b'')
# 2023-05-16 09:47:42,959 __main__ INFO:   [Characteristic] 6c53db25-47a1-45fe-a022-7c92fb334fd4 (Handle: 59): Unknown (read), Value: bytearray(b'')
# 2023-05-16 09:47:43,139 __main__ INFO:   [Characteristic] 5f78df94-798c-46f5-990a-b3eb6a065c88 (Handle: 65): Unknown (read,notify), Value: bytearray(b'\x00')
# 2023-05-16 09:47:43,319 __main__ INFO:     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 67): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
# 2023-05-16 09:47:43,499 __main__ INFO:   [Characteristic] 64b4e8b5-0de5-401b-a21d-acc8db3b913a (Handle: 70): Unknown (read), Value: bytearray(b'\r')
# 2023-05-16 09:47:43,679 __main__ INFO:   [Characteristic] 457871e8-d516-4ca1-9116-57d0b17b9cb2 (Handle: 63): Unknown (read,write-without-response,write), Value: bytearray(b'')
# 2023-05-16 09:47:43,859 __main__ INFO:   [Characteristic] 42c3dfdd-77be-4d9c-8454-8f875267fb3b (Handle: 72): Unknown (read), Value: bytearray(b'\xf4\x00')
# 2023-05-16 09:47:44,039 __main__ INFO:   [Characteristic] 9d84b9a3-000c-49d8-9183-855b673fda31 (Handle: 61): Unknown (read,write), Value: bytearray(b'')
# 2023-05-16 09:47:44,219 __main__ INFO:   [Characteristic] 8082caa8-41a6-4021-91c6-56f9b954cc34 (Handle: 55): Unknown (read,write), Value: bytearray(b'')
# 2023-05-16 09:47:44,400 __main__ INFO:   [Characteristic] b7de1eea-823d-43bb-a3af-c4903dfce23c (Handle: 74): Unknown (read), Value: bytearray(b'\xc8\x00')
# 2023-05-16 09:47:44,579 __main__ INFO:   [Characteristic] 61c8849c-f639-4765-946e-5c3419bebb2a (Handle: 68): Unknown (read), Value: bytearray(b'\x81\x00')
# 2023-05-16 09:47:44,579 __main__ INFO: [Service] 0000180f-0000-1000-8000-00805f9b34fb (Handle: 9): Battery Service
# 2023-05-16 09:47:44,849 __main__ INFO:   [Characteristic] 00002a19-0000-1000-8000-00805f9b34fb (Handle: 10): Battery Level (read,notify), Value: bytearray(b'd')
# 2023-05-16 09:47:45,029 __main__ INFO:     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 12): Client Characteristic Configuration, Value: bytearray(b'\x01\x00')
# 2023-05-16 09:47:45,029 __main__ INFO: disconnecting...
# 2023-05-16 09:47:47,550 __main__ INFO: disconnected