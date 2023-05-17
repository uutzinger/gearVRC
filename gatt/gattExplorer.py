#!/usr/bin/python3
# bluetoothd --version
# sudo pip3 install gatt
# sudo apt-get install python3-dbus
#
# sudo gattctl --discover
# sudo gattctl --connect AA:BB:CC:DD:EE:FF # Replace the MAC address with your Bluetooth device's MAC address
# sudo gattctl --help # To list all available commands

# need to figur eout how to reseve more space for self.services

import gatt

MACADDRESS = '2C:BA:BA:2E:17:DB'

manager = gatt.DeviceManager(adapter_name='hci0')

class AnyDevice(gatt.Device):
    def device_discovered(self, device):
        print("Discovered [%s] %s" % (device.mac_address, device.alias()))

    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        print("[%s] Disconnected" % (self.mac_address))

    def services_resolved(self):
        super().services_resolved()

        print("[%s] Resolved services" % (self.mac_address))
        for service in self.services:
            print("[%s]\tService [%s]" % (self.mac_address, service.uuid))
            for characteristic in service.characteristics:
                print("[%s]\t\tCharacteristic [%s]" % (self.mac_address, characteristic.uuid))
                try: 
                    for descriptor in characteristic.descriptors:
                        print("[%s]\t\t\tDescriptor [%s] (%s)" % (self.mac_address, descriptor.uuid, descriptor.read_value()))
                except:
                    pass

    def descriptor_read_value_failed(self, descriptor, error):
        print('descriptor_value_failed')

device = AnyDevice(mac_address=MACADDRESS, manager=manager)
# manager.start_discovery()
device.connect()
manager.run()


# [2C:BA:BA:2E:17:DB] Connected
# [2C:BA:BA:2E:17:DB] Resolved services
# [2C:BA:BA:2E:17:DB]     Service [0000fef5-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [b7de1eea-823d-43bb-a3af-c4903dfce23c]
# [2C:BA:BA:2E:17:DB]             Characteristic [42c3dfdd-77be-4d9c-8454-8f875267fb3b]
# [2C:BA:BA:2E:17:DB]             Characteristic [64b4e8b5-0de5-401b-a21d-acc8db3b913a]
# [2C:BA:BA:2E:17:DB]             Characteristic [61c8849c-f639-4765-946e-5c3419bebb2a]
# [2C:BA:BA:2E:17:DB]             Characteristic [5f78df94-798c-46f5-990a-b3eb6a065c88]
# [2C:BA:BA:2E:17:DB]             Characteristic [457871e8-d516-4ca1-9116-57d0b17b9cb2]
# [2C:BA:BA:2E:17:DB]             Characteristic [9d84b9a3-000c-49d8-9183-855b673fda31]
# [2C:BA:BA:2E:17:DB]             Characteristic [6c53db25-47a1-45fe-a022-7c92fb334fd4]
# [2C:BA:BA:2E:17:DB]             Characteristic [724249f0-5ec3-4b5f-8804-42345af08651]
# [2C:BA:BA:2E:17:DB]             Characteristic [8082caa8-41a6-4021-91c6-56f9b954cc34]
# [2C:BA:BA:2E:17:DB]     Service [4f63756c-7573-2054-6872-65656d6f7465]
# [2C:BA:BA:2E:17:DB]             Characteristic [c8c51726-81bc-483b-a052-f7a14ea3d282]
# [2C:BA:BA:2E:17:DB]             Characteristic [c8c51726-81bc-483b-a052-f7a14ea3d281]
# [2C:BA:BA:2E:17:DB]     Service [00001879-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a32-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a22-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4d-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4c-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4a-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4b-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4e-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]     Service [0000180a-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a50-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a28-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a26-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a27-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a25-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a24-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a29-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]     Service [0000180f-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a19-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]     Service [00001801-0000-1000-8000-00805f9b34fb]
# is missing stuff here
# Read UUID 00002a00-0000-1000-8000-00805f9b34fb: b'4765617220565220436f6e74726f6c6c657228' # Device Name
# Read UUID 00002a01-0000-1000-8000-00805f9b34fb: b'c003'                                   # Appearance
# Read UUID 00002a04-0000-1000-8000-00805f9b34fb: b'0b000b000000e803'                       # Peripheral Preferred Connection Parameters
