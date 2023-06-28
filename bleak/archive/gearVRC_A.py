#!/usr/bin/python3

# This work is based on:
# Gear VRC reverse engineering (Java): https://github.com/jsyang/gearvr-controller-webbluetooth
# Gear VRC to UDEV (Python): https://github.com/rdady/gear-vr-controller-linux
################################################################
# Prerequisite:
# A) python packages:
#   $ sudo pip3 install bleak more-itertools
# B Linux) Pair Controller:
#   $ bluetoothlctl
#     scan on
#     pair yourMAC
#     trust yourMAC
#     connect yourMAC
# B Windows) Pair Controller:
#   No command line tools for pairing of BLE devices available
#   You will need to manually remove the device from the system each time before using it
################################################################
            
# IMPORTS
################################################################
import asyncio
import logging
import struct
import argparse
import signal
import math
import time
import numpy as np
import more_itertools as mit
import os

from bleak      import BleakClient, BleakScanner
from bleak.exc  import BleakError
from bleak.backends.characteristic import BleakGATTCharacteristic

RAD2DEG = 180.0 / math.pi
# Device name:
################################################################
DEVICE_NAME = 'Gear VR Controller(17DB)'
DEVICE_MAC  = '2C:BA:BA:2E:17:DB'

################################################################
# Support Functions
################################################################

################################################################
# gearVRC 
################################################################

class gearVRC:
            
    def __init__(self, device_name=None, logger=None, VRMode=False) -> None:

        # super(gearVRC, self).__init__()

        # Bluetooth device description
        self.device_name          = device_name
  
        # Bluetooth device and client
        self._device              = None
        self._client              = None

        self.generic_access_profile_service     = None
        self.device_name_characteristic         = None
        self.device_information_service         = None
        self.manufacturer_name_characteristic   = None
        self.model_number_characteristic        = None
        self.serial_number_characteristic       = None
        self.hardware_revision_characteristic   = None
        self.firmware_version_characteristic    = None
        self.software_revision_characteristic   = None
        self.PnP_ID_characteristic              = None
        self.batter_service                     = None
        self.battery_level_characteristic       = None
        self.controller_data_service            = None
        self.controller_data_characteristic     = None
        self.controller_command_characteristics = None


        self.manufacturer_name    = ''
        self.model_number         = ''
        self.model_number         = ''
        self.hardware_revision    = ''
        self.firmware_revision    = ''
        self.software_revision    = ''
        self.pnp_ID               = -1

        # Signals
        self.disconnected        = asyncio.Event()
        self.connected           = asyncio.Event()
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger("gearVRC")

        
    def disconnected_callback(self,client):
        if self.connected.is_set():
            self.connected.clear()
            self.disconnected.set()

    async def connect(self):
        '''
        Connection Loop:
          Scan for Device
            Create Client
                Connect to Device
                Scan for Services & Characteristics

            Wait for Disconnection
            Attempt Reconnection
          Back to Scan for Device            
        '''
        while True: # Run for ever
            # Scan for Device
            #################
            # try to find device using its name
            if self._device is None and self.device_name is not None:
                self._device = await BleakScanner.find_device_by_name(self.device_name, timeout=5.0)

            # Connect to Device
            ###################
            if self._device is not None:
                # Create client
                self.logger.log(logging.INFO,"Found {}".format(self.device_name))            
                if self._client is None:
                    self._client = BleakClient(self._device, disconnected_callback=self.disconnected_callback)
                # Connect to device
                if not self._client.is_connected:
                    try:
                        await self._client.connect()
                        if self._client.is_connected:
                            self.connected.set() # signal we have connection
                            self.disconnected.clear()
                            self.logger.log(logging.INFO,"Connected to {}".format(self.device_name))
                            self.logger.log(logging.INFO,"Finding Characteristics")
                            self.find_characteristics() # scan an assign device characteristics
                            self.logger.log(logging.INFO,"Reading Device Information")
                            await self.read_deviceInformation() # populate device information
                    except BleakError:
                        self.disconnected.set()
                        self.logger.log(logging.ERROR,"Error connecting to {}".format(self.device_name))

                # Wait until disconnection occurs
                # Hopefully this will take a long time
                await self.disconnected.wait()

                # Attempt reconnection, device might still be available
                try:
                    await self._client.connect()
                    if self._client.is_connected:
                        self.connected.set()
                        self.disconnected.clear()
                        self.logger.log(logging.INFO,"Reconnected to {}".format(self.device_name))            
                        self.logger.log(logging.INFO,"Finding Characteristics")
                        self.find_characteristics()
                        self.logger.log(logging.INFO,"Reading Device Information")
                        await self.read_deviceInformation()
                    else: # could not connect
                        self._client = None
                        self._device = None
                        self.logger.log(logging.DEBUG,"Could not reconnect to {}".format(self.device_name))            
                        await asyncio.sleep(5)
                                            
                except BleakError:
                    self.connected.clear()
                    # next loop iteration we want to scan for device and reconnect
                    self._client = None   
                    self._device = None
                    self.logger.log(logging.DEBUG,"Could not reconnect to {}".format(self.device_name))            
                    await asyncio.sleep(5)

            else: # Device not found
                self.logger.log(logging.INFO,"Device not found. Retrying...")
                await asyncio.sleep(5)

    async def disconnect(self):
        '''
        Disconnect from sensor
        '''
        try: 
            await self._client.disconnect()
        except: 
            pass
        self.connected.clear()

 
    def find_characteristics(self):
        '''
        Scan device services and characteristics for the expected values
        '''


        # Generic Access Profile
        ########################
        for s in self._client.services:
            if s.uuid == '00001800-0000-1000-8000-00805f9b34fb':
                self.generic_access_profile_service = s        
        # read
        try:
            for c in self.generic_access_profile_service.characteristics:
                if c.uuid == '00002a00-0000-1000-8000-00805f9b34fb':
                    self.device_name_characteristic = c
        except:
            self.device_name_characteristic = None

        # Device Information
        ####################
        for s in self._client.services:
            if s.uuid == '0000180a-0000-1000-8000-00805f9b34fb':
                self.device_information_service = s
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a29-0000-1000-8000-00805f9b34fb':
                self.manufacturer_name_characteristic = c
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a24-0000-1000-8000-00805f9b34fb':
                self.model_number_characteristic = c
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a25-0000-1000-8000-00805f9b34fb':
                self.serial_number_characteristic = c
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a27-0000-1000-8000-00805f9b34fb':
                self.hardware_revision_characteristic = c
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a26-0000-1000-8000-00805f9b34fb':
                self.firmware_version_characteristic = c
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a28-0000-1000-8000-00805f9b34fb':
                self.software_revision_characteristic = c
        # read
        for c in self.device_information_service.characteristics:
            if c.uuid == '00002a50-0000-1000-8000-00805f9b34fb':
                self.PnP_ID_characteristic = c

        # Battery Service
        #################
        for s in self._client.services:
            if s.uuid == '0000180f-0000-1000-8000-00805f9b34fb':
                self.batter_service = s
        # read & notify
        for c in self.batter_service.characteristics:
            if c.uuid == '00002a19-0000-1000-8000-00805f9b34fb':
                self.battery_level_characteristic = c

        # Controller Sensor Data
        ########################
        for s in self._client.services:
            if s.uuid == '4f63756c-7573-2054-6872-65656d6f7465': # SERVICE
                self.controller_data_service = s
        # read & notify
        for c in self.controller_data_service.characteristics:
            if c.uuid == 'c8c51726-81bc-483b-a052-f7a14ea3d281': # NOTIFY
                self.controller_data_characteristic = c
        # read & write
        for c in self.controller_data_service.characteristics:
            if c.uuid == 'c8c51726-81bc-483b-a052-f7a14ea3d282': # WRITE
                self.controller_command_characteristics = c
        
    async def read_deviceInformation(self):
        '''
        Read Device Information
        Primarily used for characteristics that dont have notification
        '''
        
        # Battery Level
        try:
            if self.battery_level_characteristic is not None:
                value = await self._client.read_gatt_char(self.battery_level_characteristic)
                self.battery_level = int.from_bytes(value,byteorder='big')
            else:
                self.battery_level = -1
            self.logger.log(logging.DEBUG,'Battery Level: {}'.format(self.battery_level))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read Battery Level: {}'.format(e))
        # Device Name
        try:
            if self.device_name_characteristic is not None:
                value = await self._client.read_gatt_char(self.device_name_characteristic)
                self.device_name = value.decode("utf-8")
            else:
                self.device_name = DEVICE_NAME
            self.logger.log(logging.DEBUG,'Device Name: {}'.format(self.device_name))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read device name: {}'.format(e))
        # Manufacturer
        try: 
            if self.manufacturer_name_characteristic is not None:
                value = await self._client.read_gatt_char(self.manufacturer_name_characteristic)
                self.manufacturer_name = value.decode("utf-8")
            else:
                self.manufacturer_name = ''
            self.logger.log(logging.DEBUG,'Manufacturer Name: {}'.format(self.manufacturer_name))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read manufacturer name: {}'.format(e))
        # Model Number
        try: 
            if self.model_number_characteristic is not None:
                value = await self._client.read_gatt_char(self.model_number_characteristic)
                self.model_number = value.decode("utf-8")
            else:
                self.model_number = ''
            self.logger.log(logging.DEBUG,'Model Number: {}'.format(self.model_number))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read model number: {}'.format(e))
        # Serial Number
        try: 
            if self.serial_number_characteristic is not None:
                value = await self._client.read_gatt_char(self.serial_number_characteristic)
                self.serial_number = value.decode("utf-8")
            else:
                self.serial_number = ''
            self.logger.log(logging.DEBUG,'Model Number: {}'.format(self.model_number))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read serial number: {}'.format(e))
        # Hardware Revision
        try: 
            if self.hardware_revision_characteristic is not None:
                value = await self._client.read_gatt_char(self.hardware_revision_characteristic)
                self.hardware_revision = value.decode("utf-8")
            else:
                self.serial_number = ''
            self.logger.log(logging.DEBUG,'Hardware Revision: {}'.format(self.hardware_revision))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read hardware revision: {}'.format(e))
        # Firmware Revision
        try: 
            if self.firmware_version_characteristic is not None:
                value = await self._client.read_gatt_char(self.firmware_version_characteristic)
                self.firmware_revision = int.from_bytes(value,'big')
            else:
                self.firmware_revision = -1
            self.logger.log(logging.DEBUG,'Firmware Revision: {}'.format(self.firmware_revision))
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read firmware revision: {}'.format(e))
        # Software Revision
        try: 
            if self.software_revision_characteristic is not None:
                value = await self._client.read_gatt_char(self.software_revision_characteristic)
                self.software_revision = value.decode("utf-8")
            else:
                self.software_revision = ''
            self.logger.log(logging.DEBUG,'Software Revision: {}'.format(self.software_revision))
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read software revision: {}'.format(e))
        # PnP ID
        try: 
            if self.PnP_ID_characteristic is not None:
                value = await self._client.read_gatt_char(self.PnP_ID_characteristic)
                self.pnp_ID = int.from_bytes(value,'big')
            else:
                self.pnp_ID = -1
            self.logger.log(logging.DEBUG,'PnP ID: {}'.format(self.pnp_ID))                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not read pnp id: {}'.format(e))
              
async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)

    # gearVRC Controller
    controller = gearVRC(device_name=args.name, logger=logger)


    connection_task = asyncio.create_task(controller.connect())        # remain connected, will not terminate

    # These tasks will not terminate 
    await asyncio.gather(connection_task)

    logger.log(logging.INFO,"Exit")

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()

    device_group = parser.add_mutually_exclusive_group(required=True)

    device_group.add_argument(
        "-n",
        "--name",
        metavar="<name>",
        help="the name of the bluetooth device to connect to",
        default = 'Gear VR Controller(17DB)'
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
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
