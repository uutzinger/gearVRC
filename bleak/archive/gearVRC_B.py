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

from pyIMU.quaternion import Vector3D

RAD2DEG = 180.0 / math.pi
# Device name:
################################################################
DEVICE_NAME = 'Gear VR Controller(17DB)'
DEVICE_MAC  = '2C:BA:BA:2E:17:DB'

# Device commands:
################################################################
CMD_OFF                          = bytearray(b'\x00\x00')   # Turn modes off and stop sending data
CMD_SENSOR                       = bytearray(b'\x01\x00')   # Touchpad and sensor buttons but low rate IMU data
CMD_UNKNOWN_FIRMWARE_UPDATE_FUNC = bytearray(b'\x02\x00')   # Initiate frimware update sequence
CMD_CALIBRATE                    = bytearray(b'\x03\x00')   # Initiate calibration: Not sure how to compensate for drift
CMD_KEEP_ALIVE                   = bytearray(b'\x04\x00')   # Keep alive: Not sure about time interval
CMD_UNKNOWN_SETTING              = bytearray(b'\x05\x00')   # Setting mode: ?
CMD_LPM_ENABLE                   = bytearray(b'\x06\x00')   # ?
CMD_LPM_DISABLE                  = bytearray(b'\x07\x00')   # ?
CMD_VR_MODE                      = bytearray(b'\x08\x00')   # Enable VR mode: high frequency event mode

# Device Supplements:
################################################################
# Virtual wheel (touching the touchpad along its rim)
WHEELDIAMETER                    = 315                      # Touchpad diameter
WHEELTICKNESS                    = 25                       # Virtual wheel rim thickness
WHEELRADIUS                      = WHEELDIAMETER / 2.       # Radius of touch pad
RTHRESH2                         = (WHEELRADIUS - WHEELTICKNESS)**2
NUMWHEELPOS                      = 64                       # simulate rotating wheel, number of positions
NUMWHEELPOS1_8                   = NUMWHEELPOS*1/8
NUMWHEELPOS3_8                   = NUMWHEELPOS*3/8
NUMWHEELPOS5_8                   = NUMWHEELPOS*5/8
NUMWHEELPOS7_8                   = NUMWHEELPOS*7/8
MAXWHEELPOS                      = NUMWHEELPOS-1
MAXWHEELPOS2                     = MAXWHEELPOS/2.0
# Scrolling on virtual touchpad
MINXTOUCH                        = 0
MINYTOUCH                        = 0
MAXXTOUCH                        = 1024
MAXYTOUCH                        = 1024

# Program Timing:
################################################################
MINIMUPDATETIME                  = 0.23                     # Not sure how fast this device might be
KEEPALIVEINTERVAL                = 60                       # Every minute

################################################################
# Support Functions
################################################################

def ror(l, n):
    '''
    split list at position -n and attach right section in front left section
    '''
    return l[-n:] + l[:-n]

def clamp(val, smallest, largest): 
    '''
    clip val to [smallest, largest]
    '''
    if val < smallest: return smallest
    if val > largest: return largest
    return val

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
        self.imuDataAvailable    = asyncio.Event()
        self.fusedDataAvailable  = asyncio.Event()
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger("gearVRC")

        self._VRMode             = VRMode

        # device sensors
        self.sensor_time         = 0     # There are 3 different times transmitted from the sensor
        self.aTime               = 0     # Assuming the earliest time corresponds to IMU
        self.bTime               = 0     #
        self.accX                = 0.    # IMU Accelerometer
        self.accY                = 0.    #
        self.accZ                = 0.    #
        self.gyrX                = 0.    # IMY Gyroscope
        self.gyrY                = 0.    #
        self.gyrZ                = 0.    #
        self.magX                = 0.    # IMU Magnetometer
        self.magY                = 0.    #
        self.magZ                = 0.    #
        self.touchX              = 0.    # Touchpad Location (up/down)
        self.touchY              = 0.    # (left/right)
        self.temperature         = 0.    # Device Temperature
        self.battery_level       = 0.0   # Device Battery level
        # device buttons
        self.touch               = False # Touchpad has been pressed
        self.trigger             = False # Trigger button pressed
        self.home                = False # Home button pressed
        self.back                = False # Back button pressed
        self.volume_up           = False # Volume up button pressed
        self.volume_down         = False # Volume down button pressed
        self.noButton            = True  # No button was pressed (mutually exclusive with the above)

        # virtual wheel
        self.wheel_touched       = False # Touchpad has been touched in wheel region
        self.wheelPos            = 0     # Wheel position 0..63
        self.top                 = False # Wheel touched in top quadrant
        self.bottom              = False # Wheel touched in buttom quadrant
        self.left                = False # Wheel touched in left quadrant
        self.right               = False # Wheel touched in right quadrant
        self.center              = False # Touchpad touched inside of wheel
        self.rotating            = False # Moving along the rim of the wheel
        self.clockwise           = False # Moving clockwise or counter clockwise
        
        # scrolling
        # Touch pad
        self.absX                = 0     # Position on virtual touchpad (scrolling)
        self.absY                = 0     #
        self.dirUp               = False # Touching pad and moving upwards
        self.dirDown             = False # Touching pad and moving downwards
        self.dirLeft             = False # Touching pad and moving to the left
        self.dirRight            = False # Touching pad and moving to the right
                        
        self._previous_touchX    = 0  # touch pad X
        self._previous_touchY    = 0  # touch pad Y
        self._previous_gyroX     = 0.  # gyroscope
        self._previous_gyroY     = 0.  #
        self._previous_gyroZ     = 0.  #
        self._previous_accelX    = 0.  # accelerometer
        self._previous_accelY    = 0.  #
        self._previous_accelZ    = 0.  #
        self._previous_magX      = 0.  # magnetometer
        self._previous_magY      = 0.  #
        self._previous_magZ      = 0.  #

        self._lastTime           = time.perf_counter()
        self._currentTime        = time.perf_counter()
        self._updatecounts       = 0
        self._previous_sensor_time = 0

        # NEED TO FIX THIS
        # add code if the sensor is not responding within timely fashion
        if self._VRMode: self._updateTime  = MINIMUPDATETIME
        else:            self._updateTime  = MINIMUPDATETIME

        # Virtual Wheel
        ###############
        self._previous_wheelPos  = -1
        self._2pi                = 2.0*math.pi
        _wheelPositions          = [i for i in range(0, NUMWHEELPOS)]
        # shift by 45 degrees (want to create region for left, right, top , bottom)
        _wheelPositions          = ror(_wheelPositions, NUMWHEELPOS // 8)
        # make 4 equal length sections
        _position_sections       = mit.divide(4,_wheelPositions)
        # assign 4 sections to individual ranges: top, right, bottom, left
        [self._l_right, self._l_top, self._l_left, self._l_bottom] = [list(x) for x in _position_sections]
        
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
                            self.logger.log(logging.INFO,"Subscribing to Notifications")
                            await self.subscribe_notifications() # subscribe to device characteristics that have notification function
                            self.logger.log(logging.INFO,"Starting Sensor")
                            await self.start_sensor() # start the sensors
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
                        self.logger.log(logging.INFO,"Subscribing to Notifications")
                        await self.subscribe_notifications() # subscribe to device characteristics that have notification function
                        self.logger.log(logging.INFO,"Starting Sensor")
                        await self.start_sensor() # start the sensors
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

    async def subscribe_notifications(self):
        ''' 
        Subscribe to data and battery notifications
        '''        
        
        # Sensor data
        await self._client.start_notify(self.controller_data_characteristic, self.handle_sensorData)
        # Battery data
        await self._client.start_notify(self.battery_level_characteristic,   self.handle_batteryData)
                
    async def unsubscribe_notifications(self):
        '''
        Unsubscribe
        '''
        # unsubscribe from sensor data
        await self._client.stop_notify(self.controller_data_characteristic)
        # unsubscribe from battery data
        await self._client.stop_notify(self.battery_level_characteristic)

    def handle_sensorData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        '''
        Decode the sensor data
        '''
        
        self._currentTime = time.perf_counter()
        
        # Should receive 60 values
        if (len(data) < 60):
            self.logger.log(logging.ERROR, "Not enough values: {}".format(len(data)))
            return
        
        # Update rate
        self._updatecounts += 1
        self._deltaTime = self._currentTime - self._lastTime
        self._lastTime = self._currentTime
        self.logger.log(logging.DEBUG, "Update rate: {}".format(self._deltaTime))
        if self._deltaTime > self._updateTime: 
            self.logger.log(logging.DEBUG, "Update rate slow")

        # Decode the values
        ###################
        
        # Time:
        #  There are three time stamps, one is general and one is for touch pad
        #   Not sure which belongs to which: 
        #   accelerometer is usually read faster than magnetometer, 
        #   gyroscope and accelerometer are usually on the same chip
        #   assuming earliest time stamp is the IMU time stamp
        self.sensorTime = (struct.unpack('<I', data[0:4])[0]   & 0xFFFFFFFF) /1000000.
        self.aTime      = (struct.unpack('<I', data[16:20])[0] & 0xFFFFFFFF) /1000000.
        self.bTime      = (struct.unpack('<I', data[32:36])[0] & 0xFFFFFFFF) /1000000.
        self.logger.log(logging.DEBUG, "Time: {:10.6f}, {:10.6f}, {:10.6f}".format(self.sensorTime, self.aTime, self.bTime))

        # Temperature:
        self.temperature = data[57]
        self.logger.log(logging.DEBUG, "Temperature: {} ".format(self.temperature))

        # Buttons
        self.trigger     = True if ((data[58] &  1) ==  1) else False
        self.home        = True if ((data[58] &  2) ==  2) else False
        self.back        = True if ((data[58] &  4) ==  4) else False
        self.touch       = True if ((data[58] &  8) ==  8) else False
        self.volume_up   = True if ((data[58] & 16) == 16) else False
        self.volume_down = True if ((data[58] & 32) == 32) else False
        self.noButton    = True if ((data[58] & 64) == 64) else False

        self.logger.log(logging.DEBUG, "Back button is {}".format("pushed" if self.back else "released"))
        self.logger.log(logging.DEBUG, "Home button is {}".format("pushed" if self.home else "released"))
        self.logger.log(logging.DEBUG, "Touchpad button is {}".format("pushed" if self.touch else "released"))
        self.logger.log(logging.DEBUG, "Trigger button is {}".format("pushed" if self.trigger else "released"))
        self.logger.log(logging.DEBUG, "Volume up button is {}".format("pushed" if self.volume_up else "released"))
        self.logger.log(logging.DEBUG, "Volume down button is {}".format("pushed" if self.volume_down else "released"))
        self.logger.log(logging.DEBUG, "{} button is pushed".format("No" if self.noButton else "A"))

        # Touch pad:
        #  Max observed value = 315
        #  Location 0/0 is not touched
        #  Bottom right is largest number
        #  Y axis is up-down
        #  X axis is left-right 
        self.touchX     = ((data[54] & 0xF) << 6) + ((data[55] & 0xFC) >> 2) & 0x3FF
        self.touchY     = ((data[55] & 0x3) << 8) + ((data[56] & 0xFF) >> 0) & 0x3FF
        self.logger.log(logging.DEBUG, "Touchpad Position: {}, {}".format(self.touchX, self.touchY))

        # Virtual Wheel
        #  Detects if rim of touchpad is touched and where 0..63
        #  Detects finger moves along wheel (rotation clockwise or counter clockwise)
        #  Detects if rim touched on top, left, right, bottom
        #  Detects scrolling of touchpad to create absolute "mouse" position, allowing to reach larger field than touchpad allone
        
        # Where are we touching the pad?
        x = self.touchX - WHEELRADIUS # horizontal distance from center of touchpad
        y = self.touchY - WHEELRADIUS # vertical distance from center
        l2 = x*x + y*y               # distance from center (squared)
        self.wheel_touched = True if l2 > RTHRESH2  else False
        if self.wheel_touched:
            self.center = False
            phi = (math.atan2(y,x) + self._2pi) % self._2pi # angle 0 .. 2*pi from pointing to the right counter clockwise
            self.wheelPos = int(math.floor(phi / self._2pi * NUMWHEELPOS))
            # Top, Bottom, Left, Right
            if self.wheelPos > NUMWHEELPOS1_8:
                if self.wheelPos < NUMWHEELPOS3_8:
                    self.top    = True
                    self.left   = False
                    self.bottom = False
                    self.right  = False
                elif self.wheelPos < NUMWHEELPOS5_8:
                    self.top    = False
                    self.left   = True
                    self.bottom = False
                    self.right  = False
                elif self.wheelPos < NUMWHEELPOS7_8:
                    self.top    = False
                    self.left   = False
                    self.bottom = True
                    self.right  = False
                else: 
                    self.top    = False
                    self.left   = False
                    self.bottom = False
                    self.right  = True
            self.logger.log(logging.DEBUG, "Wheel Position: {}".format(self.wheelPos))
        else: # wheel not touched
            self.top    = False
            self.left   = False
            self.bottom = False
            self.right  = False
            if self.touchX == 0 and self.touchY == 0: self.center = False # touch pad is not touched
            else:                                     self.center = True
            
        # Report
        if self.center:    _loc = 'Center'
        elif self.top:     _loc = 'Top'
        elif self.bottom:  _loc = 'Bottom'
        elif self.left:    _loc = 'Left'
        elif self.right:   _loc = 'Right'
        else:              _loc = 'None'
        self.logger.log(logging.DEBUG, "Touchpad: {}".format(_loc))

        # Are we turning the virtual wheel?
        if self.wheel_touched:
            self._deltaWheelPos = self._previous_wheelPos - self.wheelPos
            self._previous_wheelPos = self.wheelPos
            # deal with discontinuity 360/0:
            #   general formula with degrees and moving along a circle is
            #   a0 = a0 - (360. * np.floor((a0 + 180.)/360.))
            #   now a0 will be < 180 and >=-180
            self._deltaWheelPos = self._deltaWheelPos - (MAXWHEELPOS * np.floor((self._deltaWheelPos + MAXWHEELPOS2)/MAXWHEELPOS))
            if (self._deltaWheelPos > 0):
                # rotating clock wise
                self.rotating = True
                self.clockwise = True
                self.logger.log(logging.DEBUG, "Wheel rotating clockwise")
            elif (self._deltaWheelPos < 0):
                # rotating counter clock wise
                self.rotating = True
                self.clockwise = False
                self.logger.log(logging.DEBUG, "Wheel rotating counter clockwise")
        else:
            self.rotating = False
            self.logger.log(logging.DEBUG, "Wheel not rotating")
        
        # Movement direction on touchpad
        # This is to assess scrolling
        if not self.wheel_touched:
            self.deltaX = self.touchX - self._previous_touchX
            self.deltaY = self.touchY - self._previous_touchY
            self._previous_touchX = self.touchX
            self._previous_touchY = self.touchY
            self.logger.log(logging.DEBUG, "Delta X {}".format(self.deltaX))
            self.logger.log(logging.DEBUG, "Delta Y {}".format(self.deltaY))
            if (abs(self.deltaX) < 50) and (abs(self.deltaY) < 50): # disregard large jumps such as when lifting finger between scrolling
                self.absX += self.deltaX 
                self.absY += self.deltaY 
                self.absX  = clamp(self.absX, MINXTOUCH, MAXXTOUCH)
                self.absY  = clamp(self.absY, MINYTOUCH, MAXYTOUCH)
                self.logger.log(logging.DEBUG, "Absolute X {} Y {}".format(self.absX, self.absY))
                # Left or Right?
                if (self.deltaX > 0):
                    self.dirRight = True
                    self.dirLeft = False
                    self.logger.log(logging.DEBUG, "Touchpad Right")
                elif (self.deltaX < 0):
                    self.dirLeft = True
                    self.dirRight = False
                    self.logger.log(logging.DEBUG, "Touchpad Left")
                else:
                    self.dirLeft = False
                    self.dirRight = False
                # Up or Down?
                if (self.deltaY > 0):
                    self.dirDown = True
                    self.dirUp = False
                    self.logger.log(logging.DEBUG, "Touchpad Down")
                elif (self.deltaY < 0):
                    self.dirUp = True
                    self._dirDown = False
                    self.logger.log(logging.DEBUG, "Touchpad Up")
                else:
                    self.dirUp = False
                    self.dirDown = False

        # IMU:
        #  Accelerometer, Gyroscope, Magnetometer
        #  Magnetometer often has large hard iron offset
        self.accX = struct.unpack('<h', data[4:6])[0]   * 0.00478840332  # 10000.0 * 9.80665 / 2048 / 10000.0       # m**2/s
        self.accY = struct.unpack('<h', data[6:8])[0]   * 0.00478840332  #
        self.accZ = struct.unpack('<h', data[8:10])[0]  * 0.00478840332  #
        self.gyrX = struct.unpack('<h', data[10:12])[0] * 0.01221791529  # 10000.0 * 0.017453292 / 14.285 / 1000.0  # rad/s
        self.gyrY = struct.unpack('<h', data[12:14])[0] * 0.01221791529  #
        self.gyrZ = struct.unpack('<h', data[14:16])[0] * 0.01221791529  #
        self.magX = struct.unpack('<h', data[48:50])[0] * 0.06           # micro Tesla?, earth mag field 25..65 muTesla
        self.magY = struct.unpack('<h', data[50:52])[0] * 0.06           #
        self.magZ = struct.unpack('<h', data[52:54])[0] * 0.06           #
        
        # Apply calibration
        #  offset, scale, cross correlation between the axes
        self.acc = Vector3D(self.accX, self.accY, self.accZ)
        self.gyr = Vector3D(self.gyrX, self.gyrY, self.gyrZ)
        self.mag = Vector3D(self.magX, self.magY, self.magZ)

        self.logger.log(logging.DEBUG, "Accel {:5.2f} {:5.2f} {:5.2f} ".format(self.acc.x,self.acc.y,self.acc.z))
        self.logger.log(logging.DEBUG, "Mag   {:5.2f} {:5.2f} {:5.2f} ".format(self.mag.x,self.mag.y,self.mag.z))
        self.logger.log(logging.DEBUG, "Gyro  {:5.2f} {:5.2f} {:5.2f} ".format(self.gyr.x,self.gyr.y,self.gyr.z))
   
    def handle_batteryData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        self.battery_level = int.from_bytes(data,'big')
        self.logger.log(logging.DEBUG,"Battery level: {}".format(self.battery_level))

    async def start_sensor(self):
        # Initialize the device, not sure why command needs to be sent twice
        if self._VRMode:
            await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
            await self._client.write_gatt_char(self.controller_command_characteristics,CMD_VR_MODE)
        else:
            await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
            await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)

    async def stop_sensor(self):
        try:
            await self._client.write_gatt_char(self.controller_command_characteristics,CMD_OFF)
            await self._client.write_gatt_char(self.controller_command_characteristics,CMD_OFF)
            await self.unsubscribe_notifications()
            self.logger.log(logging.DEBUG,'Stopped')                
        except Exception as e:
            self.logger.log(logging.ERROR, 'Could not stop: {}'.format(e))            

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
