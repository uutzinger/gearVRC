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

from pyIMU.madgwick   import Madgwick
from pyIMU.quaternion import Vector3D
from pyIMU.utilities  import q2rpy, heading

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
KEEPALIVEINTERVAL                = 10                       # Every minute

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

def calibrate(data, offset=Vector3D(0.,0.,0.), scale=Vector3D(1.,1.,1.), crosscorr=None):
    '''
    IMU calibration
    bias is offset so that 0 is in the middle of the range
    scale is gain so that 1 is the maximum value
    cross correlation is cross axis sensitivity
    
    Expects data, bias and scale to be a Vector3D
    Expects crosscorr to be 3x3 numpy array
    '''
    # Bias
    data = data-offset
    # Scale, assumed pos and neg sensitivity is the same
    data = data/scale
    # Cross Correlation
    if crosscorr is not None:
        data = data.rotate(crosscorr)

    return data

################################################################
# gearVRC 
################################################################

class gearVRC:
            
    def __init__(self, device_name=None, device_address=None, logger=None, VRMode=False) -> None:

        # super(gearVRC, self).__init__()

        # Bluetooth device description
        self.device_name          = device_name
        self.device_address       = device_address

        # Bluetooth device and client
        self._device              = None
        self._client              = None

        self.generic_access_profile_service = None
        self.device_name_characteristic =None
        self.device_information_service = None
        self.device_information_service = None
        self.manufacturer_name_characteristic = None
        self.model_number_characteristic = None
        self.serial_number_characteristic = None
        self.hardware_revision_characteristic = None
        self.firmware_version_characteristic = None
        self.software_revision_characteristic = None
        self.PnP_ID_characteristic = None
        self.batter_service = None
        self.battery_level_characteristic = None
        self.controller_data_service = None
        self.controller_data_characteristic = None
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

        # Init vars
        logger.info("Initializing variables...")

        self._VRMode             = VRMode

        # device sensors
        self.sensor_time         = 0     # There are 3 different times transmitted from the sensor
        self.aTime               = 0     # Assuming the earliest time corresponds to IMU
        self.bTime               = 0     #
        self.accelX              = 0.    # IMU Accelerometer
        self.accelY              = 0.    #
        self.accelZ              = 0.    #
        self.gyroX               = 0.    # IMY Gyroscope
        self.gyroY               = 0.    #
        self.gyroZ               = 0.    #
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

        # IMU calibration
        # In future I will read calibration from file
        #  bias is offset so that 0 is in the middle of the range
        #  scale is gain so that 1 is the maximum value
        #  cross correlation is cross axis sensitivity
        self.accel_offset        = Vector3D(0,0,0)
        self.accel_scale         = Vector3D(1,1,1)
        self.accel_crosscorr     = np.array(([1,0,0], [0,1,0], [0,0,1]))
        self.gyro_offset         = Vector3D(0,0,0)
        self.gyro_scale          = Vector3D(1,1,1)
        # Not sure how to measure cross correlation for gyroscope
        self.mag_offset          = Vector3D(0,0,0)
        self.mag_scale           = Vector3D(1,1,1)
        self.mag_crosscorr       = np.array(([1,0,0], [0,1,0], [0,0,1]))

        # Attitude fusion
        self.AHRS                = Madgwick()
        
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
                Subscribe to Notifications
                Start Sensors
            Wait for Disconnection
            Attempt Reconnection
          Back to Scan for Device            
        '''
        while True: # Run for ever
            # Scan for Device
            #################
            # try to find device using its name or address
            if self._device is None and self.device_name is not None:
                self._device = await BleakScanner.find_device_by_name(self.device_name, timeout=5.0)
            elif self._device is None and self.device_address is not None:                  
                self._device = await BleakScanner.find_device_by_address(self.device_address, timeout=5.0)
            # try default device address
            if self._device is None:
                self._device = await BleakScanner.find_device_by_address(DEVICE_MAC, timeout=5.0)

            # Connect to Device
            ###################
            if self._device is not None:
                # Create client
                if self._client is None:
                    self._client = BleakClient(self._device, disconnected_callback=self.disconnected_callback, winrt=dict(address_type='public', use_cached_services=False))
                # Connect to device
                if not self._client.is_connected:
                    try:
                        await self._client.connect()
                        if self._client.is_connected:
                            self.connected.set() # signal we have connection
                            self.disconnected.clear()
                            self.logger.log(logging.DEBUG,"Connected to {}".format(self.device_name))
                            await self.find_characteristics() # scan an assign device characteristics
                            await self.read_deviceInformation() # populate device information
                            await self.subscribe_notifications() # subscribe to device characteristics that have notification function
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
                        self.logger.log(logging.DEBUG,"Reconnected to {}".format(self.device_name))            
                        # subscribe to notifications
                        await self.find_characteristics()
                        await self.read_deviceInformation()
                        await self.subscribe_notifications()
                        await self.start_sensor()
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
                self.logger.log(logging.ERROR,"Device not found. Retrying...")
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

    async def find_characteristics(self):
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
            self.logger.log(logging.DEBUG, "Wheel Position: {}".format(self._wheelPos))
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
            self._delta_wheelPos = self._previous_wheelPos - self.wheelPos
            self._previous_wheelPos = self.wheelPos
            # deal with discontinuity 360/0:
            #   general formula with degrees and moving along a circle is
            #   a0 = a0 - (360. * np.floor((a0 + 180.)/360.))
            #   now a0 will be < 180 and >=-180
            self._delta_wheelPos = self._delta_wheelPos - (MAXWHEELPOS * np.floor((self._delta_wheelPos + MAXWHEELPOS2)/MAXWHEELPOS))
            if (self._delta_wheelPos > 0):
                # rotating clock wise
                self.rotating = True
                self.clockwise = True
                self.logger.log(logging.DEBUG, "Wheel rotating clockwise")
            elif (self._delta_wheelPos < 0):
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
            self.delta_X = self.touchX - self._previous_touchX
            self.delta_Y = self.touchY - self._previous_touchY
            self._previous_touchX = self.touchX
            self._previous_touchY = self.touchY
            self.logger.log(logging.DEBUG, "Delta X {}".format(self._delta_X))
            self.logger.log(logging.DEBUG, "Delta Y {}".format(self._delta_Y))
            if (abs(self.delta_X) < 50) and (abs(self._delta_Y) < 50): # disregard large jumps such as when lifting finger between scrolling
                self.absX += self.delta_X 
                self.absY += self.delta_Y 
                self.absX  = clamp(self.absX, MINXTOUCH, MAXXTOUCH)
                self.absY  = clamp(self.absY, MINYTOUCH, MAXYTOUCH)
                self.logger.log(logging.DEBUG, "Absolute X {} Y {}".format(self.absX, self.absY))
                # Left or Right?
                if (self.delta_X > 0):
                    self.dirRight = True
                    self.dirLeft = False
                    self.logger.log(logging.DEBUG, "Touchpad Right")
                elif (self.delta_X < 0):
                    self.dirLeft = True
                    self.dirRight = False
                    self.logger.log(logging.DEBUG, "Touchpad Left")
                else:
                    self.dirLeft = False
                    self.dirRight = False
                # Up or Down?
                if (self.delta_Y > 0):
                    self.dirDown = True
                    self.dirUp = False
                    self.logger.log(logging.DEBUG, "Touchpad Down")
                elif (self.delta_Y < 0):
                    self.dirUp = True
                    self._dirDown = False
                    self.logger.log(logging.DEBUG, "Touchpad Up")
                else:
                    self.dirUp = False
                    self.dirDown = False

        # IMU:
        #  Accelerometer, Gyroscope, Magnetometer
        #  Magnetometer often has large hard iron offset
        self.accelX = struct.unpack('<h', data[4:6])[0]   * 0.00478840332  # 10000.0 * 9.80665 / 2048 / 10000.0       # m**2/s
        self.accelY = struct.unpack('<h', data[6:8])[0]   * 0.00478840332  #
        self.accelZ = struct.unpack('<h', data[8:10])[0]  * 0.00478840332  #
        self.gyroX  = struct.unpack('<h', data[10:12])[0] * 0.01221791529  # 10000.0 * 0.017453292 / 14.285 / 1000.0  # rad/s
        self.gyroY  = struct.unpack('<h', data[12:14])[0] * 0.01221791529  #
        self.gyroZ  = struct.unpack('<h', data[14:16])[0] * 0.01221791529  #
        self.magX   = struct.unpack('<h', data[48:50])[0] * 0.06           # micro Tesla?, earth mag field 25..65 muTesla
        self.magY   = struct.unpack('<h', data[50:52])[0] * 0.06           #
        self.magZ   = struct.unpack('<h', data[52:54])[0] * 0.06           #
        
        # Apply calibration
        #  offset, scale, cross correlation between the axes
        self.acc = Vector3D(self.accelX,self.accelY,self.accelZ)
        self.gyr = Vector3D(self.gyrX,  self.gyrY,  self.gyrZ)
        self.mag = Vector3D(self.magX,  self.magY,  self.magZ)
        self.acc = self.calibrate(self.acc, self.acc_offset, self.acc_scale. self.acc_crosscorr)
        self.mag = self.calibrate(self.mag, self.mag_offset, self.mag_scale, self.mag_crosscorr)
        self.gyr = self.calibrate(self.gyr, self.gyr_offset, self.gyr_scale, None)

        self.imuDataAvailable.set()

        # Compute Azimuth (compass)
        self.azimuth = math.atan2(self.mag.y,self.mag.x)*180./math.pi
        if self.azimuth < 0: self.azimuth +=180.

        self.logger.log(logging.DEBUG, "Accel {:5.2f} {:5.2f} {:5.2f} ".format(self.acc.x,self.acc.y,self.acc.z))
        self.logger.log(logging.DEBUG, "Mag   {:5.2f} {:5.2f} {:5.2f} ".format(self.mag.x,self.mag.y,self.mag.z))
        self.logger.log(logging.DEBUG, "Gyro  {:5.2f} {:5.2f} {:5.2f} ".format(self.gyr.x,self.gyr.y,self.gyr.z))
        self.logger.log(logging.DEBUG, "Azimuth  {:6.2f}  ".format(self.azimuth))
   
    def handle_batteryData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        self.battery_level = int.from_bytes(data,'big')
        self.logger.lot(logging.DEBUG,"Battery level: {}".format(self.battery_level))

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

    async def keep_alive(self):   
        while True:
            if self.connected.is_set():
                try:
                    # Not sure about the numbers of resending needed
                    await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                    await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                    await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                    await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                    self.logger.log(logging.DEBUG,'Keep alive sent')                
                except Exception as e:
                    self.logger.log(logging.ERROR, 'Could not send Keep alive: {}'.format(e))            
                    
            await asyncio.sleep(KEEPALIVEINTERVAL)

    async def fuse(self):
        ''' 
        Fuse data to pose
        '''
        while True:
            await self.imuDataAvailable.wait()
            self.imuDataAvailable.clear()
            dt = self.self.sensor_time - self._previous_sensor_time # time interval between sensor data
            self._previous_sensor_time = self.self.sensor_time # keep track of last sensor time
            
            self.q = self.AHRS.update(acc=self.acc,gyr=self.gyr,mag=self.mag,dt=dt)
            
            self.fusedDataAvailable.set()

    async def report(self):
        '''
        Report latest fused data
        '''
        while True:
            self.logger.log(logging.DEBUG, "Waiting for fused data")
            await self.fusedDataAvailable.wait()
            self.fusedDataAvailable.clear()
            self.logger.log(logging.DEBUG, "Fused data available")

            # Display the Data
            self.logger.log(logging.INFO, "Temp {:>4.1f}, Bat {:>3d}".format(self.temperature, self.battery_level))
            self.logger.log(logging.INFO, "Time {:>10.6f}, {:>10.6f}, {:>10.6f}".format(self.sensorTime, self.aTime, self.bTime))
            self.logger.log(logging.INFO, "Accel {:>5.2f} {:>5.2f} {:>5.2f} ".format(self.acc.x,self.acc.y,self.acc.z))
            self.logger.log(logging.INFO, "Mag   {:>5.2f} {:>5.2f} {:>5.2f} ".format(self.mag.x,self.mag.y,self.mag.z))
            self.logger.log(logging.INFO, "Gyro  {:>5.2f} {:>5.2f} {:>5.2f} ".format(self.gyr.x,self.gyr.y,self.gyr.z))
            h=heading(self.q, self.mag, declination=0.0)
            rpy=q2rpy(self.q)
            self.logger.log(logging.INFO, "Roll {:>4.0f} Pitch {:>4.0f} Yaw {:>4.0f} Heading {:>4.0f}".format(
                                            rpy.x*RAD2DEG, rpy.y*RAD2DEG, rpy.z*RAD2DEG, h*RAD2DEG))
            self.logger.log(logging.INFO, "Touchpad: {:>3d},{:>3d} {:>3d},{:>3d} U{} D{} L{} R{} C{}".format(
                                           self.touchX, self.touchY, self.absX, self.absY,
                                           'Y' if self.dirUp    else 'N',
                                           'Y' if self.dirDown  else 'N',
                                           'Y' if self.dirLeft  else 'N',
                                           'Y' if self.dirRight else 'N',
                                           'Y' if self.center   else 'N'))
            if self.wheel_touched:
                self.logger.log(logging.INFO, "Wheel: {:>2d} T{} B{} L{} R{} C{} R:{}".format(
                                                self.wheelPos, 
                                                'Y' if self.top    else 'N',
                                                'Y' if self.bottom else 'N',
                                                'Y' if self.left   else 'N',
                                                'Y' if self.right  else 'N',
                                                'Y' if self.center else 'N', 
                                                (" C" if self.clockwise else "CC") if self.rotating else "--"))
            else:
                self.logger.log(logging.INFO, "Wheel: Not touched")

            self.logger.log(logging.INFO, "Trig {} Touch {} Home {} Back {}, Vol up {} Vol down {} None: {}".format(
                                                'Y' if self.trigger     else 'N', 
                                                'Y' if self.touch       else 'N', 
                                                'Y' if self.home        else 'N',
                                                'Y' if self.back        else 'N',
                                                'Y' if self.volume_up   else 'N',
                                                'Y' if self.volume_down else 'N',
                                                'Y' if self.noButton    else 'N'))

async def handle_termination(controller):
    print("Control-C or kill signal detected. Stopping...")
    await controller.stop_sensor()
    await controller.disconnect()
    asyncio.get_event_loop().stop()
                
async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)

    # gearVRC Controller
    controller = gearVRC(device_name=args.name, device_address=args.address, logger=logger, VRMode=args.vrmode)

    # Get the main event loop
    # loop = asyncio.get_running_loop()

    # Set up a Control-C handler to gracefully stop the program
    # This only exists in Unix
    # if os.name != 'nt':
    #     loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(handle_termination(controller)) ) # control-c
    #     loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.ensure_future(handle_termination(controller)) ) # kill

    connection_task = asyncio.create_task(controller.connect())        # remain connected, will not terminate
    # fusion_task     = asyncio.create_task(controller.fuse())           # attempt data fusion, will not terminate
    # reporting_task  = asyncio.create_task(controller.report())         # report new data, will not terminate
    # keepalive_task  = asyncio.create_task(controller.keep_alive())     # keep sensor alive, will not terminate

    # These tasks will not terminate 
    await asyncio.gather(connection_task) #, fusion_task, reporting_task, keepalive_task)

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

    device_group.add_argument(
        "--address",
        metavar="<address>",
        help="the address of the bluetooth device to connect to",
        default = None
    )
    
    parser.add_argument(
        "-d",
        "--debug",
        action="store_true",
        help="sets the log level to debug",
    )
    
    parser.add_argument(
        "-vr",
        "--vrmode",
        action="store_true",
        help="sets the vrmode, sensor mode is used otherwise",
    )

    args = parser.parse_args()
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format="%(asctime)-15s %(name)-8s %(levelname)s: %(message)s",
    )   

    use_VisualCode = True
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass


# Output of py -3 .\service_explorer.py --name 'Gear VR Controller(17DB)'
#
# INFO:bleak.backends.winrt.client:Paired to device with protection level 1.
# Paired: True
# [Service] 00001800-0000-1000-8000-00805f9b34fb (Handle: 1): Generic Access Profile
#   [Characteristic] 00002a00-0000-1000-8000-00805f9b34fb (Handle: 2): Device Name (read), Value: bytearray(b'Gear VR Controller(17DB)')
#   [Characteristic] 00002a01-0000-1000-8000-00805f9b34fb (Handle: 4): Appearance (read), Value: bytearray(b'\xc0\x03')
#   [Characteristic] 00002a04-0000-1000-8000-00805f9b34fb (Handle: 6): Peripheral Preferred Connection Parameters (read), Value: bytearray(b'\x0b\x00\x0b\x00\x00\x00\xe8\x03')
# [Service] 00001801-0000-1000-8000-00805f9b34fb (Handle: 8): Generic Attribute Profile
# [Service] 0000180f-0000-1000-8000-00805f9b34fb (Handle: 9): Battery Service
#   [Characteristic] 00002a19-0000-1000-8000-00805f9b34fb (Handle: 10): Battery Level (read,notify), Value: bytearray(b'd')
#     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 12): Client Characteristic Configuration, Value: bytearray(b'\x01\x00')
# [Service] 0000180a-0000-1000-8000-00805f9b34fb (Handle: 13): Device Information
#   [Characteristic] 00002a29-0000-1000-8000-00805f9b34fb (Handle: 14): Manufacturer Name String (read), Value: bytearray(b'Samsung')
#   [Characteristic] 00002a24-0000-1000-8000-00805f9b34fb (Handle: 16): Model Number String (read), Value: bytearray(b'ET-YO324')
#   [Characteristic] 00002a25-0000-1000-8000-00805f9b34fb (Handle: 18): Serial Number String (read), Value: bytearray(b'123456')
#   [Characteristic] 00002a27-0000-1000-8000-00805f9b34fb (Handle: 20): Hardware Revision String (read), Value: bytearray(b'Rev0.0')
#   [Characteristic] 00002a26-0000-1000-8000-00805f9b34fb (Handle: 22): Firmware Revision String (read), Value: bytearray(b'\xd4\xd4\xd4\xd4')
#   [Characteristic] 00002a28-0000-1000-8000-00805f9b34fb (Handle: 24): Software Revision String (read), Value: bytearray(b'YO324XXU0AQD4')
#   [Characteristic] 00002a50-0000-1000-8000-00805f9b34fb (Handle: 26): PnP ID (read), Value: bytearray(b'\x00\x00\x00\x00\x00\x01\x00')
# [Service] 00001879-0000-1000-8000-00805f9b34fb (Handle: 28): Vendor specific
#   [Characteristic] 00002a4e-0000-1000-8000-00805f9b34fb (Handle: 31): Protocol Mode (read,write-without-response), Value: bytearray(b'\x01')
#   [Characteristic] 00002a4b-0000-1000-8000-00805f9b34fb (Handle: 33): Report Map (read), Value: bytearray(b'\x05\x01\t\x06\xa1\x01\x85\x01\x05\x07\x19\xe0)\xe7\x15\x00%\x01u\x01\x95\x08\x81\x02\x95\x01u\x08\x81\x03\x95\x05u\x01\x05\x08\x19\x01)\x05\x91\x02\x95\x01u\x03\x91\x03\x95\x06u\x08\x15\x00%e\x05\x07\x19\x00)e\x81\x00\xc0\x06\x00\xff\t\x02\xa1\x01\x85\x02u\x08\x95\x01\x15\x01%d\t \x81\x00\xc0\x05\x0c\t\x01\xa1\x01\x85\x03u\x10\x95\x01\x15\x01&\xff\x02\x19\x01*\xff\x02\x81`\xc0')
#   [Characteristic] 00002a4a-0000-1000-8000-00805f9b34fb (Handle: 35): HID Information (read), Value: bytearray(b'\x00\x00\x00\x00')
#   [Characteristic] 00002a4c-0000-1000-8000-00805f9b34fb (Handle: 37): HID Control Point (write-without-response)
#   [Characteristic] 00002a4d-0000-1000-8000-00805f9b34fb (Handle: 39): Report (read,write,notify), Value: bytearray(b'\x00\x00')
#     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 41): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
#     [Descriptor] 00002908-0000-1000-8000-00805f9b34fb (Handle: 42): Report Reference, Value: bytearray(b'\x03\x01')
#   [Characteristic] 00002a22-0000-1000-8000-00805f9b34fb (Handle: 43): Boot Keyboard Input Report (read,notify), Value: bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
#     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 45): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
#   [Characteristic] 00002a32-0000-1000-8000-00805f9b34fb (Handle: 46): Boot Keyboard Output Report (read,write-without-response,write), Value: bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
# [Service] 4f63756c-7573-2054-6872-65656d6f7465 (Handle: 48): Unknown
#   [Characteristic] c8c51726-81bc-483b-a052-f7a14ea3d281 (Handle: 49): Unknown (read,notify), Value: bytearray(b'\xe9\xd4\xef\x01Y\xfe\xca\x06\x01\x04\xf3\xff\xf8\xff\xe5\xff\xc2\xe7\xef\x01X\xfe\xcb\x06\x02\x04\xf2\xff\xf9\xff\xe8\xff\x9b\xfa\xef\x01Z\xfe\xc6\x06\x04\x04\xf3\xff\xf8\xff\xeb\xff\x1c\x03\x83\xf4\xee\xfd\x14\xc0\xc3\x19@d')
#     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 51): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
#   [Characteristic] c8c51726-81bc-483b-a052-f7a14ea3d282 (Handle: 52): Unknown (read,write), Value: bytearray(b'\x00\x00')
# [Service] 0000fef5-0000-1000-8000-00805f9b34fb (Handle: 54): Dialog Semiconductor GmbH
#   [Characteristic] 8082caa8-41a6-4021-91c6-56f9b954cc34 (Handle: 55): Unknown (read,write), Value: bytearray(b'')
#   [Characteristic] 724249f0-5ec3-4b5f-8804-42345af08651 (Handle: 57): Unknown (read,write), Value: bytearray(b'')
#   [Characteristic] 6c53db25-47a1-45fe-a022-7c92fb334fd4 (Handle: 59): Unknown (read), Value: bytearray(b'')
#   [Characteristic] 9d84b9a3-000c-49d8-9183-855b673fda31 (Handle: 61): Unknown (read,write), Value: bytearray(b'')
#   [Characteristic] 457871e8-d516-4ca1-9116-57d0b17b9cb2 (Handle: 63): Unknown (read,write-without-response,write), Value: bytearray(b'')
#   [Characteristic] 5f78df94-798c-46f5-990a-b3eb6a065c88 (Handle: 65): Unknown (read,notify), Value: bytearray(b'\x00')
#     [Descriptor] 00002902-0000-1000-8000-00805f9b34fb (Handle: 67): Client Characteristic Configuration, Value: bytearray(b'\x00\x00')
#   [Characteristic] 61c8849c-f639-4765-946e-5c3419bebb2a (Handle: 68): Unknown (read), Value: bytearray(b'\x81\x00')
#   [Characteristic] 64b4e8b5-0de5-401b-a21d-acc8db3b913a (Handle: 70): Unknown (read), Value: bytearray(b'\r')
#   [Characteristic] 42c3dfdd-77be-4d9c-8454-8f875267fb3b (Handle: 72): Unknown (read), Value: bytearray(b'\xf4\x00')
#   [Characteristic] b7de1eea-823d-43bb-a3af-c4903dfce23c (Handle: 74): Unknown (read), Value: bytearray(b'\xc8\x00')

# [bluetooth]# info 2C:BA:BA:2E:17:DB
# Device 2C:BA:BA:2E:17:DB (public)
# 	Name: Gear VR Controller(17DB)
# 	Alias: Gear VR Controller(17DB)
# 	Appearance: 0x03c0
# 	Paired: yes
# 	Bonded: yes
# 	Trusted: yes
# 	Blocked: no
# 	Connected: no
# 	LegacyPairing: no
# 	UUID: Generic Access Profile    (00001800-0000-1000-8000-00805f9b34fb)
# 	UUID: Generic Attribute Profile (00001801-0000-1000-8000-00805f9b34fb)
# 	UUID: Device Information        (0000180a-0000-1000-8000-00805f9b34fb)
# 	UUID: Battery Service           (0000180f-0000-1000-8000-00805f9b34fb)
# 	UUID: Unknown                   (00001879-0000-1000-8000-00805f9b34fb)
# 	UUID: Dialog Semiconductor GmbH (0000fef5-0000-1000-8000-00805f9b34fb)
# 	UUID: Vendor specific           (4f63756c-7573-2054-6872-65656d6f7465)
# 	ManufacturerData Key: 0x0075
# 	ManufacturerData Value:
#   01 00 02 00 fb 01 02 0e 03 76 72 73 65 74 75 70  .........vrsetup
#   77 69 7a 61 72 64 10                             wizard.         
# 	AdvertisingFlags:
#   06                                               . 
