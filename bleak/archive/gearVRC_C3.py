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
from copy import copy

from bleak      import BleakClient, BleakScanner
from bleak.exc  import BleakError
from bleak.backends.characteristic import BleakGATTCharacteristic

from pyIMU.quaternion import Vector3D, Quaternion
from pyIMU.utilities import q2rpy, heading

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

# Device name:
################################################################
DEVICE_NAME = 'Gear VR Controller(17DB)'
DEVICE_MAC  = '2C:BA:BA:2E:17:DB'

# LOCATION
################################################################
DECLINATION = 9.27 * DEG2RAD      # Delcination at your location
LATITUDE    = 32.253460 *DEG2RAD  # Tucson
ALTITUDE    = 730                 # [m], Tucson
MAGFIELD    = 47392               # [nano Tesla] Tucson

# Program Timing:
################################################################
MINIMUPDATETIME                  = 1./20. # 20 fps, gives warning
KEEPALIVEINTERVAL                = 10     # Every 10 secs, needed?
VIRTUALUPDATEINTERVAL            = 1/20.
FUSIONUPDATEINTERVAL             = 1/50.
REPORTINTERVAL                   = 1/10

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
WHEELRADIUS2                     = WHEELRADIUS**2
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
    data -= offset
    # Scale, assumed pos and neg sensitivity is the same
    data /= scale
    # Cross Correlation
    if crosscorr is not None:
        data = data.rotate(crosscorr)

    return data

################################################################
# gearVRC 
################################################################

class gearVRC:
            
    def __init__(self, device_name=None, logger=None, VRMode=False) -> None:

        # super(gearVRC, self).__init__()

        # Bluetooth device description
        self.device_name                        = device_name
  
        # Bluetooth device and client
        self._device                            = None
        self._client                            = None

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

        self.manufacturer_name      = ''
        self.model_number           = ''
        self.model_number           = ''
        self.hardware_revision      = ''
        self.firmware_revision      = ''
        self.software_revision      = ''
        self.pnp_ID                 = -1

        # Signals
        self.lost_connection        = asyncio.Event()
        self.connected              = asyncio.Event()
        self.dataAvailable       = asyncio.Event()
        self.virtualDataAvailable   = asyncio.Event()
        self.fusedDataAvailable     = asyncio.Event()
        self.sensorStarted          = asyncio.Event()
        self.finish_up              = asyncio.Event()
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger('gearVRC')

        self._VRMode                = VRMode

        # device sensors
        self.sensorTime             = 0.    # There are 3 different times transmitted from the sensor
        self.aTime                  = 0.    # Assuming the earliest time corresponds to IMU
        self.bTime                  = 0.    #
        self.delta_sensorTime       = 0.
        self.delta_aTime            = 0.
        self.delta_bTime            = 0.
        self._previous_sensorTime   = 0.
        self._previous_aTime        = 0.
        self._previous_bTime        = 0.
        # IMU
        self.accX                   = 0.    # IMU Accelerometer
        self.accY                   = 0.    #
        self.accZ                   = 0.    #
        self.gyrX                   = 0.    # IMY Gyroscope
        self.gyrY                   = 0.    #
        self.gyrZ                   = 0.    #
        self.magX                   = 0.    # IMU Magnetometer
        self.magY                   = 0.    #
        self.magZ                   = 0.    #
        # Touchpad
        self.touchX                 = 0.    # Touchpad Location (up/down)
        self.touchY                 = 0.    # (left/right)
        # Other
        self.temperature            = 0.    # Device Temperature
        self.battery_level          = 0.0   # Device Battery level
        # device buttons
        self.touch                  = False # Touchpad has been pressed
        self.trigger                = False # Trigger button pressed
        self.home                   = False # Home button pressed
        self.back                   = False # Back button pressed
        self.volume_up              = False # Volume up button pressed
        self.volume_down            = False # Volume down button pressed
        self.noButton               = True  # No button was pressed (mutually exclusive with the above)

        # virtual wheel
        self.wheelPos               = 0     # Wheel position 0..63
        self._previous_wheelPos     = 0
        self.delta_wheelPos         = 0
        self.top                    = False # Wheel touched in top quadrant
        self.bottom                 = False # Wheel touched in buttom quadrant
        self.left                   = False # Wheel touched in left quadrant
        self.right                  = False # Wheel touched in right quadrant
        self.center                 = False # Touchpad touched inside of wheel
        self.isRotating             = False # Moving along the rim of the wheel
        self.clockwise              = False # Moving clockwise or counter clockwise

        # scrolling
        # Touch pad
        self.absX                   = 0     # Position on virtual touchpad (scrolling)
        self.absY                   = 0     #
        self.dirUp                  = False # Touching pad and moving upwards
        self.dirDown                = False # Touching pad and moving downwards
        self.dirLeft                = False # Touching pad and moving to the left
        self.dirRight               = False # Touching pad and moving to the right
                        
        self._previous_touchX       = 0  # touch pad X
        self._previous_touchY       = 0  # touch pad Y

        self._data_lastTime         = time.perf_counter()
        self._data_lastTimeFPS      = time.perf_counter()
        self.data_deltaTime         = 0.
        self._data_updatecounts     = 0
        self.data_fps               = 0

        # NEED TO FIX THIS
        # add code if the sensor is not responding within timely fashion
        if self._VRMode: self._updateTime  = MINIMUPDATETIME
        else:            self._updateTime  = MINIMUPDATETIME

        # Virtual Wheel
        ###############
        wheelPositions              = [i for i in range(0, NUMWHEELPOS)]
        # shift by 45 degrees (want to create region for left, right, top , bottom)
        wheelPositions              = ror(wheelPositions, NUMWHEELPOS // 8)
        # make 4 equal length sections
        position_sections           = mit.divide(4,wheelPositions)
        # assign 4 sections to individual ranges: top, right, bottom, left
        [self._l_right, self._l_top, self._l_left, self._l_bottom] = [list(x) for x in position_sections]

        # IMU calibration
        # In future I will read calibration from file
        #  bias is offset so that 0 is in the middle of the range
        #  scale is gain so that 1 is the maximum value
        #  cross correlation is cross axis sensitivity
        self.acc_offset          = Vector3D(0,0,0)
        self.acc_scale           = Vector3D(1,1,1)
        self.acc_crosscorr       = np.array(([1,0,0], [0,1,0], [0,0,1]))
        self.gyr_offset          = Vector3D(0,0,0)
        self.gyr_scale           = Vector3D(1,1,1)
        # Not sure how to measure cross correlation for gyroscope
        self.mag_offset          = Vector3D(0,0,0)
        self.mag_scale           = Vector3D(1,1,1)
        self.mag_crosscorr       = np.array(([1,0,0], [0,1,0], [0,0,1]))

    def handle_disconnect(self,client):
        self.logger.log(logging.INFO,'Client disconnected, signaling...')
        self.connected.clear()
        self.lost_connection.set()

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
        while not self.finish_up.is_set(): # Run for ever

            # Scan for Device
            #################
            # try to find device using its name
            self.logger.log(logging.INFO,'Searching for {}'.format(self.device_name))            
            # if self._device is None and self.device_name is not None:
            if self.device_name is not None:
                self._device = await BleakScanner.find_device_by_name(self.device_name, timeout=5.0)

            # Connect to Device
            ###################
            if self._device is not None:
                # Create client
                self.logger.log(logging.INFO,'Found {}'.format(self.device_name))            
                self._client = BleakClient(self._device, disconnected_callback=self.handle_disconnect)
                # Connect to device
                if not (self._client is None):
                    if not self._client.is_connected:
                        await self._client.connect()
                        if self._client.is_connected:
                            self.connected.set() # signal we have connection
                            self.lost_connection.clear()
                            self.logger.log(logging.INFO,'Connected to {}'.format(self.device_name))
                            self.logger.log(logging.INFO,'Finding Characteristics')
                            self.find_characteristics() # scan and assign device characteristics
                            self.logger.log(logging.INFO,'Reading Device Information')
                            await self.read_deviceInformation() # populate device information
                            self.logger.log(logging.INFO,'Subscribing to Notifications')
                            await self.subscribe_notifications() # subscribe to device characteristics that have notification function
                            self.logger.log(logging.INFO,'Starting Sensor')
                            await self.start_sensor() # start the sensors
                        else:
                            self.connected.clear()
                            self.logger.log(logging.ERROR,"Could not connect to {}".format(self.device_name))
                    else:
                        self.logger.log(logging.INFO,'{} is already connected'.format(self.device_name))

                # Wait until disconnection occurs
                await self.lost_connection.wait()
                self.logger.log(logging.INFO,'Lost connection to {}'.format(self.device_name))            

                # # Attempt reconnection, device might still be available
                # if not self.finish_up.is_set():
                #     if self._client is not None:
                #         self.logger.log(logging.INFO,'Attempting reconnection') 
                #         await self._client.connect()
                #         if self._client.is_connected:
                #             self.connected.set()
                #             self.lost_connection.clear()
                #             self.logger.log(logging.INFO,'Reconnected to {}'.format(self.device_name))            
                #             # self.logger.log(logging.INFO,'Finding Characteristics')
                #             # self.find_characteristics()
                #             # self.logger.log(logging.INFO,'Reading Device Information')
                #             # await self.read_deviceInformation()
                #             # self.logger.log(logging.INFO,'Subscribing to Notifications')
                #             # await self.subscribe_notifications() # subscribe to device characteristics that have notification function
                #             self.logger.log(logging.INFO,'Starting Sensor')
                #             await self.start_sensor() # start the sensors
                #         else: # could not connect, reset
                #             self.connected.clear()
                #             self._client = None
                #             self._device = None
                #             self.logger.log(logging.INFO,'Could not reconnect to {}'.format(self.device_name))            
                #             await asyncio.sleep(5)                       
                #     else:
                #         self.connected.clear()
                #         # next loop iteration we want to scan for device and reconnect
                #         self._client = None   
                #         self._device = None
                #         self.logger.log(logging.ERROR,'Could not reconnect to {}'.format(self.device_name))            
                #         await asyncio.sleep(5)
                # else:
                #     self.logger.log(logging.INFO,'Not reconnecting, finishing up')            

            else: # Device not found
                self.logger.log(logging.INFO,'{} not found. Retrying...'.format(self.device_name))
                await asyncio.sleep(5)

    async def disconnect(self):
        '''
        Disconnect from sensor
        '''
        if self._client is not None:
            await self._client.disconnect()
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
        if self._client is not None:
            if self.battery_level_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.battery_level_characteristic)
                self.battery_level = int.from_bytes(value,byteorder='big')
            else:
                self.battery_level = -1
            self.logger.log(logging.INFO,'Battery Level: {}'.format(self.battery_level))                
        else:
            self.logger.log(logging.ERROR, 'Could not read Battery Level: disconnected')
        # Device Name
        if self._client is not None:
            if self.device_name_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.device_name_characteristic)
                self.device_name = value.decode('utf-8')
            else:
                self.device_name = DEVICE_NAME
            self.logger.log(logging.INFO,'Device Name: {}'.format(self.device_name))                
        else:
            self.logger.log(logging.ERROR, 'Could not read device name: disconnected')
        # Manufacturer
        if self._client is not None:
            if self.manufacturer_name_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.manufacturer_name_characteristic)
                self.manufacturer_name = value.decode('utf-8')
            else:
                self.manufacturer_name = ''
            self.logger.log(logging.INFO,'Manufacturer Name: {}'.format(self.manufacturer_name))                
        else:
            self.logger.log(logging.ERROR, 'Could not read manufacturer name: disconnected')
        # Model Number
        if self._client is not None:
            if self.model_number_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.model_number_characteristic)
                self.model_number = value.decode('utf-8')
            else:
                self.model_number = ''
            self.logger.log(logging.INFO,'Model Number: {}'.format(self.model_number))                
        else:
            self.logger.log(logging.ERROR, 'Could not read model number: disconnected')
        # Serial Number
        if self._client is not None:
            if self.serial_number_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.serial_number_characteristic)
                self.serial_number = value.decode('utf-8')
            else:
                self.serial_number = ''
            self.logger.log(logging.INFO,'Serial Number: {}'.format(self.serial_number))                
        else:
            self.logger.log(logging.ERROR, 'Could not read serial number: disconnected')
        # Hardware Revision
        if self._client is not None:
            if self.hardware_revision_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.hardware_revision_characteristic)
                self.hardware_revision = value.decode('utf-8')
            else:
                self.serial_number = ''
            self.logger.log(logging.INFO,'Hardware Revision: {}'.format(self.hardware_revision))                
        else:
            self.logger.log(logging.ERROR, 'Could not read hardware revision: disconnected')
        # Firmware Revision
        if self._client is not None:
            if self.firmware_version_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.firmware_version_characteristic)
                self.firmware_revision = int.from_bytes(value,'big')
            else:
                self.firmware_revision = -1
            self.logger.log(logging.INFO,'Firmware Revision: {}'.format(self.firmware_revision))
        else:
            self.logger.log(logging.ERROR, 'Could not read firmware revision: disconnected')
        # Software Revision
        if self._client is not None:
            if self.software_revision_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.software_revision_characteristic)
                self.software_revision = value.decode('utf-8')
            else:
                self.software_revision = ''
            self.logger.log(logging.INFO,'Software Revision: {}'.format(self.software_revision))
        else:
            self.logger.log(logging.ERROR, 'Could not read software revision: disconnected')
        # PnP ID
        if self._client is not None:
            if self.PnP_ID_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.PnP_ID_characteristic)
                self.pnp_ID = int.from_bytes(value,'big')
            else:
                self.pnp_ID = -1
            self.logger.log(logging.INFO,'PnP ID: {}'.format(self.pnp_ID))                
        else:
            self.logger.log(logging.ERROR, 'Could not read pnp id: disconnected')

    async def subscribe_notifications(self):
        ''' 
        Subscribe to data and battery notifications
        '''        
        if self._client is not None:
            # Sensor data
            if self.controller_data_characteristic is not None:
               await self._client.start_notify(self.controller_data_characteristic, self.handle_sensorData)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to data chracteristic')
            # Battery data
            if self.battery_level_characteristic is not None:
                await self._client.start_notify(self.battery_level_characteristic,   self.handle_batteryData)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to battery chracteristic')
        else:
            self.logger.log(logging.ERROR, 'No Client available for {}, can not susbscribe'.format(self.device_name))
                
    async def unsubscribe_notifications(self):
        '''
        Unsubscribe
        '''
        if self._client is not None:
            # unsubscribe from sensor data
            if self.controller_command_characteristics is not None:
                await self._client.stop_notify(self.controller_data_characteristic)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to data chracteristic')
            # unsubscribe from battery data
            if self.battery_level_characteristic is not None:
                await self._client.stop_notify(self.battery_level_characteristic)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to battery chracteristic')
        else:
            self.logger.log(logging.ERROR, 'No Client available for {}, can not unscubscribe'.format(self.device_name))

    def handle_sensorData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        '''
        Decode the sensor data
        Gear VRC reverse engineering (Java): https://github.com/jsyang/gearvr-controller-webbluetooth
        Additions includes the 3 different time stamps as well as correct magnetometer fields
        Keep this short so taht it just reads and decodes the data
        '''
        
        currentTime = time.perf_counter()
        
        # Update rate
        self._data_updatecounts += 1
        self.data_deltaTime = currentTime - self._data_lastTime
        self._data_lastTime = copy(currentTime)
        # self.logger.log(logging.DEBUG, 'Update rate: {}'.format(self.data_deltaTime))
        if self.data_deltaTime > self._updateTime: 
            self.logger.log(logging.INFO, 'Update rate slow')

        if currentTime - self._data_lastTimeFPS >= 1:
            self.data_fps = self._data_updatecounts
            self._data_lastTimeFPS = currentTime
            self._data_updatecounts = 0
            # self.logger.log(logging.DEBUG, 'FPS: {}'.format(self.data_fps))

        # Should receive 60 values
        if (len(data) >= 60):
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
            # self.logger.log(logging.DEBUG, 'Time: {:10.6f}, {:10.6f}, {:10.6f}'.format(self.sensorTime, self.aTime, self.bTime))

            self.delta_sensorTime = self.sensorTime - self._previous_sensorTime
            self.delta_aTime      = self.aTime      - self._previous_aTime
            self.delta_bTime      = self.bTime      - self._previous_bTime
            self._previous_sensorTime = copy(self.sensorTime)
            self._previous_aTime  = copy(self.aTime)
            self._previous_bTime  = copy(self.bTime)

            # Temperature:
            self.temperature = data[57]
            # self.logger.log(logging.DEBUG, 'Temperature: {} '.format(self.temperature))

            # Buttons
            self.trigger     = True if ((data[58] &  1) ==  1) else False
            self.home        = True if ((data[58] &  2) ==  2) else False
            self.back        = True if ((data[58] &  4) ==  4) else False
            self.touch       = True if ((data[58] &  8) ==  8) else False
            self.volume_up   = True if ((data[58] & 16) == 16) else False
            self.volume_down = True if ((data[58] & 32) == 32) else False
            self.noButton    = True if ((data[58] & 64) == 64) else False

            # self.logger.log(logging.DEBUG, 'Back button is {}'.format('pushed' if self.back else 'released'))
            # self.logger.log(logging.DEBUG, 'Home button is {}'.format('pushed' if self.home else 'released'))
            # self.logger.log(logging.DEBUG, 'Touchpad button is {}'.format('pushed' if self.touch else 'released'))
            # self.logger.log(logging.DEBUG, 'Trigger button is {}'.format('pushed' if self.trigger else 'released'))
            # self.logger.log(logging.DEBUG, 'Volume up button is {}'.format('pushed' if self.volume_up else 'released'))
            # self.logger.log(logging.DEBUG, 'Volume down button is {}'.format('pushed' if self.volume_down else 'released'))
            # self.logger.log(logging.DEBUG, '{} button is pushed'.format('No' if self.noButton else 'A'))

            # Touch pad:
            #  Max observed value = 315
            #  Location 0/0 is not touched
            #  Bottom right is largest number
            #  Y axis is up-down
            #  X axis is left-right 
            self.touchX     = ((data[54] & 0xF) << 6) + ((data[55] & 0xFC) >> 2) & 0x3FF
            self.touchY     = ((data[55] & 0x3) << 8) + ((data[56] & 0xFF) >> 0) & 0x3FF
            # self.logger.log(logging.DEBUG, 'Touchpad Position: {}, {}'.format(self.touchX, self.touchY))

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
            
            # self.logger.log(logging.DEBUG, 'Accel {:5.2f} {:5.2f} {:5.2f} '.format(self.acc.x,self.acc.y,self.acc.z))
            # self.logger.log(logging.DEBUG, 'Mag   {:5.2f} {:5.2f} {:5.2f} '.format(self.mag.x,self.mag.y,self.mag.z))
            # self.logger.log(logging.DEBUG, 'Gyro  {:5.2f} {:5.2f} {:5.2f} '.format(self.gyr.x,self.gyr.y,self.gyr.z))

            self.dataAvailable.set()

        else:
            self.logger.log(logging.INFO, 'Not enough values: {}'.format(len(data)))
           

    def handle_batteryData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        self.battery_level = int.from_bytes(data,'big')
        # self.logger.log(logging.DEBUG,'Battery level: {}'.format(self.battery_level))

    async def start_sensor(self):
        '''Start the sensors'''
        if self.connected.is_set():
            # Initialize the device, not sure why command needs to be sent twice
            if self._VRMode:
                self.logger.log(logging.INFO, 'Setting VR Mode')
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_VR_MODE)
            else:
                self.logger.log(logging.INFO, 'Setting Sensor Mode')
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
            self.sensorStarted.set()
        else:
            self.logger.log(logging.ERROR, 'Sensor not connected, can not send start signal')            
        
        self.logger.log(logging.INFO, 'Device Mode set')

    async def stop_sensor(self):
        '''Stop the sensors'''
        if self._client is not None:
            if self.connected.is_set() :
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_OFF)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_OFF)
                await self.unsubscribe_notifications()
                self.sensorStarted.clear()
                self.logger.log(logging.INFO,'Stopped sensor')
            else:
                self.logger.log(logging.ERROR,'Sensor not connected, can not send stop signal')
        else:
            self.logger.log(logging.ERROR, 'Could not stop: disconnected')

    async def keep_alive(self):
        '''
        Periodically send Keep Alive commands
        Does not prevent disconnection when device is left idle
        Not sure if this is needed.
        '''
        self.logger.log(logging.INFO, 'Keeping alive started')

        while not self.finish_up.is_set():
            if self._client is not None and self.connected.is_set() and self.sensorStarted.is_set() and (not self.lost_connection.is_set()):
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                self.logger.log(logging.INFO,'Keep alive sent')        
                sleepTime=KEEPALIVEINTERVAL
            else:
                if self.sensorStarted.is_set(): self.logger.log(logging.INFO,'Cound not send Keep alive')
                # do not report keep alive issues if sensor was not started            
                sleepTime=1
            await asyncio.sleep(sleepTime)
        
        self.logger.log(logging.INFO, 'Keeping alive stopped')

    async def update_report(self):
        '''
        Report latest fused data
        '''

        while not self.finish_up.is_set():
            # self.logger.log(logging.DEBUG, 'Waiting for fused data')
            await self.dataAvailable.wait()
            self.dataAvailable.clear()
            # self.logger.log(logging.DEBUG, 'IMU data available')

            # Display the Data
            self.logger.log(logging.INFO, '--------------------------------------------------------')
            self.logger.log(logging.INFO, 'gearVR Controller: Temp {:>4.1f}, Bat {:>3d}, fps: {:>3d} dT:{:>5.1f}'.format(self.temperature, self.battery_level, self.data_fps, self.data_deltaTime*1000.))
            self.logger.log(logging.INFO, '--------------------------------------------------------')
            self.logger.log(logging.INFO, 'Time  {:>10.6f}, {:>10.6f}, {:>10.6f}'.format(self.sensorTime, self.aTime, self.bTime))
            self.logger.log(logging.INFO, 'dt    {:>10.6f}, {:>10.6f}, {:>10.6f}'.format(self.delta_sensorTime, self.delta_aTime, self.delta_bTime))
            self.logger.log(logging.INFO, 'Accel {:>8.3f} {:>8.3f} {:>8.3f}'.format(self.accX,self.accY,self.accZ))
            self.logger.log(logging.INFO, 'Mag   {:>8.3f} {:>8.3f} {:>8.3f}'.format(self.magX,self.magY,self.magZ))
            self.logger.log(logging.INFO, 'Gyro  {:>8.3f} {:>8.3f} {:>8.3f}'.format(self.gyrX,self.gyrY,self.gyrZ))
            
            self.logger.log(logging.INFO, 'Pad:  {:>3d},{:>3d}'.format(
                                                self.touchX, self.touchY))
            
            self.logger.log(logging.INFO, 'Trig {} Touch {} Home {} Back {}, Vol up {} Vol down {} None: {}'.format(
                                                'Y' if self.trigger     else 'N', 
                                                'Y' if self.touch       else 'N', 
                                                'Y' if self.back        else 'N',
                                                'Y' if self.home        else 'N',
                                                'Y' if self.volume_up   else 'N',
                                                'Y' if self.volume_down else 'N',
                                                'Y' if self.noButton    else 'N'))

    async def handle_termination(self, tasks:None):
        self.logger.log(logging.INFO, 'Control-C or kill signal detected')
        self.finish_up.set()
        self.logger.log(logging.INFO, 'Stopping Sensor...')
        await self.stop_sensor()
        self.logger.log(logging.INFO, 'Disconnecting...')
        await self.disconnect()
        self.logger.log(logging.INFO, 'Cancelling all tasks...')
        if tasks is not None:
            await asyncio.sleep(1)
            for task in tasks:
                task.cancel()

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)

    # gearVRC Controller
    controller = gearVRC(device_name=args.name, logger=logger, VRMode=args.vrmode)

    connection_task = asyncio.create_task(controller.connect())        # remain connected, will not terminate
    keepalive_task  = asyncio.create_task(controller.keep_alive())     # keep sensor alive, will not terminate
    if args.report:
        reporting_task  = asyncio.create_task(controller.update_report())         # report new data, will not terminate
        tasks = [connection_task, keepalive_task, reporting_task ]
    else:
        tasks = [connection_task, keepalive_task]
 
    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(controller.handle_termination(tasks)) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(controller.handle_termination(tasks)) ) # kill

    # These tasks will not terminate 
    await asyncio.wait(tasks, timeout=float('inf'))

    logger.log(logging.INFO,'Exit')

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()

    device_group = parser.add_mutually_exclusive_group(required=True)

    device_group.add_argument(
        '-n',
        '--name',
        metavar='<name>',
        help='the name of the bluetooth device to connect to',
        default = 'Gear VR Controller(17DB)'
    )

    parser.add_argument(
        '-vr',
        '--vrmode',
        action='store_true',
        help='sets the vrmode, sensor mode is used otherwise',
    )

    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level to debug',
    )

    parser.add_argument(
        '-r',
        '--report',
        action='store_true',
        help='turns on logging of sensor values',
    )

    args = parser.parse_args()
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)-15s %(name)-8s %(levelname)s: %(message)s',
    )   
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass

# python gearVRC_C.py -n 'Gear VR Controller(17DB)'