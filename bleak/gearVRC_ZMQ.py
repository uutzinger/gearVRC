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
#   You will need to manually remove the device from the system 
#     each time before using it. 
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
import serial_asyncio
import re
import msgpack


from bleak      import BleakClient, BleakScanner
from bleak.exc  import BleakError
from bleak.backends.characteristic import BleakGATTCharacteristic

from pyIMU.madgwick import Madgwick
from pyIMU.quaternion import Vector3D, Quaternion
from pyIMU.utilities import q2rpy, heading

import zmq

if os.name != 'nt':
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

BAUDRATE = 115200
ZMQPORT = 5556

# Device name:
################################################################
DEVICE_NAME = 'Gear VR Controller(17DB)'
DEVICE_MAC  = '2C:BA:BA:2E:17:DB'

# LOCATION
################################################################
DECLINATION  = 9.27 * DEG2RAD      # Delcination at your location
LATITUDE     = 32.253460 *DEG2RAD  # Tucson
ALTITUDE     = 730                 # [m], Tucson
MAGFIELD     = 47392               # [nano Tesla] Tucson
MAGFIELD_MAX = 1.2 *MAGFIELD/1000. # micro Tesla
MAGFIELD_MIN = 0.8*MAGFIELD/1000.  # micro Tesla

# Program Timing:
################################################################
MINIMUPDATETIME                  = 1./20. # 20 fps, gives warning
KEEPALIVEINTERVAL                = 10     # Every 10 secs, needed?
VIRTUALUPDATEINTERVAL            = 1/20.
REPORTINTERVAL                   = 1/10

# Device commands:
################################################################
CMD_OFF                          = bytearray(b'\x00\x00')   # Turn modes off and stop sending data
CMD_SENSOR                       = bytearray(b'\x01\x00')   # Touchpad and sensor buttons but low rate IMU data
CMD_UNKNOWN_FIRMWARE_UPDATE_FUNC = bytearray(b'\x02\x00')   # Initiate firmware update sequence
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
#  The touchpad has a limited resolution and we can extend 
#  the range by scrolling: lifting the finger and moving in same direction again
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

def calibrate(data:Vector3D, offset:Vector3D, crosscorr=None):
    ''' 
    IMU calibration
    bias is offset so that 0 is in the middle of the range
    scale is the gain so that 1 is the maximum value
    cross correlation is cross axis sensitivity
    the diagonal elements of the cross correlation matrix are the scale

    Expects data and bias to be a Vector3D
    Expects crosscorr to be 3x3 numpy array
    '''
    # Bias
    data -= offset
    # Cross Correlation
    if crosscorr is not None:
        data = data.rotate(crosscorr)

    return data

# This will create readable text, for example to send numbers over serial connection
def float_to_hex(f):
    '''Pack float into 8 characters representing '''
    bytes_ = struct.pack('!f', f)                            # Single precision, big-endian byte order, 4 bytes
    hex_strings = [format(byte, '02X') for byte in bytes_]   # Convert each byte to a hexadecimal string
    return ''.join(hex_strings)

def hex_to_float(hex_chars):
    '''Unpack 8 bytes to float'''
    hex_bytes = bytes.fromhex(hex_chars)  # Convert hex characters to bytes
    return struct.unpack('!f', hex_bytes)[0]     

def int_to_hex(n):
    '''Pack integer to 8 bytes'''
    bytes_ = n.to_bytes((n.bit_length() + 7) // 8, 'big')  # Convert integer to bytes
    hex_strings = [format(byte, '02X') for byte in bytes_]           # Convert each byte to a hexadecimal string
    return ''.join(hex_strings)

def hex_to_int(hex_chars):
    '''Unpack bytes to integer'''
    hex_bytes = bytes.fromhex(hex_chars)  # Convert hex characters to bytes
    return struct.unpack('!i', hex_bytes)[0]

# Data Packets to be sent with ZMQ

class gearSystemData(object):
    '''System relevant performance data'''
    def __init__(self, 
                 data_fps: int = 0, virtual_fps: int = 0, fusion_fps: int = 0, zmq_fps: int = 0, reporting_fps: int =0) -> None:
        self.data_fps       = data_fps
        self.virtual_fps    = virtual_fps
        self.fusion_fps     = fusion_fps
        self.zmq_fps        = zmq_fps
        self.reporting_fps  = reporting_fps

class gearRawData(object):
    '''Raw Data from the sensor'''
    def __init__(self, 
                 temperature: float=0.0, battery_level: float =-1.0, 
                 sensorTime: float=0.0, aTime: float=0.0, bTime: float=0.0,
                 accX: float=0.0, accY: float=0.0, accZ: float=0.0,
                 gyrX: float=0.0, gyrY: float=0.0, gyrZ: float=0.0,
                 magX: float=0.0, magY: float=0.0, magZ: float=0.0,
                 trigger: bool = False, touch: bool = False, back: bool = False, home: bool = False, 
                 volume_up: bool = False, volume_down: bool = False, noButton: bool = False,
                 touchX: int = 0, touchY: int = 0) -> None:
        
        self.temperature    = temperature
        self.battery_level  = battery_level
        self.sensorTime     = sensorTime
        self.aTime          = aTime
        self.bTime          = bTime
        self.accX           = accX
        self.accY           = accY
        self.accZ           = accZ
        self.magX           = magX
        self.magY           = magY
        self.magZ           = magZ
        self.gyrX           = gyrX
        self.gyrY           = gyrY
        self.gyrZ           = gyrZ
        self.trigger        = trigger
        self.touch          = touch
        self.back           = back
        self.home           = home
        self.volume_up      = volume_up
        self.volume_down    = volume_down
        self.noButton       = noButton
        self.touchX         = touchX
        self.touchY         = touchY

class gearVirtualData(object):
    '''Virtual wheel and touchpad data'''
    def __init__(self, 
                 absX: int = 0, absY: int = 0,
                 dirUp: bool = False, dirDown: bool = False, dirLeft: bool = False, dirRight: bool = False,
                 wheelPos: int = 0,
                 top: bool = False, bottom: bool = False, left: bool = False, right: bool = False, center: bool = False,
                 clockwise: bool = False, isRotating: bool = False) -> None:
                 
        self.absX           = absX
        self.absY           = absY
        self.dirUp          = dirUp
        self.dirDown        = dirDown
        self.dirLeft        = dirLeft
        self.dirRight       = dirRight
        self.wheelPos       = wheelPos
        self.top            = top
        self.bottom         = bottom
        self.left           = left
        self.right          = right
        self.center         = center
        self.clockwise      = clockwise
        self.isRotating     = isRotating

class gearFusionData(object):
    '''AHRS fusion data'''
    def __init__(self, 
                 acc: Vector3D = Vector3D(0,0,0), mag: Vector3D = Vector3D(0,0,0), gyr: Vector3D = Vector3D(0,0,0), 
                 rpy: Vector3D = Vector3D(0,0,0), heading: float = 0.0, q: Quaternion = Quaternion(1,0,0,00)) -> None:
        self.acc            = acc
        self.mag            = mag
        self.gyr            = gyr
        self.rpy            = rpy
        self.heading        = heading
        self.q              = q

################################################################
# gearVRC 
################################################################

class gearVRC:
            
    def __init__(self, device_name=None, device_address = None, logger=None, VRMode=False) -> None:

        # super(gearVRC, self).__init__()

        # Bluetooth device description
        self.device_name                        = device_name
        self.device_address                     = device_address
  
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
        self.dataAvailable          = asyncio.Event()
        self.virtualDataAvailable   = asyncio.Event()
        self.fusedDataAvailable     = asyncio.Event()
        self.reportingDataAvailable = asyncio.Event()
        self.sensorStarted          = asyncio.Event()
        self.finish_up              = asyncio.Event()
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger('gearVRC')

        self._VRMode                = VRMode
        self.HighResMode            = False

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
        self.touchX                 = 0     # Touchpad Location (up/down) 
        self.touchY                 = 0     # (left/right)
        
        # Other
        self.temperature            = 0.    # Device Temperature
        self.battery_level          = 0.0   # Device Battery level

        # Device buttons 
        self.touch                  = False # Touchpad has been pressed
        self.trigger                = False # Trigger button pressed
        self.home                   = False # Home button pressed
        self.back                   = False # Back button pressed
        self.volume_up              = False # Volume up button pressed
        self.volume_down            = False # Volume down button pressed
        self.noButton               = True  # No button was pressed (mutually exclusive with the above)

        # Virtual wheel
        self.wheelPos               = 0     # Wheel position 0..63
        self._previous_wheelPos     = 0
        self.delta_wheelPos         = 0
        self.top                    = False # Wheel touched in top quadrant
        self.bottom                 = False # Wheel touched in bottom quadrant
        self.left                   = False # Wheel touched in left quadrant
        self.right                  = False # Wheel touched in right quadrant
        self.center                 = False # Touchpad touched inside of wheel
        self.isRotating             = False # Moving along the rim of the wheel
        self.clockwise              = False # Moving clockwise or counter clockwise

        # Scrolling
        # Touch pad
        self.absX                   = 0     # Position on virtual touchpad (scrolling)
        self.absY                   = 0     #
        self.dirUp                  = False # Touching pad and moving upwards
        self.dirDown                = False # Touching pad and moving downwards
        self.dirLeft                = False # Touching pad and moving to the left
        self.dirRight               = False # Touching pad and moving to the right
                        
        self._previous_touchX       = 0  # touch pad X
        self._previous_touchY       = 0  # touch pad Y


        # Timing
        ###################
        self._data_lastTime         = time.perf_counter()
        self._data_lastTimeFPS      = time.perf_counter()
        self.data_deltaTime         = 0.
        self._data_updateCounts     = 0
        self.data_fps               = 0

        self._virtual_lastTimeFPS   = time.perf_counter()
        self._previous_virtualUpdate= time.perf_counter()
        self.virtual_deltaTime      = 0.
        self._virtual_updateCounts  = 0
        self._virtual_updateInterval= VIRTUALUPDATEINTERVAL
        self.virtual_fps            = 0

        self._fusion_lastTimeFPS    = time.perf_counter()
        self._previous_fusionUpdate = time.perf_counter()
        self.fusion_deltaTime       = 0.
        self._fusion_updateCounts   = 0
        self.fusion_fps             = 0
        self._previous_fusionTime   = 0. # The difference between the sensor time stamps

        self._report_lastTimeFPS    = time.perf_counter()
        self._previous_reportUpdate = time.perf_counter()
        self.report_deltaTime       = 0.
        self._report_updateCounts   = 0
        self._report_updateInterval = REPORTINTERVAL
        self.report_fps             = 0

        self._serial_lastTimeFPS    = time.perf_counter()
        self._previous_serialUpdate = time.perf_counter()
        self.serial_deltaTime       = 0.
        self._serial_updateCounts   = 0
        self._serial_updateInterval = REPORTINTERVAL
        self.serial_fps             = 0

        #
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
        # In future I will read calibration data from file
        #  bias is offset so that 0 is in the middle of the range
        #  cross correlation is cross axis sensitivity, diagonal elements are the scale
        #  scale is gain so that 1 is the maximum value and it is the diagonale of the cross correlation matrix
        self.acc_offset          = Vector3D(0.,0.,0.)
        self.acc_crosscorr       = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
        self.gyr_offset          = Vector3D(-0.0104,-0.0135,-0.0391)
        self.gyr_crosscorr       = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
        self.mag_offset          = Vector3D(223,48.25,-35.5)
        self.mag_crosscorr       = np.array(([1./1.045,0.,0.], [0.,1./0.961,0.], [0.,0.,1./0.994]))

        self.acc         = Vector3D(0.,0.,0.)
        self.gyr         = Vector3D(0.,0.,0.)
        self.mag         = Vector3D(0.,0.,0.)
        self.gyr_average = Vector3D(0.,0.,0.)

        # Attitude fusion
        self.AHRS                = Madgwick()
        self.q                   = Quaternion(1.,0.,0.,0.)
        self.heading             = 0.
        self.rpy                 = Vector3D(0.,0.,0.)

    def handle_disconnect(self,client):
        self.logger.log(logging.INFO,'Client disconnected, signaling...')
        self.connected.clear()
        self.lost_connection.set()

    async def update_connect(self):
        '''
        This is most tricky part of the code to remain connected with the device.
        If device connection is interrupted because of signal loss it does not yet recover.
        
        Connection Loop:
          Scan for Device
            Create Client
                Connect to Device
                Scan for Services & Characteristics

            Wait for Disconnection
            Attempt Reconnection
          Back to Scan for Device            
        '''

        first_time = True

        while not self.finish_up.is_set(): # Run for ever

            # print('C', end='', flush=True)

            # Scan for Device
            #################
            # try to find device using its name
            self.logger.log(logging.INFO,'Searching for {}'.format(self.device_name))            
            if self.device_name is not None:
                self._device = await BleakScanner.find_device_by_name(self.device_name, timeout=10.0)
            if self._device is None:
                if self.device_address is not None:
                    self._device = await BleakScanner.find_device_by_address(self.device_address, timeout=10.0)

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
                            # this needs to be run each time we create a new client
                            self.find_characteristics() # scan and assign device characteristics
                            if first_time:
                                # We dont need to read device information each time, as it remains the same
                                self.logger.log(logging.INFO,'Reading Device Information')
                                await self.read_deviceInformation() # populate device information
                                first_time = False
                            # this needs to be run each time we create a new client
                            self.logger.log(logging.INFO,'Starting Sensor')
                            await self.start_sensor(self._VRMode) # start the sensors
                            # this needs to be run each time we create a new client
                            self.logger.log(logging.INFO,'Subscribing to Notifications')
                            await self.subscribe_notifications() # subscribe to device characteristics that have notification function
                            self.logger.log(logging.INFO,'Setup completed')
                        else:
                            self.connected.clear()
                            self.logger.log(logging.ERROR,"Could not connect to {}".format(self.device_name))
                    else:
                        self.logger.log(logging.INFO,'{} is already connected'.format(self.device_name))

                # Wait until disconnection occurs
                await self.lost_connection.wait()
                self.logger.log(logging.INFO,'Lost connection to {}'.format(self.device_name))            

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
        This needs to be read only once.
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
                self.device_name = value.decode()
            else:
                self.device_name = DEVICE_NAME
            self.logger.log(logging.INFO,'Device Name: {}'.format(self.device_name))                
        else:
            self.logger.log(logging.ERROR, 'Could not read device name: disconnected')
        # Manufacturer
        if self._client is not None:
            if self.manufacturer_name_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.manufacturer_name_characteristic)
                self.manufacturer_name = value.decode()
            else:
                self.manufacturer_name = ''
            self.logger.log(logging.INFO,'Manufacturer Name: {}'.format(self.manufacturer_name))                
        else:
            self.logger.log(logging.ERROR, 'Could not read manufacturer name: disconnected')
        # Model Number
        if self._client is not None:
            if self.model_number_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.model_number_characteristic)
                self.model_number = value.decode()
            else:
                self.model_number = ''
            self.logger.log(logging.INFO,'Model Number: {}'.format(self.model_number))                
        else:
            self.logger.log(logging.ERROR, 'Could not read model number: disconnected')
        # Serial Number
        if self._client is not None:
            if self.serial_number_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.serial_number_characteristic)
                self.serial_number = value.decode()
            else:
                self.serial_number = ''
            self.logger.log(logging.INFO,'Serial Number: {}'.format(self.serial_number))                
        else:
            self.logger.log(logging.ERROR, 'Could not read serial number: disconnected')
        # Hardware Revision
        if self._client is not None:
            if self.hardware_revision_characteristic is not None and self.connected.is_set():
                value = await self._client.read_gatt_char(self.hardware_revision_characteristic)
                self.hardware_revision = value.decode()
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
                self.software_revision = value.decode()
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

    async def handle_sensorData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        '''
        Decode the sensor data
        Gear VRC reverse engineering (Java): https://github.com/jsyang/gearvr-controller-webbluetooth
        Additions includes the 3 different time stamps as well as correct magnetometer fields
        Keep this short so taht it just reads and decodes the data
        '''
         
        currentTime = time.perf_counter()
        
        # Update rate
        self._data_updateCounts += 1
        self.data_deltaTime = currentTime - self._data_lastTime
        self._data_lastTime = copy(currentTime)
        # self.logger.log(logging.DEBUG, 'Update rate: {}'.format(self.data_deltaTime))
        if self.data_deltaTime >= self._updateTime: 
            self.logger.log(logging.DEBUG, 'Update rate slow')

        if currentTime - self._data_lastTimeFPS >= 1:
            self.data_fps = self._data_updateCounts
            self._data_lastTimeFPS = currentTime
            self._data_updateCounts = 0
            # self.logger.log(logging.DEBUG, 'FPS: {}'.format(self.data_fps))

        # After 20 successful runs and low frequency updates, try to engage VRMode
        # Not Implemented

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
            # self.logger.log(logging.INFO, 'Time: {:10.6f}, {:10.6f}, {:10.6f}'.format(self.sensorTime, self.aTime, self.bTime))

            self.delta_sensorTime     = self.sensorTime - self._previous_sensorTime
            self.delta_aTime          = self.aTime      - self._previous_aTime
            self.delta_bTime          = self.bTime      - self._previous_bTime
            self._previous_sensorTime = copy(self.sensorTime)
            self._previous_aTime      = copy(self.aTime)
            self._previous_bTime      = copy(self.bTime)
            # self.logger.log(logging.INFO, 'Dt:   {:>10.6f}, {:>10.6f}, {:>10.6f}'.format(self.delta_sensorTime, self.delta_aTime, self.delta_bTime))

            # Temperature:
            self.temperature = data[57]

            # Buttons
            self.trigger     = True if ((data[58] &  1) ==  1) else False
            self.home        = True if ((data[58] &  2) ==  2) else False
            self.back        = True if ((data[58] &  4) ==  4) else False
            self.touch       = True if ((data[58] &  8) ==  8) else False
            self.volume_up   = True if ((data[58] & 16) == 16) else False
            self.volume_down = True if ((data[58] & 32) == 32) else False
            self.noButton    = True if ((data[58] & 64) == 64) else False

            # Touch pad:
            #  Max observed value = 315
            #  Location 0/0 is not touched
            #  Bottom right is largest number
            #  Y axis is up-down
            #  X axis is left-right 
            self.touchX     = ((data[54] & 0xF) << 6) + ((data[55] & 0xFC) >> 2) & 0x3FF
            self.touchY     = ((data[55] & 0x3) << 8) + ((data[56] & 0xFF) >> 0) & 0x3FF

            # IMU:
            #  Accelerometer, Gyroscope, Magnetometer
            #  Magnetometer often has large hard iron offset
            #
            self.accX = struct.unpack('<h', data[4:6])[0]   * 0.00478840332  # 10000.0 * 9.80665 / 2048 / 10000.0       # m**2/s
            self.accY = struct.unpack('<h', data[6:8])[0]   * 0.00478840332  #
            self.accZ = struct.unpack('<h', data[8:10])[0]  * 0.00478840332  #
            self.gyrX = struct.unpack('<h', data[10:12])[0] * 0.001221791529 # 10000.0 * 0.017453292 / 14.285 / 1000.0 / 10. # rad/s
            self.gyrY = struct.unpack('<h', data[12:14])[0] * 0.001221791529 #
            self.gyrZ = struct.unpack('<h', data[14:16])[0] * 0.001221791529 #
            self.magY = struct.unpack('<h', data[48:50])[0] * 0.06           # micro Tesla?, earth mag field 25..65 muTesla
            self.magX = struct.unpack('<h', data[50:52])[0] * 0.06           #
            self.magZ = struct.unpack('<h', data[52:54])[0] * 0.06           #

            # Rearrange the Axis
            #
            # It was found for gearVRC when holding device:
            #
            # acc.x points to the left/west
            # acc.y points towards user
            # acc.z points downwards
            # -Y-X+Z
            #
            # mag.x points to the right/east
            # mag.y points to the user
            # mag.z points downwards
            # -Y+X+Z
            #
            # gyr.x is counter clockwise around x
            # gyr.y is counter clockwise around y
            # gyr.z is counter clockwise around z
            # +Y+X-Z
            # 
            # We want x to point North y to East and z to Down Orientation:
            #   acc.x pointing forward      -acc.y
            #   acc.y pointing to the right -acc.x
            #   acc.z pointing downwards     acc.z
            #   mag.x pointing forward      -mag.y
            #   mag.y pointing to the right  mag.x
            #   mag.z pointing down          mag.z
            #   gyr.x clock wise forward     gyr.y
            #   gyr.y clockwise to the right gyr.x
            #   gyr.z clockwise downwards   -gyr.z

            self.acc = Vector3D(-self.accY, -self.accX,  self.accZ)
            self.gyr = Vector3D( self.gyrY,  self.gyrX, -self.gyrZ)
            self.mag = Vector3D(-self.magY,  self.magX,  self.magZ)

            self.gyr_average = 0.99*self.gyr_average + 0.01*self.gyr

            self.dataAvailable.set()
            self.reportingDataAvailable.set()

        else:
            self.logger.log(logging.INFO, 'Not enough values: {}'.format(len(data)))
            # switch to sensor mode
            await self.start_sensor(VRMode = False)

    def handle_batteryData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        self.battery_level = int.from_bytes(data,'big')

    async def start_sensor(self, VRMode):
        '''Start the sensors'''
        if self.connected.is_set():
            # Initialize the device, not sure why command needs to be sent twice

            # Start
            # self.write(bytearray(CMD_SENSOR), 3) 2 times
            # self.write(bytearray(CMD_LPM_ENABLE), 1) not running 
            # self.write(bytearray(CMD_LPM_DISABLE), 1) not running
            # self.write(bytearray(CMD_VR_MODE), 3) 2 times

            # Switch to VR MODE while running
            # self.write(bytearray(CMD_LPM_ENABLE), 1) will not run
            # self.write(bytearray(CMD_VR_MODE), 3) runs only 2 times
            # self.write(bytearray(CMD_LPM_DISABLE), 1) will not run

            # Switch to SENSOR MODE while running
            # self.write(bytearray(CMD_SENSOR), 3) runs only two times

            if VRMode:
                self.logger.log(logging.INFO, 'Setting VR Mode')
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_VR_MODE)
                self.HighResMode = True
            else:
                self.logger.log(logging.INFO, 'Setting Sensor Mode')
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
                self.HighResMode = False
            self.sensorStarted.set()
        else:
            self.logger.log(logging.ERROR, 'Sensor not connected, can not send start signal')            
        
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

    async def update_EscSequence(self, counts=3, timeout=1.0):
        '''
        Check if home button is pressed three times in a row within timeout seconds
        '''

        self.logger.log(logging.INFO, 'ESC Detection started')

        # Keep track of home button presses
        previous_home = False
        home_updateCounts = 0
        home_pressedTime = time.perf_counter()

        while not self.finish_up.is_set():

            await self.dataAvailable.wait()
            
            if self.home == True:
                if not previous_home:
                    # Home button pressed
                    elapsed = time.perf_counter() - home_pressedTime
                    home_updateCounts += 1
                    previous_home = True
                    
                    if home_updateCounts == 1:
                        home_pressedTime = time.perf_counter()
                    
                    elif home_updateCounts == counts:
                        if elapsed <= timeout:
                            # ESC sequence detected, signal finish
                            self.logger.log(logging.INFO, 'ESC detected')
                            self.handle_termination()
                        else:
                            # To slow, ESC not detected, reset
                            home_updateCounts = 0
                else: 
                    # it remains pressed
                    pass
            else:
                # its not pressed
                previous_home = False

        self.logger.log(logging.INFO, 'ESC Detection stopped')
            
    async def update_virtual(self):
        ''' 
        Create Virtual Wheel
        Create Extended Touchpad
        Inspired from https://github.com/rdady/gear-vr-controller-linux
        '''

        self.logger.log(logging.INFO, 'Virtual started')

        while not self.finish_up.is_set():

            # self.logger.log(logging.DEBUG, 'Waiting for sensor data')
            await self.dataAvailable.wait()
            self.dataAvailable.clear()

            currentTime = time.perf_counter()

            self.virtual_deltaTime = currentTime - self._previous_virtualUpdate
            self._previous_virtualUpdate = copy(currentTime)

            self._virtual_updateCounts += 1
            if (currentTime - self._virtual_lastTimeFPS)>= 1.:
                self.virtual_fps = self._virtual_updateCounts
                self._virtual_lastTimeFPS = copy(currentTime)
                self._virtual_updateCounts = 0

            # Where are we touching the pad?
            if not (self.touchX == 0 and self.touchY==0): 

                # Virtual Wheel
                #  Detects if rim of touchpad is touched and where 0..63
                #  Detects finger moves along wheel (rotation clockwise or counter clockwise)
                #  Detects if rim touched on top, left, right, bottom

                x = self.touchX - WHEELRADIUS # horizontal distance from center of touchpad
                y = self.touchY - WHEELRADIUS # vertical distance from center
                l2 = x*x + y*y                # distance from center (squared)
                if l2 > RTHRESH2:             # Wheel is touched
                    self.center = False
                    phi = (math.atan2(y,x) + TWOPI) % TWOPI # angle 0 .. 2*pi from pointing to the right counter clockwise
                    self.wheelPos = int(math.floor(phi / TWOPI * NUMWHEELPOS))
                    # Top, Bottom, Left, Right
                    if self.wheelPos > NUMWHEELPOS1_8:
                        if self.wheelPos < NUMWHEELPOS3_8:
                            self.top    = False
                            self.left   = False
                            self.bottom = True
                            self.right  = False
                        elif self.wheelPos < NUMWHEELPOS5_8:
                            self.top    = False
                            self.left   = True
                            self.bottom = False
                            self.right  = False
                        elif self.wheelPos < NUMWHEELPOS7_8:
                            self.top    = True
                            self.left   = False
                            self.bottom = False
                            self.right  = False
                        else: 
                            self.top    = False
                            self.left   = False
                            self.bottom = False
                            self.right  = True

                    self.logger.log(logging.DEBUG, 'Wheel Position: {}'.format(self.wheelPos))

                    self.delta_wheelPos = self.wheelPos - self._previous_wheelPos
                    self._previous_wheelPos = copy(self.wheelPos)
                    # deal with discontinuity 360/0:
                    #   general formula with intervals along a circle in degrees is
                    #   a0 = a0 - (360. * np.floor((a0 + 180.)/360.))
                    #   now a0 will be < 180 and >=-180
                    self.delta_wheelPos -= int(MAXWHEELPOS * np.floor((self.delta_wheelPos + MAXWHEELPOS2)/MAXWHEELPOS))
                    self.logger.log(logging.DEBUG, 'Delta Wheel Position: {}'.format(self.delta_wheelPos))
                    if (self.delta_wheelPos > 0):
                        # rotating clock wise
                        self.isRotating = True
                        self.clockwise  = True
                        self.logger.log(logging.DEBUG, 'Wheel rotating clockwise')
                    elif (self.delta_wheelPos < 0):
                        # rotating counter clock wise
                        self.isRotating = True
                        self.clockwise  = False
                        self.logger.log(logging.DEBUG, 'Wheel rotating counter clockwise')
                    else:
                        self.isRotating = False
                        self.logger.log(logging.DEBUG, 'Wheel not rotating')

                else: # wheel not touched, but touchpad is touched
                
                    # Virtual Pad
                    #  Detects scrolling of touchpad to create absolute 'mouse' position, allowing to reach larger field than touchpad allone        
                    self.isRotating = False
                    self.top        = False
                    self.left       = False
                    self.bottom     = False
                    self.right      = False
                    self.center     = True

                    # Movement direction on touchpad
                    # This is to assess scrolling
                    self.deltaX = self.touchX - self._previous_touchX
                    self.deltaY = self.touchY - self._previous_touchY
                    self._previous_touchX = copy(self.touchX)
                    self._previous_touchY = copy(self.touchY)
                    if (abs(self.deltaX) < 50) and (abs(self.deltaY) < 50): # disregard large jumps such as when lifting finger between scrolling
                        self.absX += self.deltaX 
                        self.absY += self.deltaY 
                        self.absX  = clamp(self.absX, MINXTOUCH, MAXXTOUCH)
                        self.absY  = clamp(self.absY, MINYTOUCH, MAXYTOUCH)
                        # Left or Right?
                        if (self.deltaX > 0):
                            self.dirRight = True
                            self.dirLeft  = False
                        elif (self.deltaX < 0):
                            self.dirLeft  = True
                            self.dirRight = False
                        else:
                            self.dirLeft  = False
                            self.dirRight = False
                        # Up or Down?
                        if (self.deltaY > 0):
                            self.dirDown  = True
                            self.dirUp    = False
                        elif (self.deltaY < 0):
                            self.dirUp    = True
                            self._dirDown = False
                        else:
                            self.dirUp    = False
                            self.dirDown  = False

                self.virtualDataAvailable.set() 
                
            else: # Touch pad was not touched
                self.center   = False
                self.top      = False
                self.left     = False
                self.bottom   = False
                self.right    = False
                self.dirUp    = False
                self.dirDown  = False
                self.dirLeft  = False
                self.dirRight = False

            # Wait to next interval time
            sleepTime = self._virtual_updateInterval - (time.perf_counter() - currentTime)
            if sleepTime > 0:
                await asyncio.sleep(sleepTime)

        self.logger.log(logging.INFO, 'Virtual stopped')

    async def update_fusion(self):
        ''' 
        Fuse data to pose
        '''

        self.logger.log(logging.INFO, 'Fusion started')

        while not self.finish_up.is_set():

            # print('F', end='', flush=True)

            await self.dataAvailable.wait()
            self.dataAvailable.clear()       # This is primary consumer, one of the tasks will need to clear it

            currentTime = time.perf_counter()

            # update interval
            self.fusion_deltaTime = currentTime - self._previous_fusionUpdate
            self._previous_fusionUpdate = copy(currentTime)

            # fps
            self._fusion_updateCounts += 1
            if (currentTime - self._fusion_lastTimeFPS)>= 1.:
                self.fusion_fps = self._fusion_updateCounts
                self._fusion_lastTimeFPS = copy(currentTime)
                self._fusion_updateCounts = 0

            # Fusion data interval, is computed from sensor provided time stamp
            dt = self.sensorTime - self._previous_fusionTime # time interval between sensor data
            self._previous_fusionTime = copy(self.sensorTime) # keep track of last sensor time

            # Calibrate IMU Data
            self.acc = calibrate(data=self.acc, offset=self.acc_offset, crosscorr=self.acc_crosscorr)
            self.mag = calibrate(data=self.mag, offset=self.mag_offset, crosscorr=self.mag_crosscorr)
            self.gyr = calibrate(data=self.gyr, offset=self.gyr_offset, crosscorr=self.gyr_crosscorr)

            if dt > 1.0: 
                # First run or reconnection, need AHRS algorythem to initilize again
                if (self.mag.norm >  MAGFIELD_MAX) or (self.mag.norm < MAGFIELD_MIN):
                    # Mag is not in acceptable range
                    self.q = self.AHRS.update(acc=self.acc,gyr=self.gyr,mag=None,dt=-1)
                else:
                    self.q = self.AHRS.update(acc=self.acc,gyr=self.gyr,mag=None,dt=-1)

            else:
                if (self.mag.norm >  MAGFIELD_MAX) or (self.mag.norm < MAGFIELD_MIN):
                    # Mag not in acceptable range
                    self.q = self.AHRS.update(acc=self.acc,gyr=self.gyr,mag=None,dt=dt)
                else:
                    self.q = self.AHRS.update(acc=self.acc,gyr=self.gyr,mag=None,dt=dt)


            # stuck here
            self.heading=heading(pose=self.q, mag=self.mag, declination=DECLINATION)
            self.rpy=q2rpy(pose=self.q)

            self.fusedDataAvailable.set()

            # Dont wait here, we want to fuse every IMU reading

        self.logger.log(logging.INFO, 'Fusion stopped')


    async def keep_alive(self):
        '''
        Periodically send Keep Alive commands
        Does not prevent disconnection when device is left idle
        Not sure if this is needed.
        '''
        self.logger.log(logging.INFO, 'Keeping alive started')

        while not self.finish_up.is_set():

            # print('K', end='', flush=True)

            if self._client is not None and self.connected.is_set() and self.sensorStarted.is_set() and (not self.lost_connection.is_set()):
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                await self._client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                self.logger.log(logging.DEBUG,'Keep alive sent')        
                sleepTime=KEEPALIVEINTERVAL
            else:
                if self.sensorStarted.is_set(): self.logger.log(logging.DEBUG,'Cound not send Keep alive')
                # do not report keep alive issues if sensor is not yet running            
                sleepTime=1
            await asyncio.sleep(sleepTime)
        
        self.logger.log(logging.INFO, 'Keeping alive stopped')

    async def update_report(self, args):
        '''
        Report latest fused data
        '''
        self.logger.log(logging.INFO, 'Starting reporting')
        while not self.finish_up.is_set():

            currentTime = time.perf_counter()

            await self.reportingDataAvailable.wait()
            self.reportingDataAvailable.clear()

            self.report_deltaTime = currentTime - self._previous_reportUpdate
            self._previous_reportUpdate = copy(currentTime)

            self._report_updateCounts += 1
            if (currentTime - self._report_lastTimeFPS)>= 1.:
                self.report_fps = self._report_updateCounts
                self._report_lastTimeFPS = copy(currentTime)
                self._report_updateCounts = 0

            # Display the Data
            msg_out = '-------------------------------------------------\n'
            if args.report > 1:
                msg_out+= 'gearVR Controller: Temp {:>4.1f}, Bat {:>3d}, HighRes:{}\n'.format(
                                                    self.temperature, self.battery_level, 
                                                    'Y' if self.HighResMode else 'N')
            else:
                msg_out+= 'gearVR Controller: Temp {:>4.1f}, Bat {:>3d}\n'.format(
                                                    self.temperature, self.battery_level)
            msg_out+= '-------------------------------------------------\n'

            if args.report > 1:
                msg_out+= 'Data    {:>10.6f}, fps {:>3d}\n'.format(self.data_deltaTime*1000.,    self.data_fps)
                msg_out+= 'Report  {:>10.6f}, fps {:>3d}\n'.format(self.report_deltaTime*1000.,  self.report_fps)
                if args.virtual:
                    msg_out+= 'Virtual {:>10.6f}, fps {:>3d}\n'.format(self.virtual_deltaTime*1000., self.virtual_fps)
                if args.fusion:
                    msg_out+= 'Fusion  {:>10.6f}, fps {:>3d}\n'.format(self.fusion_deltaTime*1000.,  self.fusion_fps)
                if args.serial is not None:
                    msg_out+= 'Serial  {:>10.6f}, fps {:>3d}\n'.format(self.serial_deltaTime*1000.,  self.serial_fps)

                msg_out+= '-------------------------------------------------\n'

            msg_out+= 'Time  {:>10.6f}, {:>10.6f}, {:>10.6f}\n'.format(self.sensorTime, self.aTime, self.bTime)
            msg_out+= 'dt    {:>10.6f}, {:>10.6f}, {:>10.6f}\n'.format(self.delta_sensorTime, self.delta_aTime, self.delta_bTime)
            msg_out+= 'Accel {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.accX,self.accY,self.accZ)
            msg_out+= 'Mag   {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.magX,self.magY,self.magZ)
            msg_out+= 'Gyro  {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.gyrX,self.gyrY,self.gyrZ)

            msg_out+= '-------------------------------------------------\n'
            
            msg_out+= 'Trig {} Touch {} Back {} Home {}, Vol+ {} Vol- {} None: {}\n'.format(
                                                'Y' if self.trigger     else 'N', 
                                                'Y' if self.touch       else 'N', 
                                                'Y' if self.back        else 'N',
                                                'Y' if self.home        else 'N',
                                                'Y' if self.volume_up   else 'N',
                                                'Y' if self.volume_down else 'N',
                                                'Y' if self.noButton    else 'N')

            msg_out+= '-------------------------------------------------\n'

            msg_out+= 'TPad:  {:>3d},{:>3d}\n'.format(self.touchX, self.touchY)

            if args.virtual:
                msg_out+= 'VPad:  {:>3d},{:>3d} U{} D{} L{} R{}\n'.format(
                                                    self.absX, self.absY,
                                                    'Y' if self.dirUp    else 'N',
                                                    'Y' if self.dirDown  else 'N',
                                                    'Y' if self.dirLeft  else 'N',
                                                    'Y' if self.dirRight else 'N')
                
                msg_out+= 'Wheel: {:>3d}:{:>3d} T{} B{} L{} R{} C{} R:{}\n'.format(
                                                    self.wheelPos, self.delta_wheelPos,
                                                    'Y' if self.top    else 'N',
                                                    'Y' if self.bottom else 'N',
                                                    'Y' if self.left   else 'N',
                                                    'Y' if self.right  else 'N',
                                                    'Y' if self.center else 'N', 
                                                    (' C' if self.clockwise else 'CC') if self.isRotating else '__')

                msg_out+= '-------------------------------------------------\n'

            if args.fusion:
                msg_out+= 'Acc     {:>8.3f} {:>8.3f} {:>8.3f} N  {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z,self.acc.norm)
                msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N  {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z,self.mag.norm)
                msg_out+= 'Gyr     {:>8.3f} {:>8.3f} {:>8.3f} N  {:>8.3f}\n'.format(self.gyr.x*RAD2DEG,self.gyr.y*RAD2DEG,self.gyr.z*RAD2DEG,self.gyr.norm*RAD2DEG)
                msg_out+= 'Gyr avg {:>8.5f} {:>8.5f} {:>8.5f} RPM{:>8.3f}\n'.format(self.gyr_average.x*RAD2DEG, self.gyr_average.y*RAD2DEG, self.gyr_average.z*RAD2DEG, self.gyr.norm*60/TWOPI)

                msg_out+= 'Euler: R{:>6.1f} P{:>6.1f} Y{:>6.1f}, Heading {:>4.0f}\n'.format(
                                                self.rpy.x*RAD2DEG, self.rpy.y*RAD2DEG, self.rpy.z*RAD2DEG, 
                                                self.heading*RAD2DEG)
                msg_out+= 'Q:     W{:>6.3f} X{:>6.3f} Y{:>6.3f} Z{:>6.3f}\n'.format(
                                                self.q.w, self.q.x, self.q.y, self.q.z)

            print(msg_out, flush=True)

            # Wait to next interval time
            sleepTime = self._report_updateInterval - (time.perf_counter() - currentTime)
            if sleepTime > 0:
                await asyncio.sleep(sleepTime)

        self.logger.log(logging.INFO, 'Reporting stopped')

    async def update_serial(self, reader, writer):
        '''
        Report latest fused data
        This is formatted to respond to freeIMU calibration GUI software
        '''
        self.logger.log(logging.INFO, 'Creating serial reader and writer with {} at {} baud...'.format(args.serial, args.baud))
        reader, writer = await serial_asyncio.open_serial_connection(url=args.serial, baudrate=args.baud)

        # writer.write('Welcome to gearVRC\r\n'.encode())

        while not self.finish_up.is_set():

            msg_in = await reader.readline() # read a full line, needs to have /n at end of line (Ctrl-J in putty)
            line_in = msg_in.decode().strip()
            if len(line_in) > 0:
                if 'v' in line_in: # version number request
                    writer.write("gearVRC v1.0.0\r\n".encode())

                elif 'x' in line_in:
                    self.logger.log(logging.INFO,'We dont have EEPROM')

                elif 'c' in line_in:
                    self.logger.log(logging.INFO,'We dont have EEPROM')

                elif 'C' in line_in:
                    self.logger.log(logging.INFO,'We dont have EEPROM')
                    writer.write('N.A.\r\n')
                    writer.write('N.A.\r\n')
                    writer.write('N.A.\r\n')
                    writer.write('N.A.\r\n')
                else:
                    # check if b command was present
                    # extract number of readings requested
                    match_b = re.search(r'b(\d+)', line_in)
                    if match_b:
                        count = int(match_b.group(1))
                        
                        startTime = time.perf_counter()
                        self._previous_serialUpdate = copy(startTime)
                                                
                        for i in range(count):
                            currentTime = time.perf_counter()

                            await self.dataAvailable.wait()
                            self.dataAvailable.clear()

                            self.serial_deltaTime = currentTime - self._previous_serialUpdate
                            self._previous_serialUpdate = copy(currentTime)

                            self._serial_updateCounts += 1
                        
                            accX_hex=float_to_hex(self.acc.x)
                            accY_hex=float_to_hex(self.acc.y)
                            accZ_hex=float_to_hex(self.acc.z)

                            gyrX_hex=float_to_hex(self.gyr.x)
                            gyrY_hex=float_to_hex(self.gyr.y)
                            gyrZ_hex=float_to_hex(self.gyr.z)

                            magX_hex=float_to_hex(self.mag.x)
                            magY_hex=float_to_hex(self.mag.y)
                            magZ_hex=float_to_hex(self.mag.z)

                            # acc,gyr,mag
                            line_out = accX_hex+accY_hex+accZ_hex+gyrX_hex+gyrY_hex+gyrZ_hex+magX_hex+magY_hex+magZ_hex+'\r\n'
                            msg_out=line_out.encode()
                            writer.write(msg_out)
                    
                        self.serial_fps = int(self._serial_updateCounts / (time.perf_counter() - startTime))
                        self._serial_updateCounts = 0
                    else:
                        pass # unknown command
            else:
                pass # empty line received
                
        self.logger.log(logging.INFO, 'Serial stopped')
    
    async def update_zmq(self, args):
        '''
        Report data on ZMQ socket
        if virtual is enabled, also send virtual data
        if fusion is enabled, also send fusion data
        '''
        
        self.logger.log(logging.INFO, 'Creating zmq publisher at tcp://*:{} ...'.format(args.zmq))
        socket = zmq.asyncio.Context().socket(zmq.PUB)
        socket.bind("tcp://*:{}".format(args.zmq))

        data_system  = gearSystemData()
        data_raw     = gearRawData()
        data_virtual = gearVirtualData()
        data_fusion  = gearFusionData()

        while not self.finish_up.is_set():

            await self.dataAvailable.wait()
            self.dataAvailable.clear()
            
            data_raw.accX = self.accX
            data_raw.accY = self.accY
            data_raw.accZ = self.accZ
            data_raw.gyrX = self.gyrX
            data_raw.gyrY = self.gyrY 
            data_raw.gyrZ = self.gyrZ
            data_raw.magX = self.magX
            data_raw.magY = self.magY
            data_raw.magZ = self.magZ
            data_raw.temp = self.temperature
            data_raw.battery = self.battery_level
            data_raw.trigger = self.trigger
            data_raw.touch = self.touch
            data_raw.back = self.back
            data_raw.home = self.home
            data_raw.volume_up = self.volume_up
            data_raw.volume_down = self.volume_down
            data_raw.noButton = self.noButton
            data_raw.touchX = self.touchX
            data_raw.touchY = self.touchY
            data_raw.sensorTime = self.sensorTime
            data_raw.aTime = self.aTime
            data_raw.bTime = self.bTime

            raw_msgpack = msgpack.packb(data_raw)
            socket.send_multipart([b"raw", raw_msgpack])               

            data_system.data_fps    = self.data_fps
            data_system.virtual_fps = self.virtual_fps
            data_system.fusion_fps  = self.fusion_fps
            data_system.zmq_fps     = self.zmq_fps
            data_system.serial_fps  = self.serial_fps
            data_system.reporting_fps = self.report_fps
            
            system_msgpack = msgpack.packb(data_system)
            socket.send_multipart([b"system", system_msgpack])               
            
            if args.virtual:
                await self.virtualDataAvailable.wait()
                self.virtualDataAvailable.clear()
                
                data_virtual.absX = self.absX
                data_virtual.absY = self.absY
                data_virtual.dirUp = self.dirUp
                data_virtual.dirDown = self.dirDown
                data_virtual.dirLeft = self.dirLeft
                data_virtual.dirRight = self.dirRight
                data_virtual.wheelPos = self.wheelPos
                data_virtual.top = self.top
                data_virtual.bottom = self.bottom
                data_virtual.left = self.left
                data_virtual.right = self.right
                data_virtual.center = self.center
                data_virtual.isRotating = self.isRotating
                data_virtual.clockwise = self.clockwise
        
                virtual_msgpack = msgpack.packb(data_virtual)               
                socket.send_multipart([b"virtual", virtual_msgpack])               

            if args.fusion:
                await self.fusedDataAvailable.wait()
                self.fusedDataAvailable.clear()

                data_fusion.acc = self.acc
                data_fusion.gyr = self.gyr
                data_fusion.mag = self.mag
                data_fusion.rpy = self.rpy
                data_fusion.heading = self.heading
                data_fusion.q = self.q
                
                fusion_msgpack = msgpack.packb(data_fusion)
                socket.send_multipart([b"fusion", fusion_msgpack])               
                
        self.logger.log(logging.INFO, 'ZMQ stopped')
            
    async def handle_termination(self, tasks:None):
        self.logger.log(logging.INFO, 'Controller ESC, Control-C or kill signal detected')
        self.finish_up.set()
        self.logger.log(logging.INFO, 'Stopping Sensor...')
        await self.stop_sensor()
        self.logger.log(logging.INFO, 'Disconnecting...')
        await self.disconnect()
        self.logger.log(logging.INFO, 'Cancelling all tasks...')
        if tasks is not None: # This will terminate tasks faster
            await asyncio.sleep(1) # give some time for tasks to finish up
            for task in tasks:
                task.cancel()

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'gearVR Controller Starting...')

    # gearVRC Controller
    controller = gearVRC(device_name=args.name, device_address=args.address, logger=logger, VRMode=args.vrmode)

    connection_task = asyncio.create_task(controller.update_connect())      # remain connected, will not terminate
    keepalive_task  = asyncio.create_task(controller.keep_alive())          # keep sensor alive, will not terminate
    escape_task     = asyncio.create_task(controller.update_EscSequence(counts=3, timeout=1.0)) # User can press Home 3 times to exit
    tasks = [connection_task, keepalive_task, escape_task]

    if args.virtual:
        virtual_task    = asyncio.create_task(controller.update_virtual())  # update wheel, will not terminate
        tasks.append(virtual_task)

    if args.fuse:
        fusion_task     = asyncio.create_task(controller.update_fusion())   # update pose, will not terminate
        tasks.append(fusion_task)

    if args.report > 0:
        reporting_task  = asyncio.create_task(controller.update_report(args))   # report new data, will not terminate
        tasks.append(reporting_task)

    if args.serial is not None:
        serial_task     = asyncio.create_task(controller.update_serial(args))   # update serial, will not terminate
        tasks.append(serial_task)

    if args.zmq is not None:
        zmq_task     = asyncio.create_task(controller.update_zmq(args))   # update zmq, will not terminate
        tasks.append(zmq_task)
 
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

    device_group = parser.add_mutually_exclusive_group(required=False)

    device_group.add_argument(
        '-n',
        '--name',
        metavar='<name>',
        dest = 'name',
        type = str,
        help='the name of the bluetooth device to connect to',
        default = DEVICE_NAME
    )

    device_group.add_argument(
        '-a',
        '--address',
        dest = 'address',
        type = str,
        metavar='<address>',
        help='the mac address of the bluetooth device to connect to',
        default = DEVICE_MAC
    )

    parser.add_argument(
        '-vr',
        '--vrmode',
        action='store_true',
        help='sets the vrmode, sensor mode is used otherwise, [does not work]',
        default = False
    )

    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level to debug',
        default = False
    )

    parser.add_argument(
        '-r',
        '--report',
        dest = 'report',
        type = int,
        metavar='<report>',
        help='report level: 0(None), 1(Minimal), 2(Verbose)',
        default = 0
    )

    parser.add_argument(
        '-f',
        '--fuse',
        action='store_true',
        help='turns on IMU data fusion',
        default = False
    )

    parser.add_argument(
        '-v',
        '--virtual',
        action='store_true',
        help='turns on virtual wheel and touchpad',
        default = False
    )

    parser.add_argument(
        '-s',
        '--serial',
        dest = 'serial',
        type = str,
        metavar='<serial>',
        help='serial port for reporting, e.g \'/tmp/ttyV0\' when you are using virtual ports \'socat -d -d pty,rawer,echo=0,link=/tmp/ttyV0 pty,rawer,echo=0,link=/tmp/ttyV1&\'',
        default = None
    )

    parser.add_argument(
        '-b',
        '--baud',
        dest = 'baud',
        type = int,
        metavar='<baud>',
        help='serial baud rate, e.g. 115200',
        default = BAUDRATE
    )

    parser.add_argument(
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = int,
        metavar='<zmqport>',
        help='port use by ZMQ, e.g. 5556',
        default = None
    )

    args = parser.parse_args()
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        # format='%(asctime)-15s %(name)-8s %(levelname)s: %(message)s'
    )   
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass

# python gearVRC_C.py -n 'Gear VR Controller(17DB)'