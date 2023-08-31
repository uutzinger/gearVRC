#!/usr/bin/python3

################################################################
# Samsung gearVR Controller
################################################################
# This work is based on:
# Gear VRC reverse engineering (Java): 
#   https://github.com/jsyang/gearvr-controller-webbluetooth
# Gear VRC to UDEV (Python): 
#   https://github.com/rdady/gear-vr-controller-linux
################################################################
# Prerequisite:
# A) python packages:
#   $ sudo pip3 install 
#           bleak, uvloop, zmq asyncio pyserial-asyncio
#           msgpack,re, pathlib, json, numpy
# B Linux) Pair Controller (check README):
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
import os
from   copy import copy
import serial_asyncio
import re
import msgpack
import pathlib
import json

from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

from pyIMU.madgwick import Madgwick
from pyIMU.quaternion import Vector3D, Quaternion
from pyIMU.utilities import q2rpy, rpymag2h, qmag2h
from pyIMU.motion import Motion

import zmq
import zmq.asyncio


# Activate uvloop to speed up asyncio
if os.name == 'nt':
    from asyncio.windows_events import WindowsSelectorEventLoopPolicy
    asyncio.set_event_loop_policy(WindowsSelectorEventLoopPolicy())
else:
    import uvloop
    asyncio.set_event_loop_policy(uvloop.EventLoopPolicy())

# to run bluetoothctl
if os.name == 'posix':  import subprocess

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
TWOPI   = 2.0*math.pi

BAUDRATE = 115200
ZMQPORT  = 5556
ZMQTIMEOUT = 1000 # ms

# Device name:
################################################################
DEVICE_NAME = 'Gear VR Controller(17DB)'
DEVICE_MAC  = '2C:BA:BA:2E:17:DB'
BLETIMEOUT = 10.0

# LOCATION
################################################################
DECLINATION  = 8.124973426113137 * DEG2RAD      # Declination at your location
LATITUDE     = 32.253460           # [degrees] Tucson
LONGITUDE    = -110.911789         # [degrees] Tucson
ALTITUDE     = 730                 # [m], Tucson
MAGFIELD     = 33078.93064435485   # [nano Tesla] Tucson
MAGFIELD_MAX = 1.2*MAGFIELD/1000.  # micro Tesla
MAGFIELD_MIN = 0.8*MAGFIELD/1000.  # micro Tesla

# Program Timing:
################################################################
KEEPALIVEINTERVAL                = 60.    # Every 60 secs send command to keep alive
VIRTUALUPDATEINTERVAL            = 1./20. # 20 Hz
REPORTINTERVAL                   = 1./10. # 10 Hz

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
WHEELDIAMETER                    = 315                      # Touch pad diameter in pixels
WHEELTICKNESS                    = 25                       # Virtual wheel rim thickness in pixels
WHEELRADIUS                      = WHEELDIAMETER / 2.       # Radius of touch pad
WHEELRADIUS2                     = WHEELRADIUS**2
RTHRESH2                         = (WHEELRADIUS - WHEELTICKNESS)**2
NUMWHEELPOS                      = 64                       # Number of simulated rotating wheel positions
NUMWHEELPOS1_8                   = NUMWHEELPOS*1/8
NUMWHEELPOS3_8                   = NUMWHEELPOS*3/8
NUMWHEELPOS5_8                   = NUMWHEELPOS*5/8
NUMWHEELPOS7_8                   = NUMWHEELPOS*7/8
MAXWHEELPOS                      = NUMWHEELPOS-1
MAXWHEELPOS2                     = MAXWHEELPOS/2.0
# Scrolling on virtual touchpad
#  We can extend the range of the touchpad by scrolling: 
#     lifting the finger and moving in same direction again
MINXTOUCH                        = 0                       # min virtual touchpad X
MINYTOUCH                        = 0
MAXXTOUCH                        = 1024                    # max virtual touchpad X
MAXYTOUCH                        = 1024

# Motion Detection
# Adjust as it depends on sensor
################################################################
FUZZY_ACCEL_ZERO_MAX    = 10.0  # threshold for acceleration activity
FUZZY_ACCEL_ZERO_MIN    = 9.5   # threshold for acceleration activity
FUZZY_DELTA_ACCEL_ZERO  = 0.04  # threshold for acceleration change
FUZZY_GYRO_ZERO         = 0.08  # threshold for gyroscope activity
FUZZY_DELTA_GYRO_ZERO   = 0.003 # threshold for gyroscope change

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

def calibrate(data:Vector3D, offset:Vector3D, crosscorr=None, diagonal=False):
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
    d = copy(data)   # we do not want input parameter to change globally
    d = d-offset
    if diagonal:
        if crosscorr is not None:
            d=Vector3D(d.x*crosscorr[0,0], d.y*crosscorr[1,1], d.z*crosscorr[2,2])
    else:
        # Cross Correlation
        if crosscorr is not None:
            d = d.rotate(crosscorr)

    return d

def loadCalibration(filename):
    '''
    Load Calibration file
    Data is stored in json format
    offset is the bias
    crosscorr is the cross correlation matrix
    diagonal of crosscorr is the scale
    '''
    with open(filename, 'r') as file:
        d = json.load(file)

    center = np.array([d['offset_x'],d['offset_y'],d['offset_z']])
    correctionMat = np.empty([3,3])

    correctionMat[0,0] = d['cMat_00']
    correctionMat[0,1] = d['cMat_01']
    correctionMat[0,2] = d['cMat_02']
    correctionMat[1,0] = d['cMat_10']
    correctionMat[1,1] = d['cMat_11']
    correctionMat[1,2] = d['cMat_12']
    correctionMat[2,0] = d['cMat_20']
    correctionMat[2,1] = d['cMat_21']
    correctionMat[2,2] = d['cMat_22']

    return center, correctionMat

def saveCalibration(filename, center, correctionMat):
    '''Save Calibration Data to JSON file'''
    d = {
    "offset_x": center[0], 
    "offset_y": center[1], 
    "offset_z": center[2], 
    "cMat_00":  correctionMat[0,0],
    "cMat_01":  correctionMat[0,1],
    "cMat_02":  correctionMat[0,2],
    "cMat_10":  correctionMat[1,0],
    "cMat_11":  correctionMat[1,1],
    "cMat_12":  correctionMat[1,2],
    "cMat_20":  correctionMat[2,0],
    "cMat_21":  correctionMat[2,1],
    "cMat_22":  correctionMat[2,2]
    }

    with open(filename, 'w') as file:
        json.dump(d, file)

def detectMotion(acc: float, gyr: float, acc_avg: float, gyr_avg:float) -> bool:
    '''
    # 4 Stage Motion Detection
    # ------------------------
    1. Acceleration Activity
    2. Change in Acceleration
    3. Gyration Activity
    4. Change in Gyration
    
    Original Code is from FreeIMU Processing example with some modifications and tuning
    '''

    # ACCELEROMETER
    # Absolute value
    acc_test       = abs(acc) > FUZZY_ACCEL_ZERO_MAX and abs(acc) < FUZZY_ACCEL_ZERO_MIN
    # Sudden changes
    acc_delta_test = abs(acc_avg - acc) > FUZZY_DELTA_ACCEL_ZERO

    # GYROSCOPE
    # Absolute value
    gyr_test       = abs(gyr)           > FUZZY_GYRO_ZERO
    # Sudden changes
    gyr_delta_test = abs(gyr_avg - gyr) > FUZZY_DELTA_GYRO_ZERO
    
    # DEBUG for fine tuning
    # print(abs(acc), abs(acc_avg-acc), abs(gyr), abs(gyr_avg-gyr), acc_test, acc_delta_test, gyr_test, gyr_delta_test)

    # Combine acceleration test, acceleration deviation test and gyro test
    return (acc_test or acc_delta_test or gyr_test or gyr_delta_test)
	
def check_bluetooth_connected(address):
    '''
    Check if bluetooth device is already connected
    Only on posix
    '''
    if os.name == 'posix':
        try:
            output = subprocess.check_output(["bluetoothctl", "info", address], timeout=5, text=True)
            lines  = output.strip().split("\n")
            for line in lines:
                if "Connected: yes" in line:
                    return True
                elif "Connected: no" in line:
                    return False
            return False
        
        except:
            return False
    else:
        return False

def disconnect_bluetooth_device(address):
    '''
    Disconnects bluetooth device
    Only on posix
    '''
    if os.name == 'posix':
        try:
            output = subprocess.check_output(["bluetoothctl", "disconnect", address], timeout=5, text=True)
            return  True        
        except:
            return False
    else:
        return False

def obj2dict(obj):
    '''
    encoding object variables to nested dictionary
    ''' 
    if isinstance(obj, dict):
        return {k: obj2dict(v) for k, v in obj.items()}
    elif hasattr(obj, '__dict__'):
        return obj2dict(vars(obj))
    elif isinstance(obj, list):
        return [obj2dict(item) for item in obj]
    else:
        return obj

class dict2obj:
    '''
    decoding nested dictionary to object
    '''
    def __init__(self, data):
        for key, value in data.items():
            if isinstance(value, dict):
                setattr(self, key, dict2obj(value))
            else:
                setattr(self, key, value)

def float_to_hex(f):
    '''
    Pack float into 8 characters 0..9,A..F
    '''
    bytes_ = struct.pack('!f', f)                            # Single precision, big-endian byte order, 4 bytes
    hex_strings = [format(byte, '02X') for byte in bytes_]   # Convert each byte to a hexadecimal string
    return ''.join(hex_strings)

def hex_to_float(hex_chars):
    '''
    Unpack 8 characters to float
    '''
    hex_bytes = bytes.fromhex(hex_chars)  # Convert hex characters to bytes
    return struct.unpack('!f', hex_bytes)[0]     

def int_to_hex(n):
    '''
    Pack integer to 8 characters
    '''
    bytes_ = n.to_bytes((n.bit_length() + 7) // 8, 'big')  # Convert integer to bytes
    hex_strings = [format(byte, '02X') for byte in bytes_]           # Convert each byte to a hexadecimal string
    return ''.join(hex_strings)

def hex_to_int(hex_chars):
    '''
    Unpack 8 characters to integer
    '''
    hex_bytes = bytes.fromhex(hex_chars)  # Convert hex characters to bytes
    return struct.unpack('!i', hex_bytes)[0]

###################################################################
# Data Classes
###################################################################

class gearSystemData(object):
    '''System relevant performance data'''
    def __init__(self, temperature: float=0.0, battery_level: float =-1.0, 
                 data_rate: int = 0, virtual_rate: int = 0, fusion_rate:    int = 0, 
                 zmq_rate:  int = 0, serial_rate:  int = 0, reporting_rate: int = 0,
                 fusion:  bool=False, 
                 motion:  bool=False,
                 virtual: bool=False,
                 serial:  bool=False) -> None:
        self.temperature     = temperature
        self.battery_level   = battery_level
        self.data_rate       = data_rate
        self.virtual_rate    = virtual_rate
        self.fusion_rate     = fusion_rate
        self.zmq_rate        = zmq_rate
        self.serial_rate     = serial_rate
        self.reporting_rate  = reporting_rate
        self.fusion          = fusion
        self.motion          = motion
        self.virtual         = virtual
        self.serial          = serial

class gearIMUData(object):
    '''IMU Data from the sensor'''
    def __init__(self, 
                 time: float=0.0,
                 acc: Vector3D = Vector3D(0.,0.,0.),
                 gyr: Vector3D = Vector3D(0.,0.,0.),
                 mag: Vector3D = Vector3D(0.,0.,0.),
                 moving: bool  = True,
                 magok: bool   = False) -> None:
        self.time   = time
        self.acc    = acc
        self.mag    = mag
        self.gyr    = gyr
        self.moving = moving
        self.magok  = magok

class gearButtonData(object):
    '''Button Data from the sensor'''
    def __init__(self, 
                 time: float=0.0,
                 trigger: bool = False, touch: bool = False, back: bool = False, home: bool = False, 
                 volume_up: bool = False, volume_down: bool = False, noButton: bool = True,
                 touchX: int = 0, touchY: int = 0) -> None:
        self.time           = time
        self.trigger        = trigger
        self.touch          = touch
        self.back           = back
        self.home           = home
        self.volume_up      = volume_up
        self.volume_down    = volume_down
        self.noButton       = noButton
        self.touchX         = touchX
        self.touchY         = touchY

class gearTouchData(object):
    '''Touch Data from the sensor'''
    def __init__(self, 
                 time: float=0.0,
                 touchX: int = 0, touchY: int = 0) -> None:
        self.time           = time
        self.touchX         = touchX
        self.touchY         = touchY

class gearVirtualData(object):
    '''Virtual wheel and touchpad data'''
    def __init__(self, time: float = 0.,
                 absX: int = 0, absY: int = 0,
                 dirUp: bool = False, dirDown: bool = False, dirLeft: bool = False, dirRight: bool = False,
                 wheelPos: int = 0,
                 top: bool = False, bottom: bool = False, left: bool = False, right: bool = False, center: bool = False,
                 clockwise: bool = False, isRotating: bool = False) -> None:
        self.time           = time
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
                 time: float   = 0.,
                 acc: Vector3D = Vector3D(0.,0.,0.), 
                 mag: Vector3D = Vector3D(0.,0.,0.), 
                 gyr: Vector3D = Vector3D(0.,0.,0.), 
                 rpy: Vector3D = Vector3D(0.,0.,0.), 
                 heading: float = 0.0, 
                 q: Quaternion = Quaternion(1.,0.,0.,0.)) -> None:
        self.time           = time
        self.acc            = acc
        self.mag            = mag
        self.gyr            = gyr
        self.rpy            = rpy
        self.heading        = heading
        self.q              = q

class gearMotionData(object):
    '''Motion data'''
    def __init__(self, 
                 time: float   = 0.,
                 residuals:    Vector3D = Vector3D(0.,0.,0.), 
                 velocity:     Vector3D = Vector3D(0.,0.,0.), 
                 position:     Vector3D = Vector3D(0.,0.,0.), 
                 accBias:      Vector3D = Vector3D(0.,0.,0.), 
                 velocityBias: Vector3D = Vector3D(0.,0.,0.), 
                 dtmotion:     float = 0.0) -> None:
            self.time         = time
            self.residuals    = residuals
            self.velocity     = velocity
            self.position     = position
            self.dtmotion     = dtmotion
            self.accBias      = accBias
            self.velocityBias = velocityBias

##############################################################################
# ZMQWorker
###############################################################################

class zmqWorkerGear():
    '''
    Receiving data from the GearVR Controller via ZMQ
    Primarily useful for third party clients
    '''

    def __init__(self, logger, zmqPort='tcp://localhost:5556'):

        # Signals
        self.dataReady  = asyncio.Event()
        self.finished   = asyncio.Event()
        self.dataReady.clear()
        self.finished.clear()

        self.logger     = logger
        self.zmqPort    = zmqPort

        self.new_fusion  = False
        self.new_imu     = False
        self.new_system  = False
        self.new_virtual = False
        self.new_button  = False
        self.new_motion  = False
        self.timeout     = False

        self.finish_up   = False
        self.paused      = False
        self.zmqTimeout  = ZMQTIMEOUT

        self.logger.log(logging.INFO, 'gearVRC zmqWorker initialized')

    async def start(self, stop_event: asyncio.Event):

        self.new_system  = False
        self.new_imu     = False
        self.new_virtual = False
        self.new_fusion  = False
        self.new_button  = False
        self.new_motion  = False

        context = zmq.asyncio.Context()
        poller  = zmq.asyncio.Poller()
        
        self.data_system  = None
        self.data_imu     = None
        self.data_motion  = None
        self.data_virtual = None
        self.data_button  = None
        self.data_fusion  = None

        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"system")
        socket.setsockopt(zmq.SUBSCRIBE, b"imu")
        socket.setsockopt(zmq.SUBSCRIBE, b"virtual")
        socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
        socket.setsockopt(zmq.SUBSCRIBE, b"button")
        socket.setsockopt(zmq.SUBSCRIBE, b"motion")
        socket.connect(self.zmqPort)
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'gearVRC zmqWorker started on {}'.format(self.zmqPort))

        while not self.finish_up:
            try:
                events = dict(await poller.poll(timeout=self.zmqTimeout))
                if socket in events and events[socket] == zmq.POLLIN:
                    response = await socket.recv_multipart()
                    if len(response) == 2:
                        [topic, msg_packed] = response
                        if topic == b"system":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_system = dict2obj(msg_dict)
                            self.new_system = True
                        elif topic == b"imu":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_imu = dict2obj(msg_dict)
                            self.new_imu = True                            
                        elif topic == b"virtual":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_virtual = dict2obj(msg_dict)
                            self.new_virtual = True
                        elif topic == b"fusion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_fusion = dict2obj(msg_dict)
                            self.new_fusion = True
                        elif topic == b"button":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_button = dict2obj(msg_dict)
                            self.new_button = True
                        elif topic == b"motion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            self.data_motion = dict2obj(msg_dict)
                            self.new_motion = True
                        else:
                            pass  # not a topic we need
                    else:
                        self.logger.log(
                            logging.ERROR, 'gearVRC zmqWorker malformed message')
                else:  # ZMQ TIMEOUT
                    self.logger.log(logging.ERROR, 'gearVRC zmqWorker timed out')
                    poller.unregister(socket)
                    socket.close()
                    socket = context.socket(zmq.SUB)
                    socket.setsockopt(zmq.SUBSCRIBE, b"system")
                    socket.setsockopt(zmq.SUBSCRIBE, b"imu")
                    socket.setsockopt(zmq.SUBSCRIBE, b"virtual")
                    socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                    socket.setsockopt(zmq.SUBSCRIBE, b"button")
                    socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                    socket.connect(self.zmqPort)
                    poller.register(socket, zmq.POLLIN)
                    self.new_system = \
                    self.new_imu    = \
                    self.new_virtual= \
                    self.new_fusion = \
                    self.new_button = \
                    self.new_motion = False

                if (self.new_imu and self.new_system):
                    if self.data_system.fusion:
                        if self.new_fusion:
                            if self.data_system.motion:
                                if self.new_motion:
                                    if self.data_system.virtual:
                                        if self.new_virtual:
                                            if not self.paused: self.dataReady.set()
                                            self.new_system  = \
                                            self.new_button  = \
                                            self.new_imu     = \
                                            self.new_fusion  = \
                                            self.new_virtual = \
                                            self.new_motion  = False
                                        else:
                                            pass # just wait until we have virtual
                                    else:
                                        if not self.paused: self.dataReady.set()
                                        self.new_system  = \
                                        self.new_button  = \
                                        self.new_imu     = \
                                        self.new_fusion  = \
                                        self.new_motion  = False
                                else:
                                    pass # wait for motion
                            else:
                                if not self.paused: self.dataReady.set()
                                self.new_system  = \
                                self.new_button  = \
                                self.new_imu     = \
                                self.new_fusion  = False
                        else:
                            pass # just wait until we have fusion data
                    else:
                        if not self.paused: self.dataReady.set()
                        self.new_system  = \
                        self.new_button  = \
                        self.new_imu     = False
                else:
                    pass # we need imu and system data, lets wiat for both
                    
            except:
                self.logger.log(logging.ERROR, 'gearVRC zmqWorker error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.SUB)
                socket.setsockopt(zmq.SUBSCRIBE, b"system")
                socket.setsockopt(zmq.SUBSCRIBE, b"imu")
                socket.setsockopt(zmq.SUBSCRIBE, b"virtual")
                socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                socket.setsockopt(zmq.SUBSCRIBE, b"button")
                socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                socket.connect(self.zmqPort)
                poller.register(socket, zmq.POLLIN)
                self.new_system = \
                self.new_imu    = \
                self.new_virtual= \
                self.new_fusion = \
                self.new_button = \
                self.new_motion = False

        self.logger.log(logging.DEBUG, 'gearVRC zmqWorker finished')
        socket.close()
        context.term()
        self.finished.set()

    def set_zmqPort(self, port):
        self.zmqPort = port

    def pause(self):
        self.paused = not self.paused

#########################################################################################################
# gearVRC 
#########################################################################################################

class gearVRC:
            
    def __init__(self, args, logger) -> None:
        
        self.args                               = args

        # Shared States
        self.fusion                             = args.fusion
        self.motion                             = args.motion
        self.report                             = args.report

        # ZMQ
        self.zmqPortPUB                         = args.zmqPortPUB
        self.zmqPortREP                         = args.zmqPortREP

        # Serial
        self.serialport                         = args.serial
        self.baudrate                           = args.baud
        if self.serialport is not None: self.serial = True
        else:                           self.serial = False
        
        # Bluetooth device description
        self.device_name                        = args.name
        self.device_address                     = args.address
  
        # Bluetooth device and client
        self.device                             = None
        self.client                             = None

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
        self.battery_service                    = None
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

        # Controller home button to stop program
        self.ESC_reps                   = args.escreps
        self.ESC_timeout                = args.esctimeout
        
        # Signals
        self.lost_connection            = asyncio.Event()
        self.processedDataAvailable     = asyncio.Event()
        self.terminate                  = asyncio.Event()

        # These Signals are easier to deal with without Event
        self.connected                  = False
        self.sensorStarted              = False
        self.finish_up                  = False
        self.paused                     = False
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger('gearVRC')

        self.VRMode                     = args.vrmode
        self.HighResMode                = False

        # device sensors
        self.sensorTime                 = 0.    # There are 3 different times transmitted from the sensor
        # self.aTime                    = 0.    # Assuming the earliest time corresponds to IMU
        # self.bTime                    = 0.    #
        self.delta_sensorTime           = 0.
        # self.delta_aTime              = 0.
        # self.delta_bTime              = 0.
        self.previous_sensorTime        = 0.
        # self.previous_aTime           = 0.
        # self.previous_bTime           = 0.

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
        self.battery_level          = 0     # Device Battery level

        # Device buttons 
        self.touch                  = False # Touchpad has been pressed
        self.trigger                = False # Trigger button pressed
        self.home                   = False # Home button pressed
        self.back                   = False # Back button pressed
        self.volume_up              = False # Volume up button pressed
        self.volume_down            = False # Volume down button pressed
        self.noButton               = True  # No button was pressed (mutually exclusive with the above)

        # ESC sequence
        self.previous_home          = False
        self.home_updateCounts      = 0
        self.home_pressedTime       = time.perf_counter()

        # Virtual wheel
        self.wheelPos               = 0     # Wheel position 0..63
        self.previous_wheelPos      = 0
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
        self.previous_touchX        = 0     # touch pad X
        self.previous_touchY        = 0     # touch pad Y
        self.absX                   = 0     # Position on virtual touchpad (scrolling)
        self.absY                   = 0     #
        self.dirUp                  = False # Touching pad and moving upwards
        self.dirDown                = False # Touching pad and moving downwards
        self.dirLeft                = False # Touching pad and moving to the left
        self.dirRight               = False # Touching pad and moving to the right
                        
        # Timing
        ###################
        self.startTime              = 0.
        self.runTime                = 0.
    
        self.data_deltaTime         = 0.
        self.data_rate              = 0
        self.data_updateCounts      = 0
        self.data_lastTime          = time.perf_counter()
        self.data_lastTimeRate      = time.perf_counter()

        self.virtual_deltaTime      = 0.
        self.virtual_rate           = 0
        self.virtual_updateCounts   = 0
        self.virtual_updateInterval = VIRTUALUPDATEINTERVAL
        self.virtual_lastTimeRate   = time.perf_counter()
        self.previous_virtualUpdate = time.perf_counter()

        self.fusion_deltaTime       = 0.
        self.fusion_rate            = 0
        self.fusion_updateCounts    = 0
        self.fusion_lastTimeRate    = time.perf_counter()
        self.previous_fusionTime    = time.perf_counter()

        self.motion_deltaTime       = 0.
        self.motion_rate            = 0
        self.motion_updateCounts    = 0
        self.motion_lastTimeRate    = time.perf_counter()
        self.previous_motionTime    = time.perf_counter()
        self.firstTimeMotion        = time.perf_counter()

        self.report_deltaTime       = 0.
        self.report_rate            = 0
        self.report_updateInterval  = REPORTINTERVAL
        self.report_updateCounts    = 0
        self.report_lastTimeRate    = time.perf_counter()

        self.serial_rate            = 0
        self.serial_deltaTime       = 0.
        self.serial_updateCounts    = 0
        
        self.zmqPUB_rate            = 0
        self.zmqPUB_deltaTime       = 0.
        self.zmqPUB_updateCounts    = 0
        self.zmqPUB_lastTimeRate    = time.perf_counter()
        
        self.current_directory = str(pathlib.Path(__file__).parent.absolute())

        # Read Calibration Data
        my_file = pathlib.Path(self.current_directory + '/Gyr.json')
        if my_file.is_file():
            self.logger.log(logging.INFO,'Loading Gyroscope Calibration from File...')
            gyr_offset, gyr_crosscorr = loadCalibration(my_file)
        else:
            self.logger.log(logging.INFO,'Loading default Gyroscope Calibration...')
            gyr_crosscorr  = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
            gyr_offset     = np.array(([-0.01335,-0.01048,0.03801]))

        my_file = pathlib.Path(self.current_directory + '/Acc.json')
        if my_file.is_file():
            self.logger.log(logging.INFO,'Loading Accelerometer Calibration from File...')
            acc_offset, acc_crosscorr = loadCalibration(my_file)
        else:
            self.logger.log(logging.INFO,'Loading default Accelerometer Calibration...')
            acc_crosscorr  = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
            acc_offset     = np.array(([0.,0.,0.]))

        my_file = pathlib.Path(self.current_directory + '/Mag.json')
        if my_file.is_file():
            self.logger.log(logging.INFO,'Loading Magnetometer Calibration from File...')
            mag_offset, mag_crosscorr = loadCalibration(my_file)
        else:
            self.logger.log(logging.INFO,'Loading default Magnetometer Calibration...')
            mag_crosscorr  = np.array(([1.,0.,0.], [0.,1.,0.], [0.,0.,1.]))
            mag_offset     = np.array(([-48.492,-222.802,-35.28]))

        self.acc_offset    = Vector3D(acc_offset)
        self.gyr_offset    = Vector3D(gyr_offset)
        self.mag_offset    = Vector3D(mag_offset)
        
        self.acc_crosscorr = acc_crosscorr
        self.gyr_crosscorr = gyr_crosscorr
        self.mag_crosscorr = mag_crosscorr

        self.gyr_offset_updated = False

        self.acc           = Vector3D(0.,0.,0.)
        self.gyr           = Vector3D(0.,0.,0.)
        self.mag           = Vector3D(0.,0.,0.)
        self.gyr_average   = Vector3D(0.,0.,0.)
        self.acc_average   = Vector3D(0.,0.,0.)

        # Attitude fusion
        self.AHRS          = Madgwick()
        self.q             = Quaternion(1.,0.,0.,0.)
        self.heading       = 0.
        self.rpy           = Vector3D(0.,0.,0.)

        self.acc_cal       = Vector3D(0.,0.,0.)
        self.gyr_cal       = Vector3D(0.,0.,0.)
        self.mag_cal       = Vector3D(0.,0.,0.)

        self.moving        = True
        self.magok         = False

        # Motion
        self.Position      = Motion()
        self.residuals     = Vector3D(0.,0.,0.)
        self.velocity      = Vector3D(0.,0.,0.)
        self.position      = Vector3D(0.,0.,0.)
        self.dtmotion      = 0.
        self.dt            = 0.
        self.accBias       = Vector3D(0.,0.,0.)
        self.velocityBias  = Vector3D(0.,0.,0.)

        # First Time Getting Data
        self.firstTimeData     = True  # want to initialize average acc,mag,gyr with current reading first time
        self.sensorIsBooting   = True  # need to read sensor a few times until we get reasonable data
        self.sensorRunInCounts = 0     # for boot up
        self.motionIsBooting   = True  # need to establish averages until we run motion

    def update_times(self):
        self.home_pressedTime       = \
        self.data_lastTime          = \
        self.data_lastTimeRate      = \
        self.virtual_lastTimeRate   = \
        self.previous_virtualUpdate = \
        self.fusion_lastTimeRate    = \
        self.previous_fusionTime    = \
        self.report_lastTimeRate    = \
        self.zmqPUB_lastTimeRate       = \
        self.motion_lastTimeRate    = \
        self.previous_motionTime    = \
        self.firstTimeMotion        = time.perf_counter()
    

    def handle_disconnect(self,client):
        self.logger.log(logging.INFO,'Client disconnected, Signaling...')
        self.connected=False
        self.lost_connection.set()

    async def connect_loop(self):
        '''
        An attempt to remain connected with the device.
        
        Connection Loop:
          Scan for Device
            Create Client
                Connect to Device
                Scan for Services & Characteristics
            Wait for Disconnection
            Attempt Reconnection
          Back to Scan for Device            
        '''

        self.logger.log(logging.INFO, 'Starting Connect Task...')

        first_time = True
        sleep_time = 5.0

        while not self.finish_up: # Run for ever

            # print('C', end='', flush=True)
            
            # Make sure device is not already connected
            #   (not available on Windows)
            ###########################################
            if self.device_address is not None:
                if check_bluetooth_connected(self.device_address):
                    self.logger.log(logging.INFO,'Device {} is already connected. Disconnecting...'.format(self.device_name))
                    if disconnect_bluetooth_device(self.device_address):
                        self.connected = False
                        self.logger.log(logging.INFO,'Device {} successfully disconnected'.format(self.device_name))
                else:
                    self.connected = False

            # Scan for Device
            #################
            self.device = None
            # try to find device using its name
            self.logger.log(logging.INFO,'Searching for {}'.format(self.device_name))            
            if self.device_name is not None:
                self.device = await BleakScanner.find_device_by_name(self.device_name, timeout=BLETIMEOUT)
            if self.device is None:
                # find by address because find by name did not work
                if self.device_address is not None:
                    self.device = await BleakScanner.find_device_by_address(self.device_address, timeout=BLETIMEOUT)
    
            # Connect to Device
            ###################
            if self.device is not None:

                self.logger.log(logging.INFO,'Found {}'.format(self.device_name))

                # Create client                
                async with BleakClient(self.device, disconnected_callback=self.handle_disconnect, timeout=BLETIMEOUT) as self.client:
                    # Connect to device
                    if not self.client.is_connected:
                        self.logger.log(logging.INFO,'Connecting to {}'.format(self.device_name))
                        await self.client.connect()
                    else:
                        self.logger.log(logging.INFO,'Already connected to {}'.format(self.device_name))

                    if self.client.is_connected:
                        self.connected = True
                        self.lost_connection.clear()
                        self.logger.log(logging.INFO,'Finding Characteristics')
                        # this needs to be run each time we create a new client
                        self.find_characteristics() # scan and assign device characteristics
                        # This needs to be run only once
                        if first_time:
                            # We dont need to read device information each time, as it remains the same
                            self.logger.log(logging.INFO,'Reading Device Information')
                            await self.read_deviceInformation() # populate device information
                            first_time = False
                        # this needs to be run each time we create a new client
                        self.logger.log(logging.INFO,'Starting Sensor')
                        await self.start_sensor(self.VRMode) # start the sensors
                        self.startTime = time.perf_counter()
                        # this needs to be run each time we create a new client
                        self.logger.log(logging.INFO,'Subscribing to Notifications')
                        await self.subscribe_notifications() # subscribe to device characteristics that have notification function
                        self.logger.log(logging.INFO,'Setup completed')
                    else:
                        self.connected = False

                    if self.connected:
                        # reset timers
                        # self.update_times()
                        
                        # Wait until disconnection occurs
                        await self.lost_connection.wait()

                        # Connection was lost
                        self.sensorIsBooting = True
                        self.sensorRunInCounts = 0
                        
                        if not self.finish_up: 
                            self.logger.log(logging.INFO,'Lost connection to {}'.format(self.device_name))            
                    else:
                        self.logger.log(logging.ERROR,"Could not connect to {}".format(self.device_name))

                # Lost connection or could not connect                        
                sleep_time = 1.0

            else: 
                # Device not found
                ##################
                self.logger.log(logging.INFO,'{} not found. Retrying...'.format(self.device_name))
                sleep_time = 5.0
            
            await asyncio.sleep(sleep_time)

        self.logger.log(logging.INFO, 'Connect Task stopping')

    async def disconnect(self):
        '''
        Disconnect from Sensor
        '''
        self.logger.log(logging.INFO, 'Disconnecting Client...')
        if self.client is not None:
            await self.client.disconnect()
            self.connected=False

 
    def find_characteristics(self):
        '''
        Scan device services and characteristics for the expected values
        '''

        # Generic Access Profile
        ########################
        for s in self.client.services:
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
        for s in self.client.services:
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
        for s in self.client.services:
            if s.uuid == '0000180f-0000-1000-8000-00805f9b34fb':
                self.battery_service = s
        # read & notify
        for c in self.battery_service.characteristics:
            if c.uuid == '00002a19-0000-1000-8000-00805f9b34fb':
                self.battery_level_characteristic = c

        # Controller Sensor Data
        ########################
        for s in self.client.services:
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
          Primarily used for characteristics that dont have a BLE notification.
          This needs to be read only once as most values dont change.
        '''
        
        # Battery Level
        if self.client is not None:
            if self.battery_level_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.battery_level_characteristic)
                self.battery_level = int.from_bytes(value,byteorder='big')
            else:
                self.battery_level = -1
            self.logger.log(logging.INFO,'Battery Level: {}'.format(self.battery_level))                
        else:
            self.logger.log(logging.ERROR, 'Could not read Battery Level: disconnected')
        # Device Name
        if self.client is not None:
            if self.device_name_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.device_name_characteristic)
                self.device_name = value.decode()
            else:
                self.device_name = DEVICE_NAME
            self.logger.log(logging.INFO,'Device Name: {}'.format(self.device_name))                
        else:
            self.logger.log(logging.ERROR, 'Could not read device name: disconnected')
        # Manufacturer
        if self.client is not None:
            if self.manufacturer_name_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.manufacturer_name_characteristic)
                self.manufacturer_name = value.decode()
            else:
                self.manufacturer_name = ''
            self.logger.log(logging.INFO,'Manufacturer Name: {}'.format(self.manufacturer_name))                
        else:
            self.logger.log(logging.ERROR, 'Could not read manufacturer name: disconnected')
        # Model Number
        if self.client is not None:
            if self.model_number_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.model_number_characteristic)
                self.model_number = value.decode()
            else:
                self.model_number = ''
            self.logger.log(logging.INFO,'Model Number: {}'.format(self.model_number))                
        else:
            self.logger.log(logging.ERROR, 'Could not read model number: disconnected')
        # Serial Number
        if self.client is not None:
            if self.serial_number_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.serial_number_characteristic)
                self.serial_number = value.decode()
            else:
                self.serial_number = ''
            self.logger.log(logging.INFO,'Serial Number: {}'.format(self.serial_number))                
        else:
            self.logger.log(logging.ERROR, 'Could not read serial number: disconnected')
        # Hardware Revision
        if self.client is not None:
            if self.hardware_revision_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.hardware_revision_characteristic)
                self.hardware_revision = value.decode()
            else:
                self.serial_number = ''
            self.logger.log(logging.INFO,'Hardware Revision: {}'.format(self.hardware_revision))                
        else:
            self.logger.log(logging.ERROR, 'Could not read hardware revision: disconnected')
        # Firmware Revision
        if self.client is not None:
            if self.firmware_version_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.firmware_version_characteristic)
                self.firmware_revision = int.from_bytes(value,'big')
            else:
                self.firmware_revision = -1
            self.logger.log(logging.INFO,'Firmware Revision: {}'.format(self.firmware_revision))
        else:
            self.logger.log(logging.ERROR, 'Could not read firmware revision: disconnected')
        # Software Revision
        if self.client is not None:
            if self.software_revision_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.software_revision_characteristic)
                self.software_revision = value.decode()
            else:
                self.software_revision = ''
            self.logger.log(logging.INFO,'Software Revision: {}'.format(self.software_revision))
        else:
            self.logger.log(logging.ERROR, 'Could not read software revision: disconnected')
        # PnP ID
        if self.client is not None:
            if self.PnP_ID_characteristic is not None and self.connected:
                value = await self.client.read_gatt_char(self.PnP_ID_characteristic)
                self.pnp_ID = int.from_bytes(value,'big')
            else:
                self.pnp_ID = -1
            self.logger.log(logging.INFO,'PnP ID: {}'.format(self.pnp_ID))                
        else:
            self.logger.log(logging.ERROR, 'Could not read pnp id: disconnected')

    async def subscribe_notifications(self):
        ''' 
        Subscribe to BLE Data and Battery Notifications
        '''        
        if self.client is not None:
            # Sensor data
            if self.controller_data_characteristic is not None:
               # await self.client.start_notify(self.controller_data_characteristic, self.handle_sensorData)
               await self.client.start_notify(self.controller_data_characteristic, self.handle_data)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to data characteristic')
            # Battery data
            if self.battery_level_characteristic is not None:
                await self.client.start_notify(self.battery_level_characteristic,   self.handle_batteryData)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to battery characteristic')
        else:
            self.logger.log(logging.ERROR, 'No Client available for {}, can not subscribe'.format(self.device_name))
                
    async def unsubscribe_notifications(self):
        '''
        Unsubscribe rom BLE Notifications
        '''
        if self.client is not None:
            # unsubscribe from sensor data
            if self.controller_command_characteristics is not None:
                await self.client.stop_notify(self.controller_data_characteristic)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to data chracteristic')
            # unsubscribe from battery data
            if self.battery_level_characteristic is not None:
                await self.client.stop_notify(self.battery_level_characteristic)
            else:
                self.logger.log(logging.ERROR, 'Client does not have access to battery chracteristic')
        else:
            self.logger.log(logging.ERROR, 'No Client available for {}, can not unscubscribe'.format(self.device_name))

    async def start_sensor(self, VRMode):
        '''
        Start the sensors
          There might be option to make update rate consistent
        '''
        if self.connected:

            # Initialize the device, not sure why commands in other apps are sent multiple times
            # I observed:
            #  when program is restarted update rate switches between 45Hz and 72Hz
            #  it would be worth establishing a sequence of commands that will run the sensor always at 72Hz
            if VRMode:
                # This will fail
                self.logger.log(logging.INFO, 'Setting VR Mode')
                await self.client.write_gatt_char(self.controller_command_characteristics,CMD_VR_MODE)
                self.HighResMode = True
            else:
                self.logger.log(logging.INFO, 'Setting Sensor Mode')
                await self.client.write_gatt_char(self.controller_command_characteristics,CMD_SENSOR)
                self.HighResMode = False
            self.sensorStarted=True
        else:
            self.logger.log(logging.ERROR, 'Sensor not connected, can not send start signal')            
        
    async def stop_sensor(self):
        '''
        Stop the sensors
        '''
        self.logger.log(logging.INFO, 'Stopping Sensor...')
        if self.client is not None:
            if self.connected :
                await self.client.write_gatt_char(self.controller_command_characteristics,CMD_OFF)
                await self.client.write_gatt_char(self.controller_command_characteristics,CMD_OFF)
                await self.unsubscribe_notifications()
                self.sensorStarted=False
                self.logger.log(logging.INFO,'Stopped Sensor')
            else:
                self.logger.log(logging.ERROR,'Sensor not connected, can not send stop signal')
        else:
            self.logger.log(logging.ERROR, 'Could not stop: disconnected')

    def decode_data(self, data: bytearray):
        '''
        Decode the data received from the sensor
        '''
        
        # Time:
        #  There are three time stamps:
        #  Not sure which belongs to which sensor unit. 
        #   accelerometer is usually read faster than magnetometer, 
        #   gyroscope and accelerometer are usually on the same chip
        #   assuming earliest time stamp is the IMU time stamp
        self.sensorTime = (struct.unpack('<I', data[0:4])[0]   & 0xFFFFFFFF) /1000000.
        # self.aTime      = (struct.unpack('<I', data[16:20])[0] & 0xFFFFFFFF) /1000000.
        # self.bTime      = (struct.unpack('<I', data[32:36])[0] & 0xFFFFFFFF) /1000000.

        self.delta_sensorTime     = self.sensorTime - self.previous_sensorTime
        self.previous_sensorTime  = copy(self.sensorTime)
        # self.delta_aTime          = self.aTime      - self.previous_aTime
        # self.previous_aTime       = copy(self.aTime)
        # self.delta_bTime          = self.bTime      - self.previous_bTime
        # self.previous_bTime       = copy(self.bTime)

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
        #   Max observed value = 315
        #   Location 0/0 is equivalent to not touched
        #   Bottom right are the largest numbers
        #   Y axis is up-down
        #   X axis is left-right 
        self.touchX     = ((data[54] & 0xF) << 6) + ((data[55] & 0xFC) >> 2) & 0x3FF
        self.touchY     = ((data[55] & 0x3) << 8) + ((data[56] & 0xFF) >> 0) & 0x3FF

        # IMU:
        #  Accelerometer, Gyroscope, Magnetometer
        self.accX = struct.unpack('<h', data[4:6])[0]   * 0.00478840332  # 10000.0 * 9.80665 / 2048 / 10000.0       # m**2/s
        self.accY = struct.unpack('<h', data[6:8])[0]   * 0.00478840332  #
        self.accZ = struct.unpack('<h', data[8:10])[0]  * 0.00478840332  #
        self.gyrX = struct.unpack('<h', data[10:12])[0] * 0.001221791529 # 10000.0 * 0.017453292 / 14.285 / 1000.0 / 10. # rad/s
        self.gyrY = struct.unpack('<h', data[12:14])[0] * 0.001221791529 #
        self.gyrZ = struct.unpack('<h', data[14:16])[0] * 0.001221791529 #
        self.magY = struct.unpack('<h', data[48:50])[0] * 0.06           # micro Tesla, earth mag field 25..65 muTesla
        self.magX = struct.unpack('<h', data[50:52])[0] * 0.06           #
        self.magZ = struct.unpack('<h', data[52:54])[0] * 0.06           #

        # Rearrange the Axes
        # ------------------
        #
        # It was found for gearVRC when holding device and pointing forward:
        # acc.x points to the left/west
        # acc.y points towards user
        # acc.z points downwards
        #
        # mag.x points to the user
        # mag.y points to the right
        # mag.z points downwards
        #
        # gyr.x is counter clockwise around acc.x
        # gyr.y is counter clockwise around acc.y
        # gyr.z is counter clockwise around acc.z
        # 
        # We want x to point North, y to East and z to Down Orientation.
        # We also want rotation to be clockwise around axes.
        # Therefore we rearrange the following way:
        #   acc.x pointing forward      -acc.y
        #   acc.y pointing to the right -acc.x
        #   acc.z pointing downwards     acc.z
        #   mag.x pointing forward      -mag.x
        #   mag.y pointing to the right  mag.y
        #   mag.z pointing down          mag.z
        #   gyr.x clock wise forward     gyr.y
        #   gyr.y clockwise to the right gyr.x
        #   gyr.z clockwise downwards   -gyr.z
        # Or in short summary:
        #   acc -Y-X+Z
        #   mag -X+Y+Z
        #   gyr +Y+X-Z
        self.acc = Vector3D(-self.accY, -self.accX,  self.accZ)
        self.mag = Vector3D(-self.magX,  self.magY,  self.magZ)
        self.gyr = Vector3D( self.gyrY,  self.gyrX, -self.gyrZ)

        if self.firstTimeData:
            self.firstTimeData = False        
            self.gyr_average = self.gyr
            self.acc_average = self.acc
        else:
            self.gyr_average = 0.99*self.gyr_average + 0.01*self.gyr
            self.acc_average = 0.99*self.acc_average + 0.01*self.acc

        self.moving = detectMotion(self.acc.norm, self.gyr.norm, self.acc_average.norm, self.gyr_average.norm)
        if not self.moving:
            self.gyr_offset = 0.99*self.gyr_offset + 0.01*self.gyr
            self.gyr_offset_updated = True

    def compute_virtual(self):
        '''
        Compute virtual wheel and virtual touchpad
          The wheel is touched when finger slides along the rim of the touchpad
          The virtual touchpad is updated when the finger slides on the touchpad
        '''        
        if not (self.touchX == 0 and self.touchY == 0): 

            # Virtual Wheel
            #  Detects if rim of touchpad is touched and at what position 0..63
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

                self.delta_wheelPos = self.wheelPos - self.previous_wheelPos
                self.previous_wheelPos   = copy(self.wheelPos)
                # deal with discontinuity at 360/0:
                #   general formula with intervals along a circle in degrees is
                #   d_a = d_a - (360. * np.floor((d_a + 180.)/360.))
                #   resulting in d_a between < 180 and >=-180
                self.delta_wheelPos -= int(MAXWHEELPOS * np.floor((self.delta_wheelPos + MAXWHEELPOS2)/MAXWHEELPOS))
                if (self.delta_wheelPos > 0):
                    # rotating clock wise
                    self.isRotating = True
                    self.clockwise  = True
                elif (self.delta_wheelPos < 0):
                    # rotating counter clock wise
                    self.isRotating = True
                    self.clockwise  = False
                else:
                    self.isRotating = False

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
            # This assesses scrolling
            self.deltaX = self.touchX - self.previous_touchX
            self.deltaY = self.touchY - self.previous_touchY
            self.previous_touchX = copy(self.touchX)
            self.previous_touchY = copy(self.touchY)
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
                    self.dirDown = False
                else:
                    self.dirUp    = False
                    self.dirDown  = False
            
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

    def compute_fusion(self):
        '''
        Update AHRS and compute heading and roll/pitch/yaw
        Currently uses pyIMU MadgwickAHRS, in future might be upgraded to imufusion & https://pypi.org/project/imufusion/
        '''

        # Fusion data interval, is computed from sensor provided time stamp
        dt = self.sensorTime - self.previous_fusionTime # time interval between sensor data
        self.previous_fusionTime = copy(self.sensorTime) # keep track of last sensor time

        # Calibrate IMU Data
        self.acc_cal = calibrate(data=self.acc, offset=self.acc_offset, crosscorr=self.acc_crosscorr)
        self.mag_cal = calibrate(data=self.mag, offset=self.mag_offset, crosscorr=self.mag_crosscorr)
        self.gyr_cal = calibrate(data=self.gyr, offset=self.gyr_offset, crosscorr=self.gyr_crosscorr)

        if dt > 1.0: 
            # First run or reconnection, need AHRS algorithm to initialize
            if (self.mag_cal.norm >  MAGFIELD_MAX) or (self.mag_cal.norm < MAGFIELD_MIN):
                # Mag is not in acceptable range
                self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=None,dt=-1)
                self.magok = False
            else:
                self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=self.mag_cal,dt=-1)
                self.magok = True

        else:
            if (self.mag_cal.norm >  MAGFIELD_MAX) or (self.mag_cal.norm < MAGFIELD_MIN):
                # Mag not in acceptable range
                self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=None,dt=dt)
                self.magok = False
            else:
                self.q = self.AHRS.update(acc=self.acc_cal,gyr=self.gyr_cal,mag=self.mag_cal,dt=dt)
                self.magok = True

        self.rpy=q2rpy(q=self.q)
        self.heading = rpymag2h(rpy=self.rpy, mag=self.mag_cal, declination=DECLINATION)

    def compute_motion(self):
        self.Position.update(q=self.q, acc=self.acc_cal, moving=self.moving, timestamp=self.sensorTime)
        self.residuals    = self.Position.worldResiduals
        self.velocity     = self.Position.worldVelocity
        self.position     = self.Position.worldPosition
        self.dtmotion     = self.Position.dtmotion
        self.dt           = self.Position.dt
        self.accBias      = self.Position.residuals_bias
        self.velocityBias = self.Position.worldVelocity_drift
                    
    def check_ESC_sequence(self):
        '''
        Was the home button pressed the necessary amount of times within the timeout?
        '''
        if self.home == True:
            if not self.previous_home:
                # Home button was pressed
                elapsed = time.perf_counter() - self.home_pressedTime
                self.home_updateCounts += 1
                self.previous_home = True
                
                if self.home_updateCounts == 1:
                    self.home_pressedTime = time.perf_counter()

                if (self.home_updateCounts == 2) and (elapsed > self.ESC_timeout):
                    # start over
                    self.home_updateCounts = 1
                    self.home_pressedTime = time.perf_counter()

                elif self.home_updateCounts == self.ESC_reps:
                    if elapsed <= self.ESC_timeout:
                        # ESC detected
                        return True
                    else:
                        # To slow, ESC not detected, reset
                        self.home_updateCounts = 0
            else: 
                # it remains pressed
                pass
        else:
            # its not pressed
            self.previous_home = False
            
        return False

    ##############################################################################################
    # Bleak Call BacK
    ##############################################################################################

    async def handle_data(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        '''
        This routine is called by bleak notifications. It will:
        - Decode the sensor data
            Compared to this Gear VRC reverse engineering (Java): https://github.com/jsyang/gearvr-controller-webbluetooth
            this code includes the 3 different time stamps as well as correct magnetometer data
        - ESC sequence
            Check if home button is pressed three times in a row within timeout seconds
        - fusion
            Fuse IMU data to pose, roll, pitch, yaw, and heading
        -virtual    
            Create Virtual Wheel
            Create Extended Touchpad
            Inspired from https://github.com/rdady/gear-vr-controller-linux
            This routine can be throttled to lower update rate
        '''

        startTime = time.perf_counter()

        if self.sensorIsBooting:
            self.sensorRunInCounts += 1
            if self.sensorRunInCounts > 50:
                self.sensorIsBooting = False
                self.sensorRunInCounts = 0

        # Update rate
        self.data_updateCounts += 1
        if startTime - self.data_lastTimeRate >= 1:
            self.data_rate = copy(self.data_updateCounts)
            self.data_lastTimeRate = copy(startTime)
            self.data_updateCounts = 0

        if self.paused:
            await asyncio.sleep(1) # allow other tasks to run
            
        else:
            # Process the Data
            ###############################################################
            if not self.sensorIsBooting:

                if (len(data) >= 60):

                    # Decode
                    ###############################################################
                    self.decode_data(data)

                    # ESC sequence
                    ###############################################################
                    if self.check_ESC_sequence():
                        self.logger.log(logging.INFO, 'ESC detected')
                        self.terminate.set()
                            
                    # Virtual
                    ###############################################################
                    if self.virtual:
                        # We throttle the update rate of the virtual features
                        if startTime - self.previous_virtualUpdate >= self.virtual_updateInterval:
                            self.previous_virtualUpdate = copy(startTime)
                            #
                            start_virtualUpdate = time.perf_counter()
                            self.virtual_updateCounts += 1
                            if (startTime - self.virtual_lastTimeRate)>= 1.:
                                self.virtual_rate = copy(self.virtual_updateCounts)
                                self.virtual_lastTimeRate = copy(startTime)
                                self.virtual_updateCounts = 0
                            #
                            self.compute_virtual()

                            self.virtual_deltaTime = time.perf_counter() - start_virtualUpdate

                    # Fusion
                    ###############################################################
                    if self.fusion:
                        # update interval
                        start_fusionUpdate = time.perf_counter()
                        # fps
                        self.fusion_updateCounts += 1
                        if (startTime - self.fusion_lastTimeRate)>= 1.:
                            self.fusion_rate = copy(self.fusion_updateCounts)
                            self.fusion_lastTimeRate = copy(startTime)
                            self.fusion_updateCounts = 0
                        #
                        self.compute_fusion()
                        self.fusion_deltaTime = time.perf_counter() - start_fusionUpdate

                    # Motion
                    ###############################################################
                    if self.motion:
                        # update interval
                        start_motionUpdate = time.perf_counter()
                        # fps
                        self.motion_updateCounts += 1
                        if (startTime - self.motion_lastTimeRate)>= 1.:
                            self.motion_rate = copy(self.motion_updateCounts)
                            self.motion_lastTimeRate = copy(startTime)
                            self.motion_updateCounts = 0
                            
                        if not self.motionIsBooting:
                            # we need some time to compute averages, once system is stable start compute motion.
                            self.compute_motion()
                        else:
                            if start_motionUpdate - self.firstTimeMotion > 5.0:
                                self.motionIsBooting = False
                                
                        self.motion_deltaTime = time.perf_counter() - start_motionUpdate

                else:
                    self.logger.log(logging.ERROR, 'Not enough values: {}'.format(len(data)))
                    # switch to sensor mode
                    await self.start_sensor(VRMode = False)

                self.processedDataAvailable.set()
                
                self.data_deltaTime = time.perf_counter() - startTime

            await asyncio.sleep(0) # allow other tasks to run


    def handle_batteryData(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        
        # print('B', end='', flush=True)

        self.battery_level = int.from_bytes(data,'big')

    ##############################################################################################
    # gearVRC Tasks
    ##############################################################################################

    async def keep_alive_loop(self):
        '''
        Periodically send Keep Alive commands
        Does not prevent disconnection when device is left idle
        Not sure if this is needed.
        '''
        self.logger.log(logging.INFO, 'Starting Keeping Alive Task...')

        while not self.finish_up:

            # print('K', end='', flush=True)

            if (self.client is not None) and self.connected and self.sensorStarted and (not self.paused):
                await self.client.write_gatt_char(self.controller_command_characteristics,CMD_KEEP_ALIVE)
                # self.logger.log(logging.DEBUG,'Keep alive sent')        
                sleepTime=KEEPALIVEINTERVAL
            else:
                if self.sensorStarted and self.connected: self.logger.log(logging.ERROR,'Could not send Keep alive')
                # do not report keep alive issues if sensor is not yet running            
                sleepTime=1

            await asyncio.sleep(sleepTime)
        
        self.logger.log(logging.INFO, 'Keeping Alive stopped')

    async def update_gyrOffset(self):
        '''
        Save new Gyroscope Offset in Calibration File every minute
        '''

        self.logger.log(logging.INFO, 'Starting Gyroscope Bias Saving Task...')

        while not self.finish_up:

            # print('Bias', end='', flush=True)

            if self.gyr_offset_updated:
                my_file = pathlib.Path(self.current_directory + '/Gyr.json')
                saveCalibration(my_file, self.gyr_offset, self.gyr_crosscorr)
                self.gyr_offset_updated = False

            await asyncio.sleep(60.0)

        self.logger.log(logging.INFO, 'Gyroscope Bias Saving Task stopped')


    async def report_loop(self):
        '''
        Report latest fused data
        Adapt report to whether fusion, virtual, serial or zmq is enabled 
        '''

        self.logger.log(logging.INFO, 'Starting Reporting Task...')

        self.report_lastTimeRate    = time.perf_counter()
        self.report_updateCounts    = 0

        await self.processedDataAvailable.wait()
        # no clear needed as we just wait for system to start

        while not self.finish_up:

            # print('R', end='', flush=True)

            startTime = time.perf_counter()

            if (self.report > 0) and (not self.paused):
                
                self.report_updateCounts += 1
                if (startTime - self.report_lastTimeRate)>= 1.:
                    self.report_rate = copy(self.report_updateCounts)
                    self.report_lastTimeRate = time.perf_counter()
                    self.report_updateCounts = 0

                # Display the Data
                msg_out  = '\033[2J\n'
                msg_out += '-------------------------------------------------\n'
                if self.report > 1:
                    msg_out+= 'gearVR Ctr: Temp {:>4.1f}, Bat {:>3d}, HighRes:{}, Moving:{}, Mag:{}\n'.format(
                                                        self.temperature, self.battery_level, 
                                                        'Y' if self.HighResMode else 'N',
                                                        'Y' if self.moving else 'N',
                                                        'Y' if self.magok else 'N'
                                                        )
                else:
                    msg_out+= 'gearVR Ctr: Temp {:>4.1f}, Bat {:>3d}, Moving:{}, Mag:{}\n'.format(
                                                        self.temperature, self.battery_level,
                                                        'Y' if self.moving else 'N',
                                                        'Y' if self.magok else 'N')
                msg_out+= '-------------------------------------------------\n'

                msg_out+= 'Data    {:>10.6f}, {:>3d}/s\n'.format(self.data_deltaTime*1000.,        self.data_rate)
                msg_out+= 'Report  {:>10.6f}, {:>3d}/s\n'.format(self.report_deltaTime*1000.,      self.report_rate)
                if self.virtual:
                    msg_out+= 'Virtual {:>10.6f}, {:>3d}/s\n'.format(self.virtual_deltaTime*1000., self.virtual_rate)
                if self.fusion:
                    msg_out+= 'Fusion  {:>10.6f}, {:>3d}/s\n'.format(self.fusion_deltaTime*1000.,  self.fusion_rate)
                if self.serial is not None:
                    msg_out+= 'Serial  {:>10.6f}, {:>3d}/s\n'.format(self.serial_deltaTime*1000.,  self.serial_rate)
                if self.zmqPortPUB is not None:
                    msg_out+= 'ZMQ     {:>10.6f}, {:>3d}/s\n'.format(self.zmqPUB_deltaTime*1000.,  self.zmqPUB_rate)
                msg_out+= '-------------------------------------------------\n'

                if self.report > 1:
                    # print('R1', end='', flush=True)
                    # msg_out+= 'Time  {:>10.6f}, {:>10.6f}, {:>10.6f}\n'.format(self.sensorTime, self.aTime, self.bTime)
                    # msg_out+= 'dt    {:>10.6f}, {:>10.6f}, {:>10.6f}\n'.format(self.delta_sensorTime, self.delta_aTime, self.delta_bTime)
                    msg_out+= 'Time  {:>10.6f}, dt {:>10.6f}\n'.format(self.sensorTime, self.delta_sensorTime)
                    msg_out+= 'Accel     {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z)
                    msg_out+= 'Gyro      {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.gyr.x,self.gyr.y,self.gyr.z)
                    msg_out+= 'Magno     {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z)
                    msg_out+= 'Accel avg {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc_average.x, self.acc_average.y, self.acc_average.z, self.acc_average.norm)
                    msg_out+= 'Gyro  avg {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_average.x, self.gyr_average.y, self.gyr_average.z, self.gyr_average.norm*60./TWOPI)
                    msg_out+= 'Gyro bias {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_offset.x, self.gyr_offset.y, self.gyr_offset.z, self.gyr_offset.norm*60./TWOPI)
                    msg_out+= 'Moving:   {}\n'.format('Y' if self.moving else 'N')
                    msg_out+= '-------------------------------------------------\n'
                    # print('R2', end='', flush=True)

                    msg_out+= 'Trig:{} Touch:{} Back:{} Home:{}, Vol+:{} Vol-:{} Any:{}\n'.format(
                                                        'Y' if self.trigger     else 'N', 
                                                        'Y' if self.touch       else 'N', 
                                                        'Y' if self.back        else 'N',
                                                        'Y' if self.home        else 'N',
                                                        'Y' if self.volume_up   else 'N',
                                                        'Y' if self.volume_down else 'N',
                                                        'N' if self.noButton    else 'Y')
                    msg_out+= '-------------------------------------------------\n'
                    # print('R3', end='', flush=True)
                    msg_out+= 'TPad:  {:>3d},{:>3d}\n'.format(self.touchX, self.touchY)

                    if self.virtual:
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

                    # print('R4', end='', flush=True)
                    if self.fusion:
                        msg_out+= 'Acc     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc_cal.x,self.acc_cal.y,self.acc_cal.z,self.acc_cal.norm)
                        msg_out+= 'Gyr     {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr_cal.x,self.gyr_cal.y,self.gyr_cal.z,self.gyr_cal.norm*60./TWOPI)
                        msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.mag_cal.x,self.mag_cal.y,self.mag_cal.z,self.mag_cal.norm)
                        msg_out+= 'Euler: R{:>6.1f} P{:>6.1f} Y{:>6.1f}, Heading {:>4.0f}\n'.format(
                                                        self.rpy.x*RAD2DEG, self.rpy.y*RAD2DEG, self.rpy.z*RAD2DEG, 
                                                        self.heading*RAD2DEG)
                        msg_out+= 'Q:     W{:>6.3f} X{:>6.3f} Y{:>6.3f} Z{:>6.3f}\n'.format(
                                                        self.q.w, self.q.x, self.q.y, self.q.z)
                        
                        msg_out+= 'Using Mag in AHRS: {}\n'.format('Y' if self.magok else 'N')

                    #print('R5', end='', flush=True)
                    if self..motion:
                        msg_out+= '-------------------------------------------------\n'
                        # print('R6a', end='', flush=True)
                        msg_out+= 'Residual {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.residuals.x,self.residuals.y,self.residuals.z,self.residuals.norm)
                        msg_out+= 'Vel      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.velocity.x,self.velocity.y,self.velocity.z,self.velocity.norm)
                        msg_out+= 'Pos      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.position.x,self.position.y,self.position.z,self.position.norm)
                        msg_out+= 'Vel Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.velocityBias.x,self.velocityBias.y,self.velocityBias.z,self.velocityBias.norm)
                        msg_out+= 'Acc Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.accBias.x,self.accBias.y,self.accBias.z,self.accBias.norm)
                        # print('R6', end='', flush=True)
                        msg_out+= 'dt       {:>10.6f} s {:>10.6f} ms\n'.format(self.dtmotion, self.dt*1000.)

                    #print('R6', end='', flush=True)
                print(msg_out, flush=True)

                self.report_deltaTime = time.perf_counter() - startTime

                # Wait to next interval time
                sleepTime = self.report_updateInterval - (time.perf_counter() - startTime)
                await asyncio.sleep(max(0.,sleepTime))
                timingError = time.perf_counter() - startTime - self.report_updateInterval
                self.report_updateInterval = max(0., REPORTINTERVAL - timingError)

            else:

                # check every second if reporting is turned back on or unpaused
                await asyncio.sleep(1.0)
                
        self.logger.log(logging.INFO, 'Reporting stopped')

    async def serial_loop(self):
        '''
        Report latest fused data over serial connection
        This is formatted for freeIMU calibration GUI software
        '''

        if self.args.serial is not None:
       
            self.logger.log(logging.INFO, 'Creating serial reader and writer with {} at {} baud...'.format(self.serialport, self.baudrate))
            reader, writer = await serial_asyncio.open_serial_connection(url=self.serialport, baudrate=self.baudrate)

            # writer.write('Welcome to gearVRC\r\n'.encode())
        
            while not self.finish_up:

                # print('S', end='', flush=True)
                
                msg_in = await reader.readline() # read a full line, needs to have /n at end of line (Ctrl-J in putty)
                if self.serial:
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
                                self.serial_updateCounts = 0

                                for i in range(count):

                                    await self.processedDataAvailable.wait()
                                    self.processedDataAvailable.clear()

                                    self.serial_updateCounts += 1
                                
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
                                    line_out = accX_hex + accY_hex + accZ_hex + \
                                            gyrX_hex + gyrY_hex + gyrZ_hex + \
                                            magX_hex + magY_hex + magZ_hex + '\r\n'
                                    msg_out=line_out.encode()
                                    writer.write(msg_out)

                                self.serial_deltaTime = time.perf_counter() - startTime
                                self.serial_rate = int(self.serial_updateCounts / self.serial_deltaTime)
                                self.serial_updateCounts = 0
                            else:
                                pass # unknown command
                    else:
                        pass # empty line received
                    await asyncio.sleep(0)
                else:
                    await asyncio.sleep(1.0) # wait until serial is turned on
            # end while loop
        else:
            self.logger.log(logging.INFO, 'No serial reader and writer ...')
            self.serial = False
            
        self.logger.log(logging.INFO, 'Serial stopped')
    
    async def zmqPUB_loop(self):
        '''
        Report data on ZMQ socket
        There are 6 data packets presented:
         - system 
         - imu
         - button
         - touch
         - virtual if enabled
         - fusion if enabled
        '''

        self.logger.log(logging.INFO, 'Creating ZMQ Publisher at \'tcp://*:{}\' ...'.format(self.args.zmqport))

        context = zmq.asyncio.Context()      
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:{}".format(self.args.zmqport))

        data_system  = gearSystemData()
        data_imu     = gearIMUData()
        data_button  = gearButtonData()
        data_touch   = gearTouchData()
        data_virtual = gearVirtualData()
        data_fusion  = gearFusionData()
        data_motion  = gearMotionData()

        self.zmqPUB_lastTimeRate   = time.perf_counter()
        self.zmqPUB_updateCounts   = 0

        while not self.finish_up:

            # print('Z', end='', flush=True)

            startTime = time.perf_counter()

            await self.processedDataAvailable.wait()
            self.processedDataAvailable.clear()

            # fps
            self.zmqPUB_updateCounts += 1
            if (startTime - self.zmqPUB_lastTimeRate)>= 1.:
                self.zmqPUB_rate = copy(self.zmqPUB_updateCounts)
                self.zmqPUB_lastTimeRate = copy(startTime)
                self.zmqPUB_updateCounts = 0

            # format the imu data
            # these are the axis flipped but not calibrated data
            data_imu.time   = self.sensorTime
            data_imu.acc    = self.acc
            data_imu.gyr    = self.gyr
            data_imu.mag    = self.mag
            data_imu.moving = self.moving
            data_imu.magok  = self.magok
            imu_msgpack = msgpack.packb(obj2dict(vars(data_imu)))
            socket.send_multipart([b"imu", imu_msgpack])               
            # print('Zimu', end='', flush=True)

            data_button.time        = self.sensorTime
            data_button.trigger     = self.trigger
            data_button.touch       = self.touch
            data_button.back        = self.back
            data_button.home        = self.home
            data_button.volume_up   = self.volume_up
            data_button.volume_down = self.volume_down
            data_button.noButton    = self.noButton
            data_button.touchX      = self.touchX
            data_button.touchY      = self.touchY
            button_msgpack = msgpack.packb( obj2dict(vars(data_button)))
            socket.send_multipart([b"button",button_msgpack])               
            # print('Zbutton', end='', flush=True)

            data_touch.time        = self.sensorTime
            data_touch.touchX      = self.touchX
            data_touch.touchY      = self.touchY
            touch_msgpack = msgpack.packb(obj2dict(vars(data_touch)))
            socket.send_multipart([b"touch", touch_msgpack])               
            # print('Ztouch', end='', flush=True)

            # format the system data
            data_system.temperature    = self.temperature
            data_system.battery_level  = self.battery_level
            data_system.data_rate      = self.data_rate
            data_system.virtual_rate   = self.virtual_rate
            data_system.fusion_rate    = self.fusion_rate
            data_system.zmq_rate       = self.zmqPUB_rate
            data_system.serial_rate    = self.serial_rate
            data_system.reporting_rate = self.report_rate
            data_system.fusion         = self.fusion
            data_system.virtual        = self.virtual
            data_system.motion         = self.motion
            data_system.serial         = self.serial
            system_msgpack = msgpack.packb(obj2dict(vars(data_system)))
            socket.send_multipart([b"system", system_msgpack])               
            # print('Zsys', end='', flush=True)

            if self.virtual:
                # format the virtual data (wheel and touchpad)
                data_virtual.time       = self.sensorTime
                data_virtual.absX       = self.absX
                data_virtual.absY       = self.absY
                data_virtual.dirUp      = self.dirUp
                data_virtual.dirDown    = self.dirDown
                data_virtual.dirLeft    = self.dirLeft
                data_virtual.dirRight   = self.dirRight
                data_virtual.wheelPos   = self.wheelPos
                data_virtual.top        = self.top
                data_virtual.bottom     = self.bottom
                data_virtual.left       = self.left
                data_virtual.right      = self.right
                data_virtual.center     = self.center
                data_virtual.isRotating = self.isRotating
                data_virtual.clockwise  = self.clockwise
                virtual_msgpack = msgpack.packb(obj2dict(vars(data_virtual)))
                socket.send_multipart([b"virtual", virtual_msgpack])
                # print('Zvirt', end='', flush=True)

            if self.fusion:
                # report fusion data
                data_fusion.time = self.sensorTime
                data_fusion.acc  = self.acc_cal
                data_fusion.gyr  = self.gyr_cal
                data_fusion.mag  = self.mag_cal
                data_fusion.rpy  = self.rpy
                data_fusion.heading = self.heading
                data_fusion.q   = self.q
                fusion_msgpack = msgpack.packb(obj2dict(vars(data_fusion)))
                socket.send_multipart([b"fusion", fusion_msgpack])   
                # print('Zfuse', end='', flush=True)
                
            if self.motion:
                data_motion.time      = self.sensorTime
                data_motion.accBias   = self.accBias
                data_motion.velocityBias = self.velocityBias
                data_motion.position  = self.position
                data_motion.velocity  = self.velocity
                data_motion.residuals = self.residuals
                data_motion.dtmotion  = self.dtmotion
                motion_msgpack = msgpack.packb(obj2dict(vars(data_motion)))
                socket.send_multipart([b"motion", motion_msgpack])
                # print('Zmotion', end='', flush=True)

            # update interval
            self.zmqPUB_deltaTime = time.perf_counter() - startTime

            await asyncio.sleep(0)

        self.logger.log(logging.INFO, 'ZMQ stopped')

    async def zmqREP_loop(self):
        '''
        Handle program control
        - fusion on/off
        - motion on/off
        - report on/off
        - virtual on/off
        - serial on/off
        - stop
        '''

        context = zmq.asyncio.Context()
        socket  = context.socket(zmq.REP)
        socket.bind("tcp://*:{}".format(self.zmqPortREP))

        poller = zmq.asyncio.Poller()
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'Created ZMQ Request/Reply at \'tcp://*:{}\' ...'.format(self.zmqPortREP))

        while not self.finish_up:

            startTime = time.perf_counter()
            
            try: 
                events = dict(await poller.poll(timeout=-1))
                if socket in events and events[socket] == zmq.POLLIN:
                    response = await socket.recv_multipart()
                    if len(response) == 2:
                        [topic, value] = response
                        if topic == b"motion":
                            if value == b"\x01":
                                self.motion = \
                                self.fusion = True
                            else:
                                self.motion = \
                                self.fusion = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ motion received: {}'.format(value))
                        elif topic == b"fusion":
                            if value == b"\x01": self.fusion = True
                            else:                self.fusion = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ fusion received: {}'.format(value))
                        elif topic == b"report":
                            self.report = int.from_bytes(value, byteorder='big', signed=True)
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ report received: {}'.format(value))
                        elif topic == b"virtual":
                            if value == b"\x01": self.virtual = True
                            else:                self.virtual = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ fusion received: {}'.format(value))
                        elif topic == b"serial":
                            if value == b"\x01": self.serial = True
                            else:                self.serial = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ fusion received: {}'.format(value))
                        elif topic == b"stop":
                            if value == b"\x01": 
                                self.terminate.set()
                                self.finish_up = True
                            else:          
                                self.terminate.clear()
                                self.finish_up = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ stop received: {}'.format(value))
                        elif topic == b"pause":
                            if value == b"\x01": 
                                self.paused=True
                            else:          
                                self.paused = False
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ pause received: {}'.format(value))
                        elif topic == b"continue":
                            if value == b"\x01": 
                                self.paused = False
                            else:          
                                self.paused = True
                            socket.send_string("OK")
                            self.logger.log(logging.INFO, 'ZMQ continue received: {}'.format(value))
                        else:
                            socket.send_string("UNKNOWN")
                            self.logger.log(logging.INFO, 'ZMQ received UNKNOWN')
                    else:
                        self.logger.log(logging.ERROR, 'ICM zmqWorker receiver malformed REQ message')
                        socket.send_string("ERROR")

            except:
                self.logger.log(logging.ERROR, 'ICM zmqWorker REQ/REP error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.REP)
                socket.bind("tcp://*:{}".format(self.zmqPortREP))
                poller.register(socket, zmq.POLLIN)
            
            await asyncio.sleep(0)
                
            # update interval
            self.zmqREP_deltaTime = time.perf_counter() - startTime

        self.logger.log(logging.INFO, 'ZMQ REQ/REP finished')
        socket.close()
        context.term()

    async def handle_termination(self, tasks):
        '''
        Stop the while loops in the tasks
        Stop the sensor from producing data
        Cancel slow tasks based on provided list (speed up closing for program)
        '''
        self.logger.log(logging.INFO, 'Controller ESC, Control-C or Kill signal detected')
        self.finish_up = True
        await self.stop_sensor()
        await self.disconnect()
        if tasks is not None: # This will terminate tasks faster
            self.logger.log(logging.INFO, 'Cancelling all Tasks...')
            await asyncio.sleep(1) # give some time for tasks to finish up
            for task in tasks:
                if task is not None:
                    task.cancel()

    async def terminator_loop(self, tasks):
        '''
        Wrapper for Task Termination
        Waits for termination signal and then executes the termination sequence
        '''
        self.logger.log(logging.INFO, 'Starting Terminator...')

        while not self.finish_up:
            await self.terminate.wait()
            self.terminate.clear()
            await self.handle_termination(tasks=tasks)

        self.logger.log(logging.INFO, 'Terminator completed')

##############################################################################################
# MAIN
##############################################################################################

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'Starting gearVR Controller...')

    # gearVRC Controller
    controller = gearVRC(logger=logger, args=args)

    # Create all the async tasks
    # They will run until stop signal is created, stop signal is indicated with event
    connection_task = asyncio.create_task(controller.connect_loop())      # remain connected, will not terminate
    keepalive_task  = asyncio.create_task(controller.keep_alive_loop())          # keep sensor alive, will not terminate

    tasks = [connection_task, keepalive_task] # frequently used tasks
    terminator_tasks = [keepalive_task]       # slow tasks, long wait times

    # fusion_task     = asyncio.create_task(controller.update_fusion()) # update pose, will not terminate
    # tasks.append(fusion_task)
    gyroffset_task  = asyncio.create_task(controller.update_gyrOffset())
    tasks.append(gyroffset_task)
    terminator_tasks.append(gyroffset_task)

    reporting_task  = asyncio.create_task(controller.report_loop())   # report new data, will not terminate
    tasks.append(reporting_task)

    serial_task     = asyncio.create_task(controller.serial_loop())   # update serial, will not terminate
    tasks.append(serial_task)

    zmq_task_pub     = asyncio.create_task(controller.zmqPUB_loop())  # update zmq, will not terminate
    tasks.append(zmq_task_pub)
 
    zmq_task_rep   = asyncio.create_task(controiller.zmqREP_loop())   # update zmq, will not terminate
    tasks.append(zmq_task_rep)
 
    terminator_task = asyncio.create_task(controller.terminator_loop(terminator_tasks)) # make sure we shutdown keep alive in timely fashion (has long sleep)
    tasks.append(terminator_task)

    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(controller.handle_termination(tasks=tasks)) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(controller.handle_termination(tasks=tasks)) ) # kill

    # Wait until all tasks are completed, which is when user wants to terminate the program
    await asyncio.wait(tasks, timeout=float('inf'))

    logger.log(logging.INFO,'gearVR Controller exit')

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
        help='sets the log level from info to debug',
        default = False
    )

    parser.add_argument(
        '-r',
        '--report',
        dest = 'report',
        type = int,
        metavar='<report>',
        help='report level: 0(None), 1(Rate only), 2(Regular)',
        default = 0
    )

    parser.add_argument(
        '-f',
        '--fusion',
        dest = 'fusion',
        action='store_true',
        help='turns on IMU data fusion',
        default = False
    )

    parser.add_argument(
        '-m',
        '--motion',
        dest = 'motion',
        action='store_true',
        help='turns on velocity and position computation',
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
        '-zp',
        '--zmq_pub',
        dest = 'zmqPortPUB',
        type = int,
        metavar='<zmqPortPUB>',
        help='port used by ZMQ, e.g. 5556',
        default = 5556
    )

    parser.add_argument(
        '-zr',
        '--zmq_rep',
        dest = 'zmqPortREP',
        type = int,
        metavar='<zmqPortREP>',
        help='port used by ZMQ, e.g. 5555',
        default = 5555
    )

    parser.add_argument(
        '-e',
        '--escreps',
        dest = 'escreps',
        type = int,
        metavar='<escreps>',
        help='number of consecutive HOME presses to terminate program',
        default = 3
    )

    parser.add_argument(
        '-t',
        '--timeout',
        dest = 'esctimeout',
        type = float,
        metavar='<esctimeout>',
        help='interval for consecutive HOME presses in seconds to terminate program',
        default = 2.
    )

    args = parser.parse_args()

    if args.motion: args.fusion=True
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        # format='%(asctime)-15s %(name)-8s %(levelname)s: %(message)s'
        format='%(asctime)-15s %(levelname)s: %(message)s'
    )   
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
