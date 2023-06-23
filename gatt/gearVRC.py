#!/usr/bin/python3

# The library is free.
# MIT License
# Copyright (c) 2019, Robert K. Dady
# Copyright (c) 2023, Urs Utzinger
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Based on:
# BLE Device Handling: https://github.com/getsenic/gatt-python
# Gear VRC reverse engineering (Java): https://github.com/jsyang/gearvr-controller-webbluetooth
# Gear VRC to UDEV (Python): https://github.com/rdady/gear-vr-controller-linux
# Gear VRC Windows: https://github.com/gb2111/Access-GearVR-Controller-from-PC    
# 
# Prerequisite:
# A) python packages:
#   $ sudo pip3 install gatt more-itertools ahrs
# B) Pair Controller:
#   $ bluetoothlctl
#     scan on
#     pair yourMAC
#     trust yourMAC
#     connect yourMAC
#
# Controller Service
# [2C:BA:BA:2E:17:DB]     Service [4f63756c-7573-2054-6872-65656d6f7465]                    # CUSTOM SERVICE
# [2C:BA:BA:2E:17:DB]             Characteristic [c8c51726-81bc-483b-a052-f7a14ea3d282]     # COMMAND SEND
# [2C:BA:BA:2E:17:DB]             Characteristic [c8c51726-81bc-483b-a052-f7a14ea3d281]     # DATA RECEIVE
#
# Keyboard Information
# [2C:BA:BA:2E:17:DB]     Service [00001879-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a22-0000-1000-8000-00805f9b34fb]     # "Boot Keyboard Input Report" Notifications
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a32-0000-1000-8000-00805f9b34fb]     # "Boot Keyboard Output Report"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4a-0000-1000-8000-00805f9b34fb]     # "HID Information"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4b-0000-1000-8000-00805f9b34fb]     # "Report Map"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4c-0000-1000-8000-00805f9b34fb]     # "HID Control Point"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4d-0000-1000-8000-00805f9b34fb]     # "Report" Notifications
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4e-0000-1000-8000-00805f9b34fb]     # "Protocol Mode"
#
# Device Information
# [2C:BA:BA:2E:17:DB]     Service [0000180a-0000-1000-8000-00805f9b34fb] k UUID a.class
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a50-0000-1000-8000-00805f9b34fb]     # "PnP ID" t UUID
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a28-0000-1000-8000-00805f9b34fb]     # "Software Revision String" q UUID
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a26-0000-1000-8000-00805f9b34fb]     # "Firmware Revision String" p UUID
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a27-0000-1000-8000-00805f9b34fb]     # "Hardware Revision String" o UUID
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a25-0000-1000-8000-00805f9b34fb]     # "Serial Number String" n UUID
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a24-0000-1000-8000-00805f9b34fb]     # "Model Number String"  m UUID
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a29-0000-1000-8000-00805f9b34fb]     # "Manufacturer Name String" l UUID
#
# Battery
# [2C:BA:BA:2E:17:DB]     Service [0000180f-0000-1000-8000-00805f9b34fb] a UUID a.class
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a19-0000-1000-8000-00805f9b34fb]     # Battery level, Notifications b UUID, a.class
#
# General
# [2C:BA:BA:2E:17:DB]     Service [00001801-0000-1000-8000-00805f9b34fb]                    # Generic Attribute
# Read UUID 00002a00-0000-1000-8000-00805f9b34fb: b'4765617220565220436f6e74726f6c6c657228' # Device Name (Gear VR Controller)
# Read UUID 00002a01-0000-1000-8000-00805f9b34fb: b'c003'                                   # Appearance
# Read UUID 00002a04-0000-1000-8000-00805f9b34fb: b'0b000b000000e803'                       # Peripheral Preferred Connection Parameters
#
# UNKNOWN/FIRMWARE UPDATE
# c,d,e,f,g,h,i class
# [2C:BA:BA:2E:17:DB]     Service [0000fef5-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [b7de1eea-823d-43bb-a3af-c4903dfce23c]     # UNKNOWN
# [2C:BA:BA:2E:17:DB]             Characteristic [42c3dfdd-77be-4d9c-8454-8f875267fb3b]
# [2C:BA:BA:2E:17:DB]             Characteristic [64b4e8b5-0de5-401b-a21d-acc8db3b913a]
# [2C:BA:BA:2E:17:DB]             Characteristic [61c8849c-f639-4765-946e-5c3419bebb2a]
# [2C:BA:BA:2E:17:DB]             Characteristic [5f78df94-798c-46f5-990a-b3eb6a065c88]     # Notifications
# [2C:BA:BA:2E:17:DB]             Characteristic [457871e8-d516-4ca1-9116-57d0b17b9cb2]
# [2C:BA:BA:2E:17:DB]             Characteristic [9d84b9a3-000c-49d8-9183-855b673fda31]
# [2C:BA:BA:2E:17:DB]             Characteristic [6c53db25-47a1-45fe-a022-7c92fb334fd4]
# [2C:BA:BA:2E:17:DB]             Characteristic [724249f0-5ec3-4b5f-8804-42345af08651]
# [2C:BA:BA:2E:17:DB]             Characteristic [8082caa8-41a6-4021-91c6-56f9b954cc34]
#

MACADDRESS                       = '2C:BA:BA:2E:17:DB'
CMD_OFF                          = bytearray(b'\x00\x00') # Turn modes off and stop sending data
CMD_SENSOR                       = bytearray(b'\x01\x00') # Touchpad and sensor buttons but low rate IMU data
CMD_UNKNOWN_FIRMWARE_UPDATE_FUNC = bytearray(b'\x02\x00') # Initiate frimware update sequence
CMD_CALIBRATE                    = bytearray(b'\x03\x00') # Initiate calibration: Not sure how to compensate for drift
CMD_KEEP_ALIVE                   = bytearray(b'\x04\x00') # Keep alive: Not sure about time interval
CMD_UNKNOWN_SETTING              = bytearray(b'\x05\x00') # Setting mode: ?
CMD_LPM_ENABLE                   = bytearray(b'\x06\x00') # ?
CMD_LPM_DISABLE                  = bytearray(b'\x07\x00') # ?
CMD_VR_MODE                      = bytearray(b'\x08\x00') # Enable VR mode: high frequency event mode

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
MINXTOUCH                        = 0                        # Virtual touchpad
MINYTOUCH                        = 0
MAXXTOUCH                        = 1024
MAXYTOUCH                        = 1024

MINIMUPDATETIME                  = 0.23                     # Not sure haw fast this device might be

KEEPALIVEINTERVAL                = 60                       # Every minute
BATTERYINTERVAL                  = 60*5                     # Every 5 minutes

TUCSON_LATITUDE                  = 32.253460
TUCSON_LONGITUDE                 =-110.911789
TUCSON_HEIGHT                    = 0.759
NUMSAMPLES                       = 2

import gatt
import math
import time
import numpy as np
import sys
import more_itertools as mit
import collections
import logging
import struct
import ahrs

def ror(l, n):
    '''
    split list at position -n and attach right section in front left section
    '''
    return l[-n:] + l[:-n]

def clamp(val, smallest, largest): 
    '''
    Clip val to [smallest, largest]
    '''
    if val < smallest: return smallest
    if val > largest: return largest
    return val

def calibrate(x,y,z, maxX, minX, maxY, minY, maxZ, minZ, offsetX, offsetY, offsetZ, crosscorr):
    # Max Min
    if (x >= 0):
        x /= maxX
    else:
        x /= minX
    if (y >= 0):
        y /= maxY
    else:
        y /= minY
    if (z >= 0):
        z /= maxZ
    else:
        z /= minZ

    # Offset
    x -= offsetX
    y -= offsetY
    z -= offsetZ

    # Cross Correlation
    x = x * crosscorr[0][0] + y * crosscorr[0][1] + z * crosscorr[0][2]
    y = x * crosscorr[1][0] + y * crosscorr[1][1] + z * crosscorr[1][2]
    z = x * crosscorr[2][0] + y * crosscorr[2][1] + z * crosscorr[2][2]

    return (x,y,z)

class GearVRC(gatt.Device):
    '''
    GATT BLE DEVICE Manager
    provide logger, queue size, VRMode
    '''

    def __init__(self, mac_address, manager, logger = None, VRMode=False, auto_reconnect=False, queuesize=32):

        super(GearVRC, self).__init__(mac_address=mac_address, manager=manager)

        if logger is not None:
            self.logger = logger
        else:
             self.logger = logging.getLogger("Samsung Gear VR Capture")

        self._VRMode = VRMode

        self._auto_reconnect = auto_reconnect

        self._queuesize = queuesize
        # Init vars
        self._previous_axisX  = 0                # touchpad X
        self._previous_axisY  = 0                # touchpad Y
        self._previous_gyroX  = 0.               # Gyroscope
        self._previous_gyroY  = 0.
        self._previous_gyroZ  = 0.
        self._previous_accelX = 0.               # Accelerometer
        self._previous_accelY = 0.
        self._previous_accelZ = 0.
        self._previous_magX   = 0.               # Magnetometer
        self._previous_magY   = 0.
        self._previous_magZ   = 0.

        self._absX = 0
        self._absY = 0

        self._useVol   = False  # We are using volume up/down
        self._useWheel = False  # We are using wheel functionality

        self._lastKeepAlive   = time.perf_counter() # in seconds 
        self._lastTime        = time.perf_counter()
        self._currentTime     = time.perf_counter()
        self._lastBatteryTime = time.perf_counter()

        self._updatecounts    = 0

        self._firstTime       = True

        # NEED TO FIX THIS
        if self._VRMode:
            self._updateTime       = MINIMUPDATETIME
        else:
            self._updateTime       = MINIMUPDATETIME
        
        # Virtual Wheel Stuff
        self._previous_wheelPos = -1
        self._2pi = 2.0*math.pi
        # Wheel regions
        positions = [i for i in range(0, NUMWHEELPOS)]
        # shift by 45 degrees (want to create region for left, right, top , bottom)
        positions = ror(positions, NUMWHEELPOS // 8)
        # make 4 equal length sections
        position_sections = mit.divide(4,positions)
        # assign 4 sections to individual ranges: top, right, bottom, left
        [self._l_right, self._l_top, self._l_left, self._l_bottom] = [list(x) for x in position_sections]

        self._dirUp     = False
        self._dirDown   = False
        self._dirLeft   = False
        self._dirRight  = False
        self._dirCenter = False
        
        # LOCAL Eath Magnetic Field
        self.__wmm       = ahrs.utils.WMM(latitude=TUCSON_LATITUDE, longitude=TUCSON_LONGITUDE, height=TUCSON_HEIGHT)
        self.__mag       = np.array([self.__wmm.X, self.__wmm.Y, self.__wmm.Z])
        # LOCAL Gravity
        self.__gravity   = np.array([0.0, 0.0, ahrs.utils.WGS().normal_gravity(TUCSON_LATITUDE, TUCSON_HEIGHT)])
        # Initial Post Estimator
        self.__tilt = ahrs.filters.Tilt()
        # Pose Estimator
        self.__madgwick  = ahrs.filters.Madgwick()
        # Pose
        self.attitudeQ  = collections.deque([], maxlen=self.__queuesize)         
        
    def connect_succeeded(self):
        '''
        connection successful: gatt-python example
        '''
        super().connect_succeeded()
        self.logger.log(logging.INFO, "Connected to: [{}]".format(self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        self.logger.log(logging.INFO, "Connection to: [{}] failed: {}".format(self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        self.logger.log(logging.INFO, "Disconnected: [{}]".format(self.mac_address))
        if self._auto_reconnect:
            self.connect()
        else: 
            sys.exit(0)

    def write(self, cmd, times):
        for i in range(times):
            self.controller_setup_characteristics.write_value(cmd)

    def services_resolved(self):
        '''
        Initialize Device
        '''
        super().services_resolved()

        #
        # General
        #

        self.general_information_service = next(
            s for s in self.services
            if s.uuid == '00001801-0000-1000-8000-00805f9b34fb')

        #
        # Battery
        #

        self.battery_information_service = next(
            s for s in self.services
            if s.uuid == '0000180f-0000-1000-8000-00805f9b34fb')

        self.battery_level_characteristic = next(
            c for c in self.battery_information_service.characteristics
            if c.uuid == '00002a19-0000-1000-8000-00805f9b34fb')

        #
        # Device Information
        #

        self.device_information_service = next(
            s for s in self.services
            if s.uuid == '0000180a-0000-1000-8000-00805f9b34fb')

        self.model_number_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a24-0000-1000-8000-00805f9b34fb')

        self.serial_number_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a25-0000-1000-8000-00805f9b34fb')

        self.firmware_version_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a26-0000-1000-8000-00805f9b34fb')

        self.hardware_revision_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a27-0000-1000-8000-00805f9b34fb')

        self.software_revision_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a28-0000-1000-8000-00805f9b34fb')

        self.manufacturer_name_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a29-0000-1000-8000-00805f9b34fb')

        self.PnP_ID_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a50-0000-1000-8000-00805f9b34fb')

        #
        # Controller Sensor Data
        #
        # protocol / a.class
        self.controller_data_service = next(
            s for s in self.services
            if s.uuid == '4f63756c-7573-2054-6872-65656d6f7465') # SERVICE

        self.controller_setup_characteristics = next(
            c for c in self.controller_data_service.characteristics
            if c.uuid == 'c8c51726-81bc-483b-a052-f7a14ea3d282') # WRITE

        self.controller_data_characteristic = next(
            c for c in self.controller_data_service.characteristics
            if c.uuid == 'c8c51726-81bc-483b-a052-f7a14ea3d281') # NOTIFY


        # Initialize the device, not sure why command needs to be sent twice
        if self._VRMode:
            self.write(CMD_SENSOR,1)
            self.write(CMD_VR_MODE,1)
        else:
            self.write(CMD_SENSOR,1)
            self.write(CMD_SENSOR,1)

        if self._VRMode:
            self.logger.log(logging.INFO, "Setup done for VR Mode")
        else: 
            self.logger.log(logging.INFO, "Setup done for Sensor Mode)")

        # Start the notification system
        self.controller_data_characteristic.enable_notifications()
        self.battery_level_characteristic.enable_notifications()

        # General
        # self.general_devicename_characteristic.read_value()
        # self.general_appearance_characteristic.read_value()
        # self.general_connectionparameters_characteristic.read_value()
        # Battery
        self.battery_level_characteristic.read_value()
        # Device Information
        self.model_number_characteristic.read_value()
        self.serial_number_characteristic.read_value()
        self.firmware_version_characteristic.read_value()
        self.hardware_revision_characteristic.read_value()
        self.software_revision_characteristic.read_value()
        self.manufacturer_name_characteristic.read_value()
        self.PnP_ID_characteristic.read_value()


    def characteristic_value_updated(self, characteristic, value):
        '''
        On event notification received
        '''

        self._currentTime = time.perf_counter()

        if (self._currentTime - self._lastKeepAlive) >  KEEPALIVEINTERVAL:
            self._lastKeepAlive = self._currentTime
            self.write(CMD_KEEP_ALIVE, 4)

        if (self._currentTime - self._lastBatteryTime) >  BATTERYINTERVAL:
            self._lastBatteryTime = self._currentTime
            self.battery_level_characteristic.read_value()

        # Sensor Data
        if (characteristic == self.controller_data_characteristic):


            # Should receive 60 values
            if (len(value) < 60):
                self.logger.log(logging.ERROR, "Not enought values: {}".format(len(value)))
                return

            self._updatecounts += 1
            self._deltaTime = self._currentTime - self._lastTime
            self._lastTime = self._currentTime
            self.logger.log(logging.INFO, "Update rate: {}".format(self._deltaTime))

            # If update rate too slow make sure we have VR Mode enabled
            if self._deltaTime > self._updateTime: 
                # self.write(CMD_VR_MODE, 2)
                self.logger.log(logging.INFO, "Update rate slow")
                # self._VRMODE = True

            # decode the values
            # com.samsung.android.app.vr.input.service
            # ui package c.class

            # Time
            #
            # There are three time stamps, one is general and one is for touchpad
            # Not sure which belongs to which: 
            #   accelerometer is usually read faster than magnetometer, 
            #   gyroscope and accelerometer are usually in same chip
            self.sensorTime = (struct.unpack('<I', value[0:4])[0] & 0xFFFFFFFF) /1000000.
            self.aTime      = (struct.unpack('<I', value[16:20])[0] & 0xFFFFFFFF) /1000000.
            self.bTime      = (struct.unpack('<I', value[32:36])[0] & 0xFFFFFFFF) /1000000.
            self.logger.log(logging.INFO, "Time: {:10.6f}, {:10.6f}, {:10.6f}".format(self.sensorTime, self.aTime, self.bTime))

            # Touchpad
            #
            # Max observed value = 315
            # 0/0 is not touched!
            # Bottom right largest number
            # Y axis is up down
            # X axis is left right 
            self.axisX     = ((value[54] & 0xF) << 6) + ((value[55] & 0xFC) >> 2) & 0x3FF
            self.axisY     = ((value[55] & 0x3) << 8) + ((value[56] & 0xFF) >> 0) & 0x3FF

            self.logger.log(logging.INFO, "Touchpad Position: {}, {}".format(self.axisX, self.axisY))

            # IMU
            #
            self.accelX = struct.unpack('<h', value[4:6])[0]   * 0.00478840332  # 10000.0 * 9.80665 / 2048 / 10000.0       # m**2/s
            self.accelY = struct.unpack('<h', value[6:8])[0]   * 0.00478840332  #
            self.accelZ = struct.unpack('<h', value[8:10])[0]  * 0.00478840332  #
            self.gyroX  = struct.unpack('<h', value[10:12])[0] * 0.01221791529  # 10000.0 * 0.017453292 / 14.285 / 1000.0  # rad/s
            self.gyroY  = struct.unpack('<h', value[12:14])[0] * 0.01221791529  #
            self.gyroZ  = struct.unpack('<h', value[14:16])[0] * 0.01221791529  #
            self.magX   = struct.unpack('<h', value[48:50])[0] * 0.06           # micro Tesla?, earth mag field 25..65 muTesla
            self.magY   = struct.unpack('<h', value[50:52])[0] * 0.06           #
            self.magZ   = struct.unpack('<h', value[52:54])[0] * 0.06           #
            
            # Apply calibration
            #  We dont have the calibration values yet
            #  We need max/min, offset and cross correlation between the axsis
            #  We can get those for accelerometer and magnetometer by waggling sensor
            #  Gyroscope is difficult to calibrate because we would need to make a rig that rotates with known angular velocity (turn table)
            crosscorr= [[1,0,0], [0,1,0], [0,0,1]]
            self.magX,   self.magY,   self.magZ   = calibrate(self.magX,   self.magY,   self.magZ,   1, 1, 1, 1, 1, 1, 0, 0, 0, crosscorr)
            self.accelX, self.accelY, self.accelZ = calibrate(self.accelX, self.accelY, self.accelZ, 1, 1, 1, 1, 1, 1, 0, 0, 0, crosscorr)

            self.azimuth = math.atan2(self.magY,self.magX)*180./math.pi
            if self.azimuth < 0: self.azimuth +=180.

            self.logger.log(logging.INFO, "Accel {:5.2f} {:5.2f} {:5.2f} ".format(self.accelX,self.accelY,self.accelZ))
            self.logger.log(logging.INFO, "Mag   {:5.2f} {:5.2f} {:5.2f} ".format(self.magX,self.magY,self.magZ))
            self.logger.log(logging.INFO, "Gyro  {:5.2f} {:5.2f} {:5.2f} ".format(self.gyroX,self.gyroY,self.gyroZ))
            self.logger.log(logging.INFO, "Azimuth  {:6.2f}  ".format(self.azimuth))

            self._madgwick.Dt = self._deltaTime
            if self._firstTime:
                # obtain estimate of attitude
                self.previous_attitude = self._tilt.estimate(
                    acc=np.array([self.accelX,self.accelY,self.accelZ]),
                    mag=np.array([self.magX,  self.magY,  self.magZ])
                )
                self._firstTime =  False
            else:
                # compute attitude
                self.attitude = self._madgwick.updateMARG(
                    self.previous_attitude, 
                    acc=np.array([self.accelX,self.accelY,self.accelZ]), 
                    gyr=np.array([self.gyroX, self.gyroY, self.gyroZ]), 
                    mag=np.array([self.magX,  self.magY,  self.magZ]) 
                )
                # [phi, theta, psi] = self.attitude.to_angles()
                self.previous_attidude = self.attitude

            # Temperature
            #
            self.temperature = value[57]
            self.logger.log(logging.INFO, "Temperature: {} ".format(self.temperature))

            # Buttons
            #
            self.triggerButton    = True if ((value[58] &  1) ==  1) else False
            self.homeButton       = True if ((value[58] &  2) ==  2) else False
            self.backButton       = True if ((value[58] &  4) ==  4) else False
            self.touchpadButton   = True if ((value[58] &  8) ==  8) else False
            self.volumeUpButton   = True if ((value[58] & 16) == 16) else False
            self.volumeDownButton = True if ((value[58] & 32) == 32) else False
            self.noButton         = True if ((value[58] & 64) == 64) else False

            # Customization
            # =============
            # Detects if rim of touchpad is touched
            # If wheel mode enabled, detects wheel rotation left, right
            # Detects rim pushed on top, left, right, buttom
            # If center of touchpad is pressed, toggle between wheel or touchpad mode
            # Scrolling touchpad will create absolute "mouse" position, allowing to reach larger field than touchpad allone
            # If volume buttons are pressed, detects if home or back is pushed afterwards
            # If in wheel mode detects if back or home pushed
            # if in touchpad mode detects if back or home pushed

            # Touchpad and Wheel
         
            # Are we touching the wheel zone?
            x = self.axisX - WHEELRADIUS # horizontal distance from center of touchpad
            y = self.axisY - WHEELRADIUS # vertical distance from center
            l2 = x*x + y*y               # distance from center (squared)
            self._outerCircle = True if l2 > RTHRESH2  else False
            if self._outerCircle:
                phi = (math.atan2(y,x) + self._2pi) % self._2pi # angle 0 .. 2*pi from pointing to the right counter clockwise
                self._wheelPos = int(math.floor(phi / self._2pi * NUMWHEELPOS))
                # Top, Bottom, Left, Right
                if self._wheelPos > NUMWHEELPOS1_8:
                    if self._wheelPos < NUMWHEELPOS3_8:
                        self._top    = True
                        self._left   = False
                        self._bottom = False
                        self._right  = False
                    elif self._wheelPos < NUMWHEELPOS5_8:
                        self._top    = False
                        self._left   = True
                        self._bottom = False
                        self._right  = False
                    elif self._wheelPos < NUMWHEELPOS7_8:
                        self._top    = False
                        self._left   = False
                        self._bottom = True
                        self._right  = False
                    else: 
                        self._top    = False
                        self._left   = False
                        self._bottom = False
                        self._right  = True
                self.logger.log(logging.INFO, "Wheel Position: {}".format(self._wheelPos))

            if (self.touchpadButton == True):
                if (self._outerCircle ==  True):
                    if self._top:
                        self.logger.log(logging.INFO, "Wheel Click Top")
                    if self._bottom:
                        self.logger.log(logging.INFO, "Wheel Click Bottom")
                    if self._left:
                        self.logger.log(logging.INFO, "Wheel Click Left")
                    if self._right:
                        self.logger.log(logging.INFO, "Wheel Click Right")
                else:
                    self._useWheel = not self._useWheel # toggle wheel
                    self.logger.log(logging.INFO, "Wheel: {} Touchpad: {}".format("enabled" if self._useWheel else "disabled", "enabled" if not self._useWheel else "disabled"))

            if (self._useWheel):
                self._delta_wheelPos = self._previous_wheelPos - self._wheelPos
                self._previous_wheelPos = self._wheelPos

                # deal with discontinuity 360/0:
                #   general formula with degrees and moving along circle
                #   a0 = a0 - (360. * np.floor((a0 + 180.)/360.))
                #   will exclude 180 and include -180
                # wheel position 0 is 0 degrees, MAXWHEELPOS is almost 360 degrees counter clockwise
                self._delta_wheelPos = self._delta_wheelPos - (MAXWHEELPOS * np.floor((self._delta_wheelPos + MAXWHEELPOS2)/MAXWHEELPOS))

                if (self._delta_wheelPos > 0):
                    # rotating clock wise
                    self.logger.log(logging.INFO, "Wheel rotating counter-clockwise")

                elif (self._delta_wheelPos < 0):
                    # rotating counter clock wise
                    self.logger.log(logging.INFO, "Wheel rotating clockwise")

            # Movement direction on touchpad
            if (not self._useWheel):
                self._delta_X = self.axisX - self._previous_axisX
                self._delta_Y = self.axisY - self._previous_axisY
                self._previous_axisX = self.axisX
                self._previous_axisY = self.axisY
                self.logger.log(logging.INFO, "Delta X {}".format(self._delta_X))
                self.logger.log(logging.INFO, "Delta Y {}".format(self._delta_Y))
                if (abs(self._delta_X) < 50): # disregrad large jumps (sldiing along the surface)
                    self._absX += self._delta_X 
                    self._absX  = clamp(self._absX, MINXTOUCH, MAXXTOUCH)
                    self.logger.log(logging.INFO, "Absolute X {}".format(self._absX))
                    if (self._delta_X > 0):
                        self._dirUp = True
                        self._dirDown = False
                        self.logger.log(logging.INFO, "Touchpad Right")
                    elif (self._delta_X < 0):
                        self._dirDown = True
                        self._dirUp = False
                        self.logger.log(logging.INFO, "Touchpad Left")
                    else:
                        self._dirDown = False
                        self._dirUp = False                        
                if (abs(self._delta_Y) < 50): # disregrad large jumps
                    self._absY += self._delta_Y 
                    self._absY  = clamp(self._absY, MINYTOUCH, MAXYTOUCH)
                    self.logger.log(logging.INFO, "Absolute Y {}".format(self._absY))
                    if (self._delta_Y > 0):
                        self._dirRight = True
                        self._dirLeft = False
                        self.logger.log(logging.INFO, "Touchpad Down")
                    elif (self._delta_Y < 0):
                        self._dirLeft = True
                        self._dirRight = False
                        self.logger.log(logging.INFO, "Touchpad Up")
                    else:
                        self._dirLeft = False
                        self._dirRight = False

            if (self.triggerButton == True):
                self.logger.log(logging.INFO, "Trigger Button: pressed")

            if (self.homeButton == True):
                if (self._useVol == True):
                    self.logger.log(logging.INFO, "Home pushed while chaning Volume, disable Home and Back options")
                    self._useVol = False
                elif ((not self._useWheel) == True):
                    self.logger.log(logging.INFO, "Home pushed while using Touchpad")
                elif (self._useWheel == True):
                    self.logger.log(logging.INFO, "Home pushed while using Wheel, sitching to Touch Pad mode")
                    self._useWheel = False

            if (self.backButton == True):
                if (self._useVol == True):
                    self.logger.log(logging.INFO, "Back pushed while chaning Volume")
                elif ((not self._useWheel) == True):
                    self.logger.log(logging.INFO, "Back pushed while using Touchpad")
                elif (self._useWheel == True):
                    self.logger.log(logging.INFO, "Back pushed while using Wheel")
                else:
                    self.logger.log(logging.INFO, "Back pushed")

            if (self.volumeDownButton == True):
                self._useVol == True
                self.logger.log(logging.INFO, "Volume Down: pushed")

            if (self.volumeUpButton == True):
                self._usevol == True
                self.logger.log(logging.INFO, "Volume Up: pushed")

            if (self.noButton == True):
                pass

        # Device Information
        # elif (characteristic == self.general_devicename_characteristic):
        #     self._devicename = value
        #     self.logger.log(logging.INFO, "Device Name: {}".format(value.decode("utf-8")))
        # elif (characteristic == self.general_appearance_characteristic):
        #     self._appearance = value
        #     self.logger.log(logging.INFO, "Appearance: {}".format(value.decode("utf-8")))
        # elif (characteristic == self.general_connectionparameters_characteristic):
        #     self._connectionparameters = value
        #     self.logger.log(logging.INFO, "Cponnection Paramters: {}".format(value.decode("utf-8")))
        elif (characteristic == self.battery_level_characteristic):
            self._battery = int.from_bytes(value,'big')
            self.logger.log(logging.INFO, "Battery Level: {}".format(self._battery))
        elif (characteristic == self.model_number_characteristic):
            self._model_number = value.decode("utf-8")
            self.logger.log(logging.INFO, "Model Number: {}".format(self._model_number))
        elif (characteristic == self.serial_number_characteristic):
            self._serial_number = value.decode("utf-8")
            self.logger.log(logging.INFO, "Serial Number: {}".format(self._serial_number))
        elif (characteristic == self.firmware_version_characteristic):
            self._firmware_version = int.from_bytes(value,'big')
            self.logger.log(logging.INFO, "Firmware Version: {}".format(self._firmware_version))
        elif (characteristic == self.hardware_revision_characteristic):
            self._hardware_revision = value.decode("utf-8")
            self.logger.log(logging.INFO, "Hardware Revision {}".format(self._hardware_revision))
        elif (characteristic == self.software_revision_characteristic):
            self._software_revision = value.decode("utf-8")
            self.logger.log(logging.INFO, "Software: {}".format(self._software_revision))
        elif (characteristic == self.manufacturer_name_characteristic):
            self._manufacturer_name = value.decode("utf-8")
            self.logger.log(logging.INFO, "Manufacturer: {}".format(self._manufacturer_name ))
        elif (characteristic == self.PnP_ID_characteristic):
            self._pnp_id = int.from_bytes(value,'big')
            self.logger.log(logging.INFO, "PnP ID: {}".format(self._pnp_id ))

        else:
            self.logger.log(logging.INFO, "Reading somethig else {} {}".format(characteristic, len(value)))

def cleanup():
    global gearvrc
    gearvrc.write(CMD_OFF, 2)
    gearvrc._auto_reconnect = False
    gearvrc.disconnect()

#
# Blocks main when running device manager.
# Perhaps we create separate task that communicates with main program either through Queue or ZeroMQ
# Perhaps we look into BLEAK
#

def main():

    import signal
    import logging
    import gatt

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger("Samsung Gear VR Capture")

    device_manager = gatt.DeviceManager(adapter_name='hci0')

    # Control-C handler
    signal.signal(signal.SIGINT, lambda x,y: cleanup()) # Control-C Handler

    logger.log(logging.INFO, "Samsung Gear VR Controller mapper running ...")

    gearvrc = GearVRC(mac_address=MACADDRESS, manager=device_manager, logger=logger, VRMode=True, auto_reconnect=True)

    gearvrc.connect()

    print("Terminate with Ctrl+C")
    try:
        device_manager.run() # Will block for ever
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
