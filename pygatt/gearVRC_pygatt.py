#!/usr/bin/python3

# The library is free.
# MIT License
# Copyright (c) 2019, Robert K. Dady
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

# Based on:
# https://github.com/getsenic/gatt-python
# https://github.com/jsyang/gearvr-controller-webbluetooth
# https://github.com/rdady/gear-vr-controller-linux
# https://github.com/gb2111/Access-GearVR-Controller-from-PC    

# Prerequisite:
# A) python packages:
#   $ sudo pip3 install pygatt more-itertools cmath ahrs
# C) Pair Controller:
#   $ bluetoothlctl
#     scan on
#     pair yourMAC
#     trust yourMAC
#     connect yourMAC

#
# Services in Gear VR Controller
#
# Controller Service
# [2C:BA:BA:2E:17:DB]     Service [4f63756c-7573-2054-6872-65656d6f7465]
# [2C:BA:BA:2E:17:DB]             Characteristic [c8c51726-81bc-483b-a052-f7a14ea3d282] # Write, Settings
# [2C:BA:BA:2E:17:DB]             Characteristic [c8c51726-81bc-483b-a052-f7a14ea3d281] # Notify
#
# Keyboard Information
# [2C:BA:BA:2E:17:DB]     Service [00001879-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a22-0000-1000-8000-00805f9b34fb] # "Boot Keyboard Input Report"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a32-0000-1000-8000-00805f9b34fb] # "Boot Keyboard Output Report"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4a-0000-1000-8000-00805f9b34fb] # "HID Information"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4b-0000-1000-8000-00805f9b34fb] # "Report Map"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4c-0000-1000-8000-00805f9b34fb] # "HID Control Point"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4d-0000-1000-8000-00805f9b34fb] # "Report"
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a4e-0000-1000-8000-00805f9b34fb] # "Protocol Mode"
#
# Device Information
# [2C:BA:BA:2E:17:DB]     Service [0000180a-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a50-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a28-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a26-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a27-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a25-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a24-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a29-0000-1000-8000-00805f9b34fb]
#
# Battery
# [2C:BA:BA:2E:17:DB]     Service [0000180f-0000-1000-8000-00805f9b34fb]
# [2C:BA:BA:2E:17:DB]             Characteristic [00002a19-0000-1000-8000-00805f9b34fb]
#
# Unknown
# [2C:BA:BA:2E:17:DB]     Service [00001801-0000-1000-8000-00805f9b34fb] # Generic Attribute
#                                 Does not have Chracteristic
# Unknown
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

WHEELDIAMETER                    = 315                      # touchpad diameter
NUMWHEELPOS                      = 64                       # simulate rotating wheel, number of positions along perimeter

MINIMUPDATETIME                  = 0.25                     # Minmum interval between sensor data in seconds

KEEPALIVEINTERVAL                = 60                       # Send keep alive signal every minute
BATTERYNTERVAL                   = 60*5                     # Check battery every 5 minutes

# AHRS Location, to compute earth magnetic field and gravity
TUCSON_LATITUDE                  = 32.253460
TUCSON_LONGITUDE                 =-110.911789
TUCSON_HEIGHT                    = 0.759
# Attitude Buffer Length
NUMSAMPLES                       = 2

import pygatt
import signal
import math
import time
import numpy as np
import sys
import os
import cmath
import more_itertools as mit
import logging
import ahrs
import collections

ADDRESSTYPE = pygatt.BLEAddressType.random

def ror(l, n):
    '''
    split list at position -n and attach right section in front left section
    '''
    return l[-n:] + l[:-n]

class GearVRC():

    '''
    GATT BLE DEVICE Manager
    provide logger, queue size, VRMode
    '''

    def __init__(self, logger = None, 
        queue_size: int = NUMSAMPLES, VRMode=False):

        if logger is not None:
            self.logger = logger
        else:
             self.logger = logging.getLogger("Samsung Gear VR Capture")

        self.__VRMODE = VRMode

        # Init vars
        self.__max = WHEELDIAMETER      # diameter of touch pad
        self.__r = self.__max / 2       # radius of touch pad
        self.__axisX = 0                # touchpad X
        self.__axisY = 0                # touchbad Y
        self.__altX  = 0
        self.__altY  = 0

        self.__reset  = True
        self.__volbtn = True
        self.__tchbtn = True 
        self.__trig   = True

        self.__lastKeepAlive   = time.perf_counter() # in seconds 
        self.__lastTime         = time.perf_counter()
        self.__currentTime      = time.perf_counter()
        self.__lastBatteryTime  = time.perf_counter()

        self.__updatecounts     = 0
        if self.__VRMode:
            self.__updateTime       = MINIMUPDATETIME
        else:
            self.__updateTime       = MINIMUPDATETIME
        

        self.__wheelPos = -1
        [self.__l_top, self.__l_right, self.__l_bottom, self.__l_left] = [list(x) for x in mit.divide(4, ror([i for i in range(0, NUMWHEELPOS)], NUMWHEELPOS // 8))]
        self.__wheelMultiplier = 2

        self.__useWheel = False
        self.__useTouch = False
        self.__dirUp = False
        self.__dirDown = False
        self.__VR = False

        # make sure there is karge ebnough buffer for all characterstics
        self.__adapter = pygatt.GATTToolBackend(search_window_size=4096)

        # LOCAL Eath Magnetic Field
        self.__wmm       = ahrs.utils.WMM(latitude=TUCSON_LATITUDE, longitude=TUCSON_LONGITUDE, height=TUCSON_HEIGHT)
        self.__mag       = np.array([self.__wmm.X, self.__wmm.Y, self.__wmm.Z])
        # LOCAL Gravity
        self.__gravity   = np.array([0.0, 0.0, ahrs.utils.WGS().normal_gravity(TUCSON_LATITUDE, TUCSON_HEIGHT)])
        # Pose Estimator
        self.__madgwick  = ahrs.filters.Madgwick()
        # Pose
        self.attitudeQ  = collections.deueue([], maxlen=queue_size) 

    def connect(self):
        try:
            self.__adapter.start()
            self.__device = self.__adapter.connect(MACADDRESS, address_type=ADDRESS_TYPE)
        except:


    def connect_succeeded(self):
        '''
        connection successfull
        '''
        super().connect_succeeded()
        self.logger.log(logging.INFO, "Connected to: [{}]".format(self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        self.logger.log(logging.INFO, "Connecttion to: [{}] failed: {}".format(self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        self.logger.log(logging.INFO, "Disconnected: [{}]".format(self.mac_address))
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

        self.general_devicename_characteristic = next(
            c for c in self.general_information_service.characteristics
            if c.uuid == '00002a00-0000-1000-8000-00805f9b34fb')

        self.general_apperance_characteristic = next(
            c for c in self.general_information_service.characteristics
            if c.uuid == '00002a01-0000-1000-8000-00805f9b34fb')

        self.general_connectionparameters_characteristic = next(
            c for c in self.general_information_service.characteristics
            if c.uuid == '00002a04-0000-1000-8000-00805f9b34fb')

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
            if c.uuid == '00002a27-0000-1000-8000-00805f9b34fb')

        self.PnP_ID_characteristic = next(
            c for c in self.device_information_service.characteristics
            if c.uuid == '00002a50-0000-1000-8000-00805f9b34fb')

        #
        # Controller Sensor Data
        #

        self.controller_data_service = next(
            s for s in self.services
            if s.uuid == '4f63756c-7573-2054-6872-65656d6f7465') # SERVICE

        self.controller_setup_characteristics = next(
            c for c in self.controller_data_service.characteristics
            if c.uuid == 'c8c51726-81bc-483b-a052-f7a14ea3d282') # WRITE

        self.controller_data_characteristic = next(
            c for c in self.controller_data_service.characteristics
            if c.uuid == 'c8c51726-81bc-483b-a052-f7a14ea3d281') # NOTIFY

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

        # Initialzie the device, not sure why command needs to be sent twice
        if self.__VRMODE:
            self.write(CMD_VR_MODE, 2)
        else:
            self.write(CMD_SENSOR, 2)

        # self.write(CMD_LPM_ENABLE, 1)
        # self.write(CMD_LPM_DISABLE, 1)

        # Start the notification system
        self.controller_data_characteristic.enable_notifications()

        if self.__VRMODE:
            self.logger.log(logging.INFO, "Setup done (VR Mode)")
        else: 
            self.logger.log(logging.INFO, "Setup done (Sensor Mode)")


    def characteristic_value_updated(self, characteristic, value):
        '''
        On event notification received
        '''

        # Sensor Data
        if (characteristic == self.controller_data_characteristic):

            int_values = [x for x in value]

            # Should receive 60 values, slow down, then exit
            if (len(int_values) < 60):
                if self.__VR == True:
                    self.write(CMD_SENSOR, 2)
                    self.controller_data_characteristic.enable_notifications()
                    self.__VR == True
                    self.logger.log(logging.INFO, "Sensor mode is activated")
                return

            self.__updatecounts += 1
            self.__currentTime = time.perf_counter()
            self.__deltatime = self.__currentTime - self.__lastTime
            self.__lastTime = self.__currentTime

            # If update rate too slow make sure we have VR Mode enabled
            if self.__deltaTime > self.__updateTime: 
                self.write(CMD_VR_MODE, 2)
                self.logger.log(logging.INFO, "VR mode enabled: {}".format(self.__deltatime))
                self.__VR = True

            if (self.__currentTime - self.__lastKeepAlive) >  KEEPALIVEINTERVAL:
                self.__lastKeepAlive = self.__currentTime
                # need to write 4 times ???
                self.write(self, CMD_KEEP_ALIVE, 4)

            if (self.__currentTime - self.__last_batteryTime) >  BATTERYINTERVAL:
                self.__last_batteryTime = self.__currentTime
                self.battery_level_characteristic.read_value()

            # Touchpad
            #
            # Max observed value = 315
            # (corresponds to touchpad sensitive dimension in mm)
            axisX   = (((int_values[54] & 0xF)  << 6) + ((int_values[55] & 0xFC) >> 2)) & 0x3FF           # as in reference hack
            axisY   = (((int_values[55] & 0x3)  << 8) + ((int_values[56] & 0xFF) >> 0)) & 0x3FF           # as in reference hack

            # IMU
            #
            accelX = np.uint16((int_values[4]  << 8) + int_values[5])  * 10000.0 * 9.80665 / 2048.0      # m**2/s
            accelY = np.uint16((int_values[6]  << 8) + int_values[7])  * 10000.0 * 9.80665 / 2048.0
            accelZ = np.uint16((int_values[8]  << 8) + int_values[9])  * 10000.0 * 9.80665 / 2048.0
            gyroX  = np.uint16((int_values[10] << 8) + int_values[11]) * 10000.0 * 0.017453292 / 14.285  # rad/s
            gyroY  = np.uint16((int_values[12] << 8) + int_values[13]) * 10000.0 * 0.017453292 / 14.285
            gyroZ  = np.uint16((int_values[14] << 8) + int_values[15]) * 10000.0 * 0.017453292 / 14.285
            magX   = np.uint16((int_values[32] << 8) + int_values[33]) * 0.06                            # micro Tesla?, earth mag field 25..65 muTesla
            magY   = np.uint16((int_values[34] << 8) + int_values[35]) * 0.06
            magZ   = np.uint16((int_values[36] << 8) + int_values[37]) * 0.06

            # gyro_data rad/s
            # acc_data m/s^2
            # mag in mT
            self.__madgwick.Dt = self.__deltaTime
            self._attitude.append(self.__madgwick(
                                    self.attitude[-1], 
                                    acc=np.array([accelX,accelY,accelZ]), 
                                    gyr=np.array([gyroX,gyroY,gyroZ]), 
                                    mag=np.array([magX,magY,magZ]) )
            )
            roll_pitch_yaw = self.__attitude[-1].to_angles()

            # Temperature
            #
            temperature = int_values[57]

            # Buttons
            #
            triggerButton    = True if ((int_values[58] &  1) ==  1) else False
            homeButton       = True if ((int_values[58] &  2) ==  2) else False
            backButton       = True if ((int_values[58] &  4) ==  4) else False
            touchpadButton   = True if ((int_values[58] &  8) ==  8) else False
            volumeUpButton   = True if ((int_values[58] & 16) == 16) else False
            volumeDownButton = True if ((int_values[58] & 32) == 32) else False
            NoButton         = True if ((int_values[58] & 64) == 64) else False

            # Customization: Interpretation of input
            #
            idelta = 30
            odelta = 25

            if (touchpadButton == True and self.__trig == True):
                self.__useWheel = not self.__useWheel
                #self.__useTouch = not self.__useTouch
                self.__trig = False
            elif (touchpadButton == False and self.__trig == False):
                self.__trig = True

            outerCircle = True if (axisX - self.__r)**2 + (axisY - self.__r)**2 > (self.__r - odelta)**2  else False
            wheelPos = self.wheelPos(axisX, axisY)
            T = True if (outerCircle and int(wheelPos) in self.__l_top)    else False # Top
            R = True if (outerCircle and int(wheelPos) in self.__l_right)  else False # Right
            B = True if (outerCircle and int(wheelPos) in self.__l_bottom) else False # Bottom
            L = True if (outerCircle and int(wheelPos) in self.__l_left)   else False # Left

            delta_X = delta_Y = 0
            delta_X = axisX - self.__axisX
            delta_Y = axisY - self.__axisY
            delta_X = round(delta_X * 1.2)
            delta_Y = round(delta_Y * 1.2)

            if (self.__useWheel):
                if (abs(self.__wheelPos - wheelPos) > 1 and abs((self.__wheelPos + 1) % NUMWHEELPOS - (wheelPos + 1) % NUMWHEELPOS) > 1):
                    self.__wheelPos = wheelPos
                    return
                if ((self.__wheelPos - wheelPos) == 1 or ((self.__wheelPos + 1) % NUMWHEELPOS - (wheelPos + 1) % NUMWHEELPOS) == 1):
                    self.__wheelPos = wheelPos
                    for i in range(self.__wheelMultiplier):
                        # self.__device.emit(uinput.KEY_UP, 1)
                        # self.__device.emit(uinput.KEY_UP, 0)
                        pass
                    return
                if ((wheelPos - self.__wheelPos) == 1 or ((wheelPos + 1) % NUMWHEELPOS - (self.__wheelPos + 1) % NUMWHEELPOS) == 1):
                    self.__wheelPos = wheelPos
                    for i in range(self.__wheelMultiplier):
                        # self.__device.emit(uinput.KEY_DOWN, 1)
                        # self.__device.emit(uinput.KEY_DOWN, 0)
                        pass
                    return
                return

            if (self.__useTouch):
                if (abs(delta_X) < 50):
                    if (axisX == 0 and axisY == 0):
                        self.__dirUp = False
                        self.__dirDown = False
                        self.__axisX = axisX
                        self.__axisY = axisY
                        return
                    elif (self.__dirUp == False and self.__dirDown == False):
                        if (delta_X > 0):
                            self.__dirUp = True
                        else:
                            self.__dirDown = True
                    if (self.__dirUp == True and abs(delta_X) > 1):
                        # self.__device.emit(uinput.KEY_UP, 1)
                        # self.__device.emit(uinput.KEY_UP, 0)
                        pass
                    elif (self.__dirDown == True and abs(delta_X) > 1):
                        # self.__device.emit(uinput.KEY_DOWN, 1)
                        # self.__device.emit(uinput.KEY_DOWN, 0)
                        pass
                self.__axisX = axisX
                self.__axisY = axisY
                logger.log(logging.INFO, "Delta X {}".format(delta_X))
                return

            if (triggerButton == True):
                self.logger.log(logging.INFO, "Trigger Button: 1")
            else:
                self.logger.log(logging.INFO, "Trigger Button: 0")

            if (homeButton == True and self.__volbtn == True):
                # self.__device.emit(uinput.KEY_LEFTALT, 1)
                # self.__device.emit(uinput.KEY_HOME, 1)
                # self.__device.emit(uinput.KEY_HOME, 0)
                # self.__device.emit(uinput.KEY_LEFTALT, 0)
                self.__volbtn = False
                return

            if (backButton == True and self.__volbtn == True):
                # self.__device.emit(uinput.KEY_LEFTALT, 1)
                # self.__device.emit(uinput.KEY_LEFT, 1)
                # self.__device.emit(uinput.KEY_LEFT, 0)
                # self.__device.emit(uinput.KEY_LEFTALT, 0)
                self.__volbtn = False
                return

            if (volumeDownButton == True and self.__volbtn == True):
                # self.__device.emit(uinput.KEY_LEFTCTRL, 1, syn = False)
                # self.__device.emit(uinput.KEY_KP0, 1, syn = True)
                # self.__device.emit(uinput.KEY_KP0, 0, syn = False)
                # self.__device.emit(uinput.KEY_LEFTCTRL, 0, syn = True)
                self.__volbtn = False
                pass
                return

            if (volumeUpButton == True and self.__volbtn == True):
                self.__volbtn = False
                # self.__device.emit(uinput.KEY_LEFTCTRL, 1, syn = False)
                # self.__device.emit(uinput.KEY_KPPLUS, 1, syn = True)
                # self.__device.emit(uinput.KEY_KPPLUS, 0, syn = False)
                # self.__device.emit(uinput.KEY_LEFTCTRL, 0, syn = True)
                return

            if (NoButton == True):
                self.__volbtn = True
                self.__tchbtn = True
                self.__trig   = True
                # self.__device.emit(uinput.BTN_LEFT, 0)
                # self.__device.emit(uinput.KEY_PAGEUP, 0)
                # self.__device.emit(uinput.KEY_PAGEDOWN, 0)
                # self.__device.emit(uinput.KEY_UP, 0)
                # self.__device.emit(uinput.KEY_DOWN, 0)
                # self.__device.emit(uinput.KEY_LEFT, 0)
                # self.__device.emit(uinput.KEY_RIGHT, 0)

            # No standalone button handling behind this point

            if (axisX == 0 and axisY == 0):
                self.__reset = True
                return

            if (self.__reset == True):
                self.__reset = False
                self.__axisX = axisX
                self.__axisY = axisY
                self.__altX = gyroX
                self.__altY = gyroY
                return

            self.movePointerREL(delta_X, delta_Y)

            self.__axisX = axisX
            self.__axisY = axisY
            self.__altX = gyroX
            self.__altY = gyroY

        # Device Information

        elif (characteristic == self.battery_level_characteristic):
            self.__battery = value
            logger.log(logging.INFO, "Battery Level: {}".format(value.decode("utf-8")))
        elif (characteristic == self.model_number_characteristic):
            self.__model_number = value
            logger.log(logging.INFO, "Model Number: {}".format(value.decode("utf-8")))
        elif (characteristic == self.serial_number_characteristic):
            self.__serial_number = value
            logger.log(logging.INFO, "Serial Number: {}".format(value.decode("utf-8")))
        elif (characteristic == self.firmware_version_characteristic):
            self.__frimware_version = value
            logger.log(logging.INFO, "Firmware Version: {}".format(value.decode("utf-8")))
        elif (characteristic == self.hardware_revision_characteristic):
            self.__hardware_revision = value
            logger.log(logging.INFO, "Hardware Revision {}".format(value.decode("utf-8")))
        elif (characteristic == self.software_revision_characteristic):
            self.__software_revision = value
            logger.log(logging.INFO, "Software: {}".format(value.decode("utf-8")))
        elif (characteristic == self.manufacturer_name_characteristic):
            self.__manufacturer_name = value
            logger.log(logging.INFO, "Manufacturer: {}".format(value.decode("utf-8")))
        elif (characteristic == self.PnP_ID_characteristic):
            self.__pnp_id = value
            logger.log(logging.INFO, "PnP ID: {}".format(value.decode("utf-8")))

        else:
            self.logger.log(logging.INFO, "Reading somethig else {} {}".format(characteristic, len(value)))

    def movePointerREL(self, dx, dy):
        incx = 0 if dx == 0 else round((0 - dx)/abs(dx))
        incy = 0 if dy == 0 else round((0 - dy)/abs(dy))

        while (dx != 0 or dy != 0):
            if (dx != 0):
                # self.__device.emit(uinput.REL_X, -incx, syn = True)
                dx += incx
            if (dy != 0):
                # self.__device.emit(uinput.REL_Y, -incy, syn = True)
                dy += incy

    # circle segments from 0 .. NUMWHEELPOS clockwise
    def wheelPos(self, x, y):
        pos = 0
        if (x == 0 and y == 0):
            pos = -1
        r, phi = cmath.polar(complex(x-157, y-157))
        pos = math.floor(math.degrees(phi) / 360 * NUMWHEELPOS)
        return pos

def defint():
    global device
    device.write(CMD_OFF, 2)
    device.disconnect()
    sys.exit(0)


logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger("Samsung Gear VR Capture")

manager = gatt.DeviceManager(adapter_name='hci0')

signal.signal(signal.SIGINT, lambda x,y: defint())

logger.log(logging.INFO, "Samsung Gear VR Controller mapper running ...")

device = AnyDevice(mac_address=MACADDRESS, manager=manager, logger=logger)
device.connect()
manager.run()

