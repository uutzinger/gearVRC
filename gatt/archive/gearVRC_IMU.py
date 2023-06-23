#!/usr/bin/python3

# The library is free.
# MIT License
# Copyright (c) 2019, Robert K. Dady
# Copyright (c) 2023, Urs Utzinger
# Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

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
import ahrs
import collections
import logging
import struct

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

        self.__VRMode = VRMode

        self.__auto_reconnect = auto_reconnect

        self.__queuesize = queuesize
        # Init vars
        self.__previous_axisX  = 0                # touchpad X
        self.__previous_axisY  = 0                # touchbad Y

        self.__absX = 0
        self.__absY = 0

        self.__useVol   = False  # We are using volume up/down
        self.__useWheel = False  # We are using wheel functionality

        self.__lastKeepAlive   = time.perf_counter() # in seconds 
        self.__lastTime        = time.perf_counter()
        self.__currentTime     = time.perf_counter()
        self.__lastBatteryTime = time.perf_counter()

        self.__updatecounts    = 0

        self.__firstTime       = True

        # NEED TO FIX THIS
        if self.__VRMode:
            self.__updateTime       = MINIMUPDATETIME
        else:
            self.__updateTime       = MINIMUPDATETIME
        
        # Virtual Wheel Stuff
        self.__previous_wheelPos = -1
        self.__2pi = 2.0*math.pi
        # Wheel regions
        positions = [i for i in range(0, NUMWHEELPOS)]
        # shift by 45 degrees (want to create region for left, right, top , bottom)
        positions = ror(positions, NUMWHEELPOS // 8)
        # make 4 equal length sections
        position_sections = mit.divide(4,positions)

        self.__dirUp     = False
        self.__dirDown   = False
        self.__dirLeft   = False
        self.__dirRight  = False

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
        connection successfull: gatt-python example
        '''
        super().connect_succeeded()
        self.logger.log(logging.INFO, "Connected to: [{}]".format(self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        self.logger.log(logging.INFO, "Connecttion to: [{}] failed: {}".format(self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        self.logger.log(logging.INFO, "Disconnected: [{}]".format(self.mac_address))
        if self.__auto_reconnect:
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


        # Initialzie the device
        if self.__VRMode:
            self.write(CMD_SENSOR,1)
            self.write(CMD_VR_MODE,1)
        else:
            self.write(CMD_SENSOR,1)
            self.write(CMD_SENSOR,1)

        if self.__VRMode:
            self.logger.log(logging.INFO, "Setup done for VR Mode")
        else: 
            self.logger.log(logging.INFO, "Setup done for Sensor Mode)")

        # Start the notification system
        self.controller_data_characteristic.enable_notifications()

    def characteristic_value_updated(self, characteristic, value):
        '''
        On event notification received
        '''

        self.__currentTime = time.perf_counter()

        if (self.__currentTime - self.__lastKeepAlive) >  KEEPALIVEINTERVAL:
            self.__lastKeepAlive = self.__currentTime
            self.write(CMD_KEEP_ALIVE, 4)

        if (self.__currentTime - self.__lastBatteryTime) >  BATTERYINTERVAL:
            self.__lastBatteryTime = self.__currentTime
            self.battery_level_characteristic.read_value()

        # Sensor Data
        if (characteristic == self.controller_data_characteristic):

            # Should recevie 60 bytes
            if (len(value) < 60):
                # if self.__VRMODE == True:
                #     self.write(CMD_SENSOR, 2)
                #     self.controller_data_characteristic.enable_notifications()
                #     self.__VRMODE == False
                self.logger.log(logging.ERROR, "Not enought values: {}".format(len(value)))
                return

            self.__updatecounts += 1
            self.__deltaTime = self.__currentTime - self.__lastTime
            self.__lastTime = self.__currentTime

            # If update rate too slow make sure we have VR Mode enabled
            if self.__deltaTime > self.__updateTime: 
                # self.write(CMD_VR_MODE, 2)
                self.logger.log(logging.INFO, "Update rate slow")
                # self.__VRMODE = True

            # Value [0..59]

            # mag
            # d = (k*16 + 0,1) *0.06
            # e = (k*16 + 2,3) *0.06
            # f = (k*16 + 4,5) *0.06
            # azimuth:
            # phi=atan2(e,d)*180/3.141
            # if phi < 0: phi=phi+180
            #

            #
            # Temperature
            # c = v[k*16 + 6 + 3]

            # Touchpad
            # k = 3
            # flag   = (byte)  (v[k*16+6]   & 0xF0) >> 4
            # touchX = (short)((v[k*16+6]   & 0xF ) << 6 + (v[k*16+6+1] & 0xFC) >> 2) & 0x3FF
            # touchY = (short)((v[k*16+6+1] & 0x3 ) << 8 + (v[k*16+6+2] & 0xFF) >> 0) & 0x3FF
            # if flag == 2: flag = 0
            # if flag==1
            #   touchX = touchX
            #   touchY = touchY
            # else   
            #   touchX = 0
            #   touchY = 0

            # a = p*16 + 0,1,2,3
            #
            # accel p=0
            # e = p*16 + 4,5
            # f = p*16 + 6,7
            # g = p*16 + 8,9
            # l=length(e,f,g)
            # x-angle=asin(e/l*57.29578)*-1
            # y-angle=asin(f/l*57.29578)*-1
            # z-angle=asin(g/l*57.29578)-90
            #
            # gyro, p=0
            # b = p*16 + 10,11
            # c = p*16 + 12,13
            # d = p*16 + 14,15
            
            # Time
            self.sensorTime = (struct.unpack('<I', value[0:4])[0] & 0xFFFFFFFF) /1000000.
            self.aTime      = (struct.unpack('<I', value[16:20])[0] & 0xFFFFFFFF) /1000000.
            self.bTime      = (struct.unpack('<I', value[32:36])[0] & 0xFFFFFFFF) /1000000.

            # Touchpad
            #
            # Max observed value = 315
            # largest values bottom right
            self.touchflag = ((value[54] & 0xF0) >> 4)
            if self.touchflag == 1:
                self.axisX     = ((value[54] & 0xF) << 6) + ((value[55] & 0xFC) >> 2) & 0x3FF
                self.axisY     = ((value[55] & 0x3) << 8) + ((value[56] & 0xFF) >> 0) & 0x3FF
            else:
                self.axisX = 0
                self.axisY = 0

            # IMU
            self.accelX = struct.unpack('<h', value[4:6])[0]   * 9.80665 / 2048        # m**2/s
            self.accelY = struct.unpack('<h', value[6:8])[0]   * 9.80665 / 2048
            self.accelZ = struct.unpack('<h', value[8:10])[0]  * 9.80665 / 2048
            self.gyroX  = struct.unpack('<h', value[10:12])[0] * 10. * 0.017453292 / 14.285  # rad/s
            self.gyroY  = struct.unpack('<h', value[12:14])[0] * 10. * 0.017453292 / 14.285  
            self.gyroZ  = struct.unpack('<h', value[14:16])[0] * 10. * 0.017453292 / 14.285 
            self.magX   = struct.unpack('<h', value[48:50])[0] * 0.06 # micro Tesla?, earth mag field 25..65 muTesla
            self.magY   = struct.unpack('<h', value[50:52])[0] * 0.06 #
            self.magZ   = struct.unpack('<h', value[52:54])[0] * 0.06 #
            self.azimuth = math.atan2(self.magY,self.magX)*180./math.pi
            if self.azimuth < 0: self.azimuth +=180.

            # Temperature
            # Single byte
            self.temperature = value[57]

            # Buttons
            #
            # Bit definition 0 = button up, 1 = button down (pressed)
            # Bit 0 - Trigger
            # Bit 1 - Home
            # Bit 2 - Back
            # Bit 3 - Track (Touch)
            # Bit 4 - Volume up
            # Bit 5 - Volume Down
            self.triggerButton    = True if ((value[58] &  1) ==  1) else False
            self.homeButton       = True if ((value[58] &  2) ==  2) else False
            self.backButton       = True if ((value[58] &  4) ==  4) else False
            self.touchpadButton   = True if ((value[58] &  8) ==  8) else False
            self.volumeUpButton   = True if ((value[58] & 16) == 16) else False
            self.volumeDownButton = True if ((value[58] & 32) == 32) else False
            self.noButton         = True if ((value[58] & 64) == 64) else False


            self.logger.log(logging.INFO, '\033[2J') # clear screen
            self.logger.log(logging.INFO, "Update rate: {}".format(self.__deltaTime))
            self.logger.log(logging.INFO, "Time: {}, {}, {}".format(self.sensorTime, self.aTime, self.bTime))
            self.logger.log(logging.INFO, "Touchpad Position: {}, {}".format(self.axisX, self.axisY))
            self.logger.log(logging.INFO, "Temperature: {} ".format(self.temperature))
            self.acc=math.sqrt(self.accelX*self.accelX+self.accelY*self.accelY+self.accelZ*self.accelZ)
            self.gyro=math.sqrt(self.gyroX*self.gyroX+self.gyroY*self.gyroY+self.gyroZ*self.gyroZ)
            self.mag=math.sqrt(self.magX*self.magX+self.magY*self.magY+self.magZ*self.magZ)
            self.logger.log(logging.INFO, "Accel {:8.2f} {:8.2f} {:8.2f} {:8.1f}".format(self.accelX,self.accelY,self.accelZ, self.acc))
            self.logger.log(logging.INFO, "Mag   {:8.2f} {:8.2f} {:8.2f} {:8.1f}".format(self.magX,self.magY,self.magZ, self.mag))
            self.logger.log(logging.INFO, "Gyro  {:8.2f} {:8.2f} {:8.2f} {:8.1f}".format(self.gyroX,self.gyroY,self.gyroZ, self.gyro))
            self.logger.log(logging.INFO, "Buttons: {} {} {} {} {} {}".format(1 if self.triggerButton else 0,
                                                                     1 if self.homeButton else 0,
                                                                     1 if self.backButton else 0,
                                                                     1 if self.touchpadButton else 0,
                                                                     1 if self.volumeUpButton else 0,
                                                                     1 if self.volumeDownButton else 0)
                            )
            self.logger.log(logging.INFO, "UNKN20: {:3d} {:3d} {:3d} {:3d}".format(value[20],value[21],value[22],value[23])) # LSB/MSB, LSB/MSB
            self.logger.log(logging.INFO, "UNKN24: {:3d} {:3d} {:3d} {:3d}".format(value[24],value[25],value[26],value[27])) # LSB/MSB, LSB/MSB
            self.logger.log(logging.INFO, "UNKN28: {:3d} {:3d} {:3d} {:3d}".format(value[28],value[29],value[30],value[31])) # LSB/MSB, LSB/MSB

            self.logger.log(logging.INFO, "UNKN36: {:3d} {:3d} {:3d} {:3d}".format(value[36],value[37],value[38],value[39])) # LSB/MSB, LSB/MSB
            self.logger.log(logging.INFO, "UNKN40: {:3d} {:3d} {:3d} {:3d}".format(value[40],value[41],value[42],value[43])) # LSB/MSB, LSB/MSB
            self.logger.log(logging.INFO, "UNKN44: {:3d} {:3d} {:3d} {:3d}".format(value[44],value[45],value[46],value[47])) # LSB/MSB, LSB/MSB
            self.logger.log(logging.INFO, "UNKN56: {:3d} {:3d} {:3d} {:3d}".format(0,0,0,value[59]))
            # any value close to 0 or 255 are MSB
            # time is 4 bytes / 1000
            # self.timestamp = ((value[3]  >> 4) + (value[2]>> 2)            # 
            # self.logger.log(logging.INFO, '\033[8A') # move 8 lines up

        else:
            self.logger.log(logging.INFO, "Reading somethig else {} {}".format(characteristic, len(value)))

def cleanup():
    global gearvrc
    gearvrc.write(CMD_OFF, 2)
    gearvrc.__auto_reconnect = False
    gearvrc.disconnect()

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