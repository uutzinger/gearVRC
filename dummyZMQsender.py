#!/usr/bin/python3

################################################################
# Dummy ZMQ Sender
################################################################
            
# IMPORTS
################################################################
import logging
import argparse
import signal
import random
import time
import os
import math
import asyncio
import zmq
import msgpack

from pyIMU.quaternion import Vector3D, Quaternion, TWOPI, RAD2DEG
from pyIMU.utilities import q2rpy, qmag2h, vector_angle2q
from archive.gearVRC import gearIMUData, gearButtonData, gearFusionData, gearMotionData, obj2dict            
from copy import copy

timerResolution = time.get_clock_info('monotonic').resolution

REPORTINTERVAL = 1./10.  - timerResolution/4.

################################################################
# Dummy
################################################################

class dummyZMQ:
            
    def __init__(self, logger=None) -> None:

        # Signals
        self.dataAvailable          = False
        self.finish_up              = False
        
        if logger is not None: self.logger = logger
        else:                  self.logger = logging.getLogger('gearVRC')

        # device sensors
        self.sensorTime             = 0.    # There are 3 different times transmitted from the sensor
        self.previous_sensorTime    = 0.

        # IMU

        # Timing
        ###################
        self.data_deltaTime         = 0.
        self.data_rate              = 0
        self.data_updateInterval    = 1./args.rate - timerResolution/8.
        self.targetRate             = 1./args.rate 

        self.zmq_deltaTime          = 0.
        self.zmq_rate               = 0

        self.report_deltaTime       = 0.
        self.report_rate            = 0
        self.report_updateInterval  = REPORTINTERVAL 
        
        self.acc           = Vector3D(0.,0.,0.)
        self.gyr           = Vector3D(0.,0.,0.)
        self.mag           = Vector3D(0.,0.,0.)
        
        self.position      = Vector3D(0.,0.,0.)
        self.dx = (random.random()*2.-1.)
        self.dy = (random.random()*2.-1.)
        self.dz = (random.random()*2.-1.)
        scale = math.sqrt(self.dx*self.dx + self.dy*self.dy + self.dz*self.dz)
        self.dx /= scale*1000.
        self.dy /= scale*1000.
        self.dz /= scale*1000.
        
    async def update(self):
        '''
        Generate normalized radom IMU data
        '''

        self.logger.log(logging.INFO, 'Starting ...')

        data_lastTimeRate   = time.perf_counter()
        previous_dataUpdate = time.perf_counter()
        data_updateCounts   = 0

        report_lastTimeRate = time.perf_counter()
        previous_reportUpdate = time.perf_counter()
        report_updateCounts = 0

        zmq_lastTimeRate   = time.perf_counter()
        previous_zmqUpdate = time.perf_counter()
        zmq_updateCounts   = 0
        
        context = zmq.Context()      
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:{}".format(args.zmqport))
        
        data_IMU    = gearIMUData()
        data_fusion = gearFusionData()
        data_button = gearButtonData()
        data_motion = gearMotionData()
        
        self.trigger     = False
        self.touch       = False
        self.back        = False
        self.home        = False
        self.volume_up   = False
        self.volume_down = False
        self.noButton    = True
        self.touchX      = 0
        self.touchY      = 0
        
        rot = 0.0
        counter = 0 
        lastButtonTime = time.perf_counter()
        
        while not self.finish_up:

            currentTime = time.perf_counter()
            counter += 1            
            self.data_deltaTime = currentTime - previous_dataUpdate
            previous_dataUpdate = copy(currentTime)

            data_updateCounts += 1
            if (currentTime - data_lastTimeRate)>= 1.:
                self.data_rate = copy(data_updateCounts)
                data_lastTimeRate = copy(currentTime)
                data_updateCounts = 0

            accX = random.random()*2.-1.
            accY = random.random()*2.-1.
            accZ = random.random()*2.-1.
            gyrX = random.random()*2.-1.
            gyrY = random.random()*2.-1.
            gyrZ = random.random()*2.-1.
            magX = random.random()*2.-1.
            magY = random.random()*2.-1.
            magZ = random.random()*2.-1.
            
            self.sensorTime = copy(currentTime)
            self.acc = Vector3D(accX, accY, accZ)
            self.gyr = Vector3D(gyrX, gyrY, gyrZ)
            self.mag = Vector3D(magX, magY, magZ)
            
            self.acc.normalize()
            self.gyr.normalize()
            self.mag.normalize()
            
            self.acc *=9.81
            self.gyr *=5.
            self.mag *=40.

            if not args.static:
                if (time.perf_counter() - lastButtonTime) > 0.5:
                    lastButtonTime = copy(currentTime)

                    if random.choice([True, False]):
                        self.trigger   = random.choice([True, False])
                        self.touch     = random.choice([True, False])
                        self.back      = random.choice([True, False])
                        self.home      = random.choice([True, False])
                        self.volume_up = random.choice([True, False])
                        self.volume_down = random.choice([True, False])
                        self.noButton  = False
                    else:
                        self.trigger     = False
                        self.touch       = False
                        self.back        = False
                        self.home        = False
                        self.volume_up   = False
                        self.volume_down = False
                        self.noButton    = True

                    if random.choice([True, True]): # if random.choice([True, False]):
                        radius =  315/2
                        angle = random.uniform(0, 2 * math.pi)
                        distance = random.uniform(0, radius)
                        x = radius + distance * math.cos(angle)
                        y = radius + distance * math.sin(angle)
                        self.touchX = int(x)
                        self.touchY = int(y)
                    else:
                        self.touchX = 0
                        self.touchY = 0
            else:
                # static turn on all buttons and touchpad center
                self.trigger     = True
                self.touch       = False
                self.back        = True
                self.home        = True
                self.volume_up   = True
                self.volume_down = True
                self.noButton    = False
                self.touchX      = 315 // 2
                self.touchY      = 315 // 2

            rot = rot + 0.01
            if rot > 2. * math.pi: rot = 0.
            self.q=vector_angle2q(vec=Vector3D(-1.,1.,1.), angle=rot)
            self.rpy      = q2rpy(self.q)
            self.heading  = qmag2h(self.q, self.mag, declination=0.0)

            self.accBias      = Vector3D(0.,0.,0.)
            self.velocityBias = Vector3D(0.,0.,0.)
            
            if counter % 100 == 0:
                self.dx = -self.dx
                self.dy = -self.dy
                self.dz = -self.dz
                
            self.position    += Vector3D(self.dx,self.dy,self.dz)
            self.velocity     = Vector3D(0.,0.,0.)
            self.residuals    = Vector3D(0.,0.,0.)
            self.dtmotion     = 0.1
                            
            ##############################################################
                
            self.zmq_deltaTime = currentTime - previous_zmqUpdate
            previous_zmqUpdate = copy(currentTime)

            zmq_updateCounts += 1
            if (currentTime - zmq_lastTimeRate) >= 1.:
                self.zmq_rate = copy(zmq_updateCounts)
                zmq_lastTimeRate = copy(currentTime)
                zmq_updateCounts = 0

            # format the IMU data
            data_IMU.time = self.sensorTime
            data_IMU.acc  = self.acc
            data_IMU.gyr  = self.gyr
            data_IMU.mag  = self.mag
            imu_msgpack = msgpack.packb(obj2dict(vars(data_IMU)))
            socket.send_multipart([b"imu", imu_msgpack])

            # format the Fusion data
            data_fusion.time = self.sensorTime
            data_fusion.acc  = self.acc
            data_fusion.gyr  = self.gyr 
            data_fusion.mag  = self.mag 
            data_fusion.rpy  = self.rpy
            data_fusion.heading = self.heading
            data_fusion.q    = self.q
            
            fusion_msgpack = msgpack.packb(obj2dict(vars(data_fusion)))
            socket.send_multipart([b"fusion", fusion_msgpack])

            # format the Button data
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
            
            button_msgpack = msgpack.packb(obj2dict(vars(data_button)))
            socket.send_multipart([b"button", button_msgpack])
            
            # Motion Data
            data_motion.time         = self.sensorTime
            data_motion.accBias      = self.acc
            data_motion.velocityBias = self.velocityBias
            data_motion.position     = self.position
            data_motion.velocity     = self.velocity
            data_motion.residuals    = self.residuals
            data_motion.dtmotion     = self.dtmotion
                             
            motion_msgpack = msgpack.packb(obj2dict(vars(data_motion)))
            socket.send_multipart([b"motion", motion_msgpack])            

            ##############################################################

            if args.report > 0:

                if (currentTime - previous_reportUpdate) >= self.report_updateInterval:
                    self.report_deltaTime = currentTime - previous_reportUpdate
                    previous_reportUpdate = copy(currentTime)

                    report_updateCounts += 1
                    if (currentTime - report_lastTimeRate) >= 1.:
                        self.report_rate = copy(report_updateCounts)
                        report_lastTimeRate = copy(currentTime)
                        report_updateCounts = 0

                    # Display the Data
                    msg_out = '-------------------------------------------------\n'
                    msg_out+= 'dummy ZMQ IMU Generator\n'
                    msg_out+= '-------------------------------------------------\n'

                    msg_out+= 'Data    {:>10.6f}, {:>3d}/s\n'.format(self.data_deltaTime*1000.,        self.data_rate)
                    msg_out+= 'Report  {:>10.6f}, {:>3d}/s\n'.format(self.report_deltaTime*1000.,      self.report_rate)
                    if args.zmqport is not None:
                        msg_out+= 'ZMQ     {:>10.6f}, {:>3d}/s\n'.format(self.zmq_deltaTime*1000.,     self.zmq_rate)
                    msg_out+= '-------------------------------------------------\n'

                    msg_out+= 'Time: {:>10.6f}, dt: {:>10.6f}\n'.format(self.sensorTime, self.data_deltaTime)
                    msg_out+= 'Accel {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z)
                    msg_out+= 'Gyro  {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.gyr.x,self.gyr.y,self.gyr.z)
                    msg_out+= 'Magno {:>8.3f} {:>8.3f} {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z)
                    
                    msg_out+= 'Trig:{} Touch:{} Back:{} Home:{}, Vol+:{} Vol-:{} Any:{}\n'.format(
                                                        'Y' if self.trigger     else 'N', 
                                                        'Y' if self.touch       else 'N', 
                                                        'Y' if self.back        else 'N',
                                                        'Y' if self.home        else 'N',
                                                        'Y' if self.volume_up   else 'N',
                                                        'Y' if self.volume_down else 'N',
                                                        'N' if self.noButton    else 'Y')
                    msg_out+= 'Touch Pad:{},{}\n'.format(self.touchX, self.touchY)

                    msg_out+= '-------------------------------------------------\n'
                    msg_out+= 'Acc     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.acc.x,self.acc.y,self.acc.z,self.acc.norm)
                    msg_out+= 'Gyr     {:>8.3f} {:>8.3f} {:>8.3f} RPM:{:>8.3f}\n'.format(self.gyr.x,self.gyr.y,self.gyr.z,self.gyr.norm*60./TWOPI)
                    msg_out+= 'Mag     {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.mag.x,self.mag.y,self.mag.z,self.mag.norm)
                    msg_out+= 'Euler: R{:>6.1f} P{:>6.1f} Y{:>6.1f}, Heading {:>4.0f}\n'.format(
                                                    self.rpy.x*RAD2DEG, self.rpy.y*RAD2DEG, self.rpy.z*RAD2DEG, 
                                                    self.heading*RAD2DEG)
                    msg_out+= 'Q:     W{:>6.3f} X{:>6.3f} Y{:>6.3f} Z{:>6.3f}\n'.format(
                                                    self.q.w, self.q.x, self.q.y, self.q.z)
                    msg_out+= '-------------------------------------------------\n'
                    
                    msg_out+= 'Residual {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.residuals.x,self.residuals.y,self.residuals.z,self.residuals.norm)
                    msg_out+= 'Vel      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.velocity.x,self.velocity.y,self.velocity.z,self.velocity.norm)
                    msg_out+= 'Pos      {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.position.x,self.position.y,self.position.z,self.position.norm)
                    msg_out+= 'Vel Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.velocityBias.x,self.velocityBias.y,self.velocityBias.z,self.velocityBias.norm)
                    msg_out+= 'Acc Bias {:>8.3f} {:>8.3f} {:>8.3f} N:  {:>8.3f}\n'.format(self.accBias.x,self.accBias.y,self.accBias.z,self.accBias.norm)
                    msg_out+= 'dt       {:>10.6f}\n'.format(self.dtmotion)


                    print(msg_out, flush=True)

            ##############################################################

            self.logger.log(logging.DEBUG, 'Data Updated')                        

            # Wait to next interval time
            sleepTime = self.data_updateInterval - (time.perf_counter() - currentTime)
            await asyncio.sleep(max(0.,sleepTime))
            timingError = time.perf_counter() - currentTime - self.data_updateInterval
            self.data_updateInterval = max(0., self.targetRate - timingError)
            
        self.logger.log(logging.INFO, 'Stopping')
                
    async def handle_termination(self, tasks:None):
        '''
        Stop the task loops
        Stop the sensor
        Cancel tasks if list is provided which will speed up closing of program
        '''
        self.finish_up = True
        if tasks is not None: # This will terminate tasks faster
            self.logger.log(logging.INFO, 'Cancelling all Tasks...')
            await asyncio.sleep(0.5) # give some time for tasks to finish up
            for task in tasks:
                if task is not None:
                    task.cancel()

async def main(args: argparse.Namespace):

    # Setup logging
    logger = logging.getLogger(__name__)
    logger.log(logging.INFO, 'Starting dummy ZMQ IMU Generator')
    # gearVRC Controller
    controller = dummyZMQ(logger=logger)

    main_task     = asyncio.create_task(controller.update()) 
             
    # Set up a Control-C handler to gracefully stop the program
    # This mechanism is only available in Unix
    if os.name == 'posix':
        # Get the main event loop
        loop = asyncio.get_running_loop()
        loop.add_signal_handler(signal.SIGINT,  lambda: asyncio.create_task(controller.handle_termination(tasks=[main_task])) ) # control-c
        loop.add_signal_handler(signal.SIGTERM, lambda: asyncio.create_task(controller.handle_termination(tasks=[main_task])) ) # kill

    # Wait until all tasks are completed, which is when user wants to terminate the program
    await asyncio.wait([main_task], timeout=float('inf'))

    logger.log(logging.INFO,'Exit')

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()

    parser.add_argument(
        '-u',
        '--updaterate',
        dest = 'rate',
        type = float,
        metavar='<rate>',
        help='update rate in Hz',
        default = 60.
    )

    parser.add_argument(
        '-r',
        '--report',
        dest = 'report',
        type = int,
        metavar='<report>',
        help='report level: 0(None), 1(regular)',
        default = 0
    )

    parser.add_argument(
        '-s',
        '--static', 
        dest = 'static',
        action='store_true', 
        help='Keeps buttons pressed',
    )

    parser.add_argument(
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = int,
        metavar='<zmqport>',
        help='port used by ZMQ, e.g. 5556',
        default = 5556
    )

    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='sets the log level from info to debug',
        default = False
    )

    args = parser.parse_args()
        
    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(
        level=log_level,
        format='%(asctime)-15s %(levelname)s: %(message)s'
    )   
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        pass
