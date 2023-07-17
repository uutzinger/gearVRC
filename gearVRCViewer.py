# Samsung gear VR Controller Viewer
# Urs Utzinger 2023

import sys, os, pathlib, math
import logging, argparse

import zmq, msgpack

import numpy as np

from PyQt5.QtCore import QThread, QSettings, QObject, pyqtSignal, pyqtSlot, QTime, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtGui import QOpenGLShaderProgram, QOpenGLShader, QSurfaceFormat, QIcon

from OpenGL.GL import glClearColor, glEnable, glDisable, glClear, glViewport, \
                      glMatrixMode, glLoadIdentity, glTranslatef, glGenTextures, \
                      glBindTexture, glTexImage2D, glLightfv, glShadeModel, glRotatef
                          
from OpenGL.GLU import gluLookAt, gluPerspective
from OpenGL.GL import GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_TEXTURE_2D, \
                      GL_LIGHTING, GL_LIGHT0, GL_RGB, GL_UNSIGNED_BYTE, GL_PROJECTION, \
                      GL_MODELVIEW, GL_DEPTH_TEST, GL_AMBIENT, GL_DIFFUSE, GL_LIGHT1, \
                      GL_POSITION, GL_COLOR_MATERIAL,GL_SMOOTH

from pywavefront.visualization import draw_material
from pywavefront import Wavefront

sys.path.append('..')
from gearVRC import gearFusionData, gearButtonData, gearMotionData, dict2obj


###################################################################
# GLOBALS
###################################################################
ZMQTIMEOUT     = 1000 # in milliseconds

###################################################################
# ZMQWorker
###################################################################

class zmqWorker(QObject):

    dataReady = pyqtSignal(object)
    finished  = pyqtSignal()
    
    def __init__(self, logger, parent=None):
        super(zmqWorker, self).__init__(parent)

        self.logger  = logger
        self.running = False
        self.paused  = False
        self.zmqPort = 'tcp://localhost:5556'

        self.new_fusion = False
        self.new_button = False
        self.timeout    = False
        
        self.zmqTimeout = ZMQTIMEOUT 
        
        self.logger.log(logging.INFO, 'zmqWorker initialized'  )
    
    def start(self):

        self.running = True
        self.paused  = False

        new_fusion = False
        new_button = False
        new_motion = False
        timeout    = False
        
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"fusion") 
        socket.setsockopt(zmq.SUBSCRIBE, b"button")
        socket.setsockopt(zmq.SUBSCRIBE, b"motion")
        socket.connect(self.zmqPort)
        poller = zmq.Poller()
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'zmqWorker started'  )
                
        while self.running:
            events = dict(poller.poll(timeout = self.zmqTimeout))
            if socket in events and events[socket] == zmq.POLLIN:
                response = socket.recv_multipart()
                if len(response) == 2:
                    [topic, msg_packed] = response
                    if topic == b"fusion":
                        msg_dict = msgpack.unpackb(msg_packed)
                        data_fusion = dict2obj(msg_dict)
                        if hasattr(data_fusion, 'q') and \
                           hasattr(data_fusion, 'rpy') and \
                           hasattr(data_fusion, 'heading') and\
                           hasattr(data_fusion, 'time'):
                            new_fusion = True
                    if topic == b"button":
                        msg_dict = msgpack.unpackb(msg_packed)
                        data_button = dict2obj(msg_dict)
                        if hasattr(data_button, 'trigger') and \
                           hasattr(data_button, 'back') and \
                           hasattr(data_button, 'home') and \
                           hasattr(data_button, 'volume_up') and \
                           hasattr(data_button, 'volume_down') and \
                           hasattr(data_button, 'touch') and \
                           hasattr(data_button, 'noButton') and \
                           hasattr(data_button, 'touchX') and \
                           hasattr(data_button, 'touchY') and\
                           hasattr(data_button, 'time'):
                            new_button = True
                    if topic == b"motion":
                        msg_dict = msgpack.unpackb(msg_packed)
                        data_motion = dict2obj(msg_dict)
                        if hasattr(data_motion, 'time') and \
                           hasattr(data_motion, 'residuals') and \
                           hasattr(data_motion, 'velocity') and \
                           hasattr(data_motion, 'position') and \
                           hasattr(data_motion, 'accBias') and \
                           hasattr(data_motion, 'velocityBias') and \
                           hasattr(data_motion, 'dtmotion'):
                            new_motion = True            
                    else:
                        pass # not a topic we need
                else:
                    pass # got malformed message
            else: # ZMQ TIMEOUT
                timeout = True

            if (new_fusion and new_button and new_motion) and not timeout:
                if not self.paused:
                    self.dataReady.emit([data_button, data_fusion, data_motion])
                    new_fusion = False
                    new_button = False
                    new_motion = False
                    timeout    = False

        socket.close()
        context.term()
        self.finished.emit()

    def stop(self):
        self.running = False
        self.paused  = False
                
    def set_zmqPort(self, port):
        self.zmqPort = port

    def pause(self):
        self.paused = not self.paused

                    
###################################################################
# OpenGL Widget
###################################################################

class GLWidget(QOpenGLWidget):
    def __init__(self, logger, parent=None):
        super(GLWidget, self).__init__(parent)
        
        self.logger = logger
        
        # Initialize variables
        self.object     = None
        
        # Rendering parameters
        self.background_color = (200./255, 200./255, 205./255)
        self.fusion = gearFusionData()
        self.button = gearButtonData()
        self.motion = gearMotionData()

        self.logger.log(logging.INFO, 'GL Widget setup'  )
                
    def initializeGL(self):
        # Set up OpenGL settings
        glClearColor(0.2, 0.2, 0.2, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        
        # Load the object
        modelFileName = os.path.join(os.path.dirname(__file__), 'model/gear_vr_controller.obj')
        self.object = Wavefront(modelFileName, collect_faces=True, create_materials=True)
                
        # Calculate the camera position and distance
        min_bound = np.min(self.object.vertices, axis=0)
        max_bound = np.max(self.object.vertices, axis=0)
        self.object_center = (min_bound + max_bound) / 2.0
        object_size = np.linalg.norm(max_bound - min_bound)
        camera_distance = 1.0 * object_size
        self.camera_position = self.object_center + np.array([0, 0, camera_distance])

        self.logger.log(logging.INFO, 'GL Widget initialized'  )
        
    def paintGL(self):

        # Clear the buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glClearColor(*self.background_color, 1.0)
         
        glLoadIdentity()

        # Set up the Camera Position and Orientation
        ############################################
        
        gluLookAt(self.camera_position[0], self.camera_position[1], self.camera_position[2],
                  self.object_center[0],   self.object_center[1],   self.object_center[2],
                  0, 1, 0)
        
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_MODELVIEW)

        # Set up the Object Position and Orientation
        ############################################
        if self.fusion is not None:
            q = self.fusion.q

        if self.button is not None:
            button = self.button

        # Translation the controller with motion
        # Will need to implement motion in gear VR Controller  
        if self.motion is not None:
            position = self.motion.position
            
        glTranslatef(0. + position.x, 0. + position.y, -1.0 + position.z)

        # Rotating the controller with Quaternion
        angle = 2 * math.acos(q.w) * 180.0 / math.pi
        scale = math.sqrt(1 - q.x * q.x)
        axis_x = q.x / scale
        axis_y = q.y / scale
        axis_z = q.z / scale
        glRotatef(angle, axis_x, axis_y, axis_z)
        
        # Render the Object
        ##############################
        for name, material in self.object.materials.items():
            draw_material(material)

        self.logger.log(logging.DEBUG, 'Scene painted'  )

    def resizeGL(self, width, height):
        # Adjust the viewport and projection matrix
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fov = 10.0
        aspect_ratio = float(width) / height
        near = 0.1
        far = 100.0
        gluPerspective(fov, aspect_ratio, near, far)
        glEnable(GL_DEPTH_TEST) # might or might not be needed
        glMatrixMode(GL_MODELVIEW)

        self.logger.log(logging.DEBUG, 'Scene resized'  )

    def updateSystem(self, data):
        # Trigger the paintGL routine to update the graphics
        [self.button, self.fusion, self.motion] = data
        self.update()
        self.logger.log(logging.DEBUG, 'System updated'  )
                
###################################################################
# Main Window
###################################################################
        
class MainWindow(QMainWindow):

    ###################################################################
    # Init
    ###################################################################
    
    def __init__(self, args):
        super(MainWindow, self).__init__()
        
        self.logger = logging.getLogger(__name__)
        log_level = logging.DEBUG if args.debug else logging.INFO
        self.logger.setLevel(log_level)
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        self.logger.addHandler(handler)
        
        self.logger.log(logging.INFO, 'Starting gearVR Controller Viewer...')

        self.zmqPort = args.zmqport
        
        self.setWindowTitle("gear VR Controller Viewer")
                
        # Create the OpenGL widget
        self.gl_widget = GLWidget(parent=self, logger=self.logger)
        self.setCentralWidget(self.gl_widget)
        
        # Set the initial size and position of the window
        self.setGeometry(100, 100, 800, 600)
        
        current_directory = str(pathlib.Path(__file__).parent.absolute())
        path = current_directory + '/gearViewer.png'
        self.setWindowIcon(QIcon(path))

        self.zmqWorker = zmqWorker(logger=self.logger)
        self.zmqWorkerThread = QThread()
        self.zmqWorker.moveToThread(self.zmqWorkerThread)

        # Connect the worker's rotationChanged signal to the updateRotation slot
        self.zmqWorker.dataReady.connect(self.gl_widget.updateSystem)

        self.zmqWorker.finished.connect(self.worker_finished)

        self.zmqWorkerThread.started.connect(self.zmqWorker.start)
        self.zmqWorkerThread.finished.connect(self.worker_thread_finished)

        self.zmqWorker.set_zmqPort(self.zmqPort)

        self.zmqWorkerThread.start()

        self.logger.log(logging.INFO, 'Main initialized with port: {}'.format(self.zmqPort)  )
    ###################################################################
    # Start Worker
    ###################################################################
  
    def stop_worker(self):
        self.zmqWorker.stop()
        self.worker_finished()

    def worker_finished(self):
        self.zmqWorkerThread.quit()
        self.zmqWorkerThread.wait()

    def worker_thread_finished(self):
        self.zmqWorkerThread.deleteLater()
        self.zmqWorker.deleteLater()
                                
if __name__ == '__main__':
    app = QApplication(sys.argv)
    
    parser = argparse.ArgumentParser(description="gearVRC Viewer")

    parser.add_argument(
        '-z',
        '--zmq',
        dest = 'zmqport',
        type = str,
        metavar='<zmqport>',
        help='port used by ZMQ, tcp://UrsPi:5556',
        default = 'tcp://localhost:5556'
    ) 

    parser.add_argument(
        '-d',
        '--debug',
        dest = 'debug',
        type = bool,
        metavar='<debug>',
        help='Debugging on or off',
        default = False
    ) 
    args=parser.parse_args()

        
    window = MainWindow(args)
    window.show()
    sys.exit(app.exec_())