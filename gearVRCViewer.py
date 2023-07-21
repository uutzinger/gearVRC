# Samsung gear VR Controller Viewer
# Urs Utzinger 2023

import sys, os, pathlib, math
import logging, argparse

import zmq, msgpack

import numpy as np

from PyQt5.QtCore import QThread, QSettings, QObject, pyqtSignal, pyqtSlot, QTime, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtGui import QOpenGLShaderProgram, QOpenGLShader, QSurfaceFormat, QIcon

from OpenGL.GL import glClearColor, glEnable, glClear, glViewport, \
                      glMatrixMode, glLoadIdentity, glTranslatef, \
                      glShadeModel, glRotatef, glColor4f, glBegin, glVertex2f, glEnd, \
                      glTexCoord2f, glVertex3f,glGenTextures, glBindTexture, glTexImage2D, \
                      glTexParameteri, glNormal3f, glGenLists, glNewList, glEndList, glCallList, \
                      glDisable, glReadBuffer, glReadPixels, glBlendFunc, glBlendFunc, \
                      glPushMatrix, glPopMatrix, glFlush, glClearDepth, glDepthFunc, glMaterialfv, \
                      glLightfv, glLightModeli, glMatrixMode, \
                      glBindVertexArray, glBindBuffer, glBufferData, glEnableVertexAttribArray, \
                      glVertexAttribPointer, glTexParameteri, glTexParameteri, glTexParameteri, \
                      glGenVertexArrays, glGenBuffers, glUseProgram, glDrawArrays, glPushClientAttrib, \
                      glPushAttrib, glInterleavedArrays, glPopAttrib, glPopClientAttrib, glMaterialf, \
                      glTexParameterf, glCullFace, glColor3f

from OpenGL.GLU import gluLookAt, gluPerspective
from OpenGL.GL import GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_TEXTURE_2D, \
                      GL_LIGHTING, GL_LIGHT0, GL_LIGHT1, GL_PROJECTION, \
                      GL_MODELVIEW, GL_DEPTH_TEST, GL_COLOR_MATERIAL, \
                      GL_SMOOTH, GL_POLYGON, GLfloat, GL_TRIANGLES, \
                      GL_UNSIGNED_BYTE, GL_RGBA, GL_RGB, GL_LINEAR, GL_TEXTURE_MIN_FILTER, \
                      GL_TEXTURE_MAG_FILTER, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE, \
                      GL_TEXTURE_WRAP_T, GL_COMPILE, GL_FRONT, GL_BLEND, GL_SRC_ALPHA, \
                      GL_ONE_MINUS_SRC_ALPHA, GL_TRIANGLE_FAN, GL_QUADS, GL_LEQUAL, \
                      GL_DIFFUSE, GL_POSITION, GL_AMBIENT, GL_SPECULAR, GL_LIGHT_MODEL_LOCAL_VIEWER, \
                      GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, GL_COMPILE_STATUS, GL_LINK_STATUS, \
                      GL_FLOAT, GL_FALSE, GL_UNSIGNED_INT, GL_UNSIGNED_BYTE, GL_UNSIGNED_SHORT, \
                      GL_STATIC_DRAW, GL_ARRAY_BUFFER, GL_REPEAT, GL_BACK, GL_FRONT_AND_BACK, \
                      GL_CLIENT_VERTEX_ARRAY_BIT, GL_CURRENT_BIT, GL_ENABLE_BIT, GL_LIGHTING_BIT, \
                      GL_CULL_FACE, GL_DEPTH_TEST, GL_BACK, GL_EMISSION, GL_SHININESS

from pywavefront.visualization import VERTEX_FORMATS, gl_light
from pywavefront import Wavefront

from pyglet import image

sys.path.append('..')
from gearVRC import gearFusionData, gearButtonData, gearMotionData, dict2obj

###################################################################
# GLOBALS
###################################################################
ZMQTIMEOUT     = 1000 # in milliseconds
DEG2RAD         = math.pi / 180.0
TWOPI           = 2.0 * math.pi
ZMQTIMEOUT      = 1000 # in milliseconds
FIELDOFVIEW     = 20.0 # in degrees

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
        self.new_motion = False
        self.timeout    = False
        
        self.zmqTimeout = ZMQTIMEOUT 
        
        self.logger.log(logging.INFO, 'zmqWorker initialized'  )
    
    def start(self):

        self.running = True
        self.paused  = False

        self.new_fusion = False
        self.new_button = False
        self.new_motion = False
        self.timedout   = False
        
        context = zmq.Context()
        poller = zmq.Poller()
        
        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"fusion") 
        socket.setsockopt(zmq.SUBSCRIBE, b"button")
        socket.setsockopt(zmq.SUBSCRIBE, b"motion")
        socket.connect(self.zmqPort)
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'zmqWorker started on {}'.format(self.zmqPort))
                
        while self.running:
            try:
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
                                self.new_fusion = True
                        elif topic == b"button":
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
                                self.new_button = True
                        elif topic == b"motion":
                            msg_dict = msgpack.unpackb(msg_packed)
                            data_motion = dict2obj(msg_dict)
                            if hasattr(data_motion, 'time') and \
                            hasattr(data_motion, 'residuals') and \
                            hasattr(data_motion, 'velocity') and \
                            hasattr(data_motion, 'position') and \
                            hasattr(data_motion, 'accBias') and \
                            hasattr(data_motion, 'velocityBias') and \
                            hasattr(data_motion, 'dtmotion'):
                                self.new_motion = True            
                        else:
                            pass # not a topic we need
                    else:
                        self.logger.log(logging.ERROR, 'zmqWorker malformed message')
                else: # ZMQ TIMEOUT
                    self.logger.log(logging.ERROR, 'zmqWorker timed out')
                    self.timedout = True
                    poller.unregister(socket)
                    socket.close()
                    socket = context.socket(zmq.SUB)
                    socket.setsockopt(zmq.SUBSCRIBE, b"fusion") 
                    socket.setsockopt(zmq.SUBSCRIBE, b"button")
                    socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                    socket.connect(self.zmqPort)
                    poller.register(socket, zmq.POLLIN)
                    
                if (self.new_fusion and self.new_button and self.new_motion) and not self.timedout:
                    if not self.paused:
                        self.dataReady.emit([data_button, data_fusion, data_motion])
                        self.new_fusion = False
                        self.new_button = False
                        self.new_motion = False
                        self.timedout   = False
            except:
                self.logger.log(logging.ERROR, 'zmqWorker error')
                poller.unregister(socket)
                socket.close()
                socket = context.socket(zmq.SUB)
                socket.setsockopt(zmq.SUBSCRIBE, b"fusion") 
                socket.setsockopt(zmq.SUBSCRIBE, b"button")
                socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                socket.connect(self.zmqPort)
                poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.DEBUG, 'zmqWorker finished')
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
        self.textureID  = None
        self.textureWidth  = 0
        self.textureHeight = 0
        
        # Rendering parameters
        self.background_color = (200./255, 200./255, 205./255)
        self.fusion = gearFusionData()
        self.button = gearButtonData()
        self.motion = gearMotionData()

        self.logger.log(logging.INFO, 'GL Widget setup'  )
                
    def initializeGL(self):
        # Set up OpenGL settings
        glClearColor(*self.background_color, 1.0)
        # glClearColor(0.2, 0.2, 0.2, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        
        # Load the object
        modelFileName = os.path.join(os.path.dirname(__file__), 'model/gear_vr_controller.obj')
        self.object = Wavefront(modelFileName, collect_faces=True, create_materials=True)

        ###############
        material = self.object.materials['controller']
        if material.gl_floats is None:
            material.gl_floats = (GLfloat * len(material.vertices))(*material.vertices)
            material.triangle_count = len(material.vertices) / material.vertex_size

        self.vertex_format = VERTEX_FORMATS.get(material.vertex_format)
        ###############
                
        # Calculate the camera position and distance
        min_bound = np.min(self.object.vertices, axis=0)
        max_bound = np.max(self.object.vertices, axis=0)
        self.object_center = (min_bound + max_bound) / 2.0
        object_size = np.linalg.norm(max_bound - min_bound)
        camera_distance = 2.5 * object_size
        self.camera_position = self.object_center + camera_distance*np.array([0.0, 0.1, 1.0])
        # x is the axis pointing to the right
        # y is the axis pointing up
        # z is the axis pointing towards the user
        
        ##############
        # Load texture
        self.texture = self.load_texture(material)
        ###############
        
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
        # if self.motion is not None:
        #    position = self.motion.position
        #    
        # glTranslatef(position.y, -position.z, -position.x)

        # Rotating the controller with Quaternion
        angle = 2 * math.acos(q.w) * 180.0 / math.pi
        scale = math.sqrt(1 - q.x * q.x)
        axis_x = q.x / scale
        axis_y = q.y / scale
        axis_z = q.z / scale
        glRotatef(angle, axis_y, -axis_z, -axis_x)
        
        # Render the Object
        ##############################
        for name, material in self.object.materials.items():

            #####################
            glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT)
            glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT)

            glEnable(GL_CULL_FACE)
            glEnable(GL_DEPTH_TEST)
            glCullFace(GL_BACK)

            glEnable(self.texture.image.target)
            glBindTexture(self.texture.image.target, self.texture.image.id)
            if self.texture.options.clamp == "on":
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            else:
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_T, GL_REPEAT)
            
            if material.has_normals:
                face = GL_FRONT_AND_BACK
                glMaterialfv(face, GL_DIFFUSE,   gl_light(material.diffuse))
                glMaterialfv(face, GL_AMBIENT,   gl_light(material.ambient))
                glMaterialfv(face, GL_SPECULAR,  gl_light(material.specular))
                glMaterialfv(face, GL_EMISSION,  gl_light(material.emissive))
                glMaterialf(face,  GL_SHININESS, min(128.0, material.shininess))
                glEnable(GL_LIGHT0)
                glEnable(GL_LIGHTING)
            else:
                glDisable(GL_LIGHTING)
                glColor4f(*material.ambient)
            
            # Unfortunately this is not working
            # self.update_texture(button, w=self.texture.image.width, h=self.texture.image.height)
            
            glInterleavedArrays(self.vertex_format, 0, material.gl_floats)
            glDrawArrays(GL_TRIANGLES, 0, int(material.triangle_count))

            glPopAttrib()
            glPopClientAttrib()
        
        ###########################
        
        self.logger.log(logging.DEBUG, 'Scene painted'  )

    def resizeGL(self, width, height):
        # Adjust the viewport and projection matrix
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fov = FIELDOFVIEW
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

    #################################
    def load_texture(self, material):

        texture = material.texture or material.texture_ambient

        if texture and material.has_uvs:

            textureFileName = material.texture.path
            img = image.load(textureFileName)
            texture.image = img.get_texture()
            
            self.textureWidth  = texture.image.width
            self.textureHeight = texture.image.height
            self.textureID     = texture.image.id
                
            glEnable(texture.image.target)
            glBindTexture(texture.image.target, texture.image.id)
            if texture.options.clamp == "on":
                glTexParameterf(texture.image.target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
                glTexParameterf(texture.image.target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            else:
                glTexParameterf(texture.image.target, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameterf(texture.image.target, GL_TEXTURE_WRAP_T, GL_REPEAT) 
        else: 
            glDisable(texture)
        
        return texture

    def update_texture(self, button, w, h):
            '''Update the texture based on the button state'''
                        
            glEnable(GL_BLEND)                                # enable blending
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) # set the blending for source and destination
            
            # Render onto rectangle
            glColor4f(1.0, 1.0, 1.0, 1.0) # set color to white
            left, right, top, bottom = 0., 1., 0, 1.
            glBegin(GL_QUADS)
            glVertex2f(left, bottom)   # Bottom-left corner
            glVertex2f(right, bottom)  # Bottom-right corner
            glVertex2f(right, top)     # Top-right corner
            glVertex2f(left, top)      # Top-left corner
            glEnd()
            
            if (button.touchX > 0) and (button.touchY > 0):
                # Touch pad indicates 0/0 when its not touched

                glColor4f(1.0, 0.0, 0.0, 1.0)
                glPushMatrix()
                glTranslatef(197./w, 60./h, 0.0)
                glRotatef(-45, 0, 0, 1)
                glBegin(GL_TRIANGLE_FAN)
                cx = ((button.touchX - 157.5) / 157.5 * 30) / w
                cy = (h - ((button.touchY - 157.5) / 157.5 * 30)) / h
                glVertex2f(cx, cy)
                rx = 5. / w
                ry = 5. / h
                num_segments = 100
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cy + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
                glPopMatrix()
                            
            if button.volume_up:

                glColor4f(1.0, 0.0, 0.0, 1.0)
                glBegin(GL_POLYGON)
                cx = 106. / w / 2.0
                cy = (h - 13.) / h / 2.0
                glVertex2f(cx, cy)
                rx =  11. / w / 2.0
                ry =  11. / h / 2.0
                num_segments = 100
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cy + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
            
            if button.volume_down:

                glColor4f(1.0, 0.0, 0.0, 0.5)
                glBegin(GL_POLYGON)
                cx = 140. / w
                cy = (h -  13.) / h
                glVertex2f(cx, cy)
                rx =  11. / w
                ry =  11. / h
                num_segments = 100
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cy + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
            
            if button.back:

                glColor4f(1.0, 0.0, 0.0, 0.5)
                glBegin(GL_POLYGON)
                cx =  24. / w
                cy =  (h - 18.) / h
                glVertex2f(cx, cy)
                rx =  20. / w
                ry =  20. / h
                num_segments = 100
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cy + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
                    
            if button.home:

                glColor4f(1.0, 0.0, 0.0, 0.5)
                glBegin(GL_POLYGON)
                cx = 124. / w
                cy = (h - 44.) / h
                glVertex2f(cx, cy)
                rx =  20. / w
                ry =  20. / h
                num_segments = 100
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cy + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
                
            if button.trigger:
                x = [113.,138.,152.,154.,154.,140.,117.,125.,122.,111.,122.,108.]
                y = [ 61., 61.,113.,170.,230.,256.,256.,220.,163.,120.,163.,101.]        
                glColor4f(1.0, 0.0, 0.0, 0.5)
                glBegin(GL_POLYGON)
                for vx, vy in zip(x, y):
                    glVertex2f(vx/w, (h - vy)/h)
                glEnd()

            if button.touch:
            
                glColor4f(1.0, 0.0, 0.0, 0.5)
                glBegin(GL_POLYGON)
                cx = 197 / w
                cy = (h - 60) / h
                glVertex2f(cx, cy)
                rx =  30 / w
                ry =  30 / h
                num_segments = 100
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cy + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
                
            if self.bluetooth_on:
                glColor4f(0.0, 0.0, 1.0, 0.5)
                glBegin(GL_TRIANGLE_FAN)
                cx = 197. / w / 2
                cy = ( 208.) / h / 2
                glVertex2f(cx, cy)
                rx = 10. / w / 2
                ry = 10. / h / 2
                num_segments = 20
                for i in range(num_segments + 1):
                    angle = TWOPI * (i / num_segments)
                    x = cx + rx * math.cos(angle)
                    y = cx + ry * math.sin(angle)
                    glVertex2f(x, y)
                glEnd()
            
            glDisable(GL_BLEND)                             # turn off blending
                            
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
        self.zmqWorker.set_zmqPort(args.zmqport)

        self.zmqWorkerThread.start()

        self.logger.log(logging.INFO, 'Main initialized with port: {}'.format(args.zmqport)  )
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
        default = "tcp://UrsPi:5556"
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