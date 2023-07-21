# Samsung gear VR Controller Viewer
# Urs Utzinger 2023

import sys, os, pathlib, math
import logging, argparse

import zmq, msgpack

import numpy as np

from PyQt5.QtCore import QThread, QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtGui import QIcon

from OpenGL.GL import glClearColor, glEnable, glClear, glViewport, \
                      glMatrixMode, glLoadIdentity, glTranslatef, \
                      glShadeModel, glRotatef, glColor4f, glBegin, glVertex2f, glEnd, \
                      glTexCoord2f, glVertex3f,glGenTextures, glBindTexture, glTexImage2D, \
                      glTexParameteri, glNormal3f, glGenLists, glNewList, glEndList, glCallList, \
                      glDisable, glReadBuffer, glReadPixels, glBlendFunc, glBlendFunc, \
                      glPushMatrix, glPopMatrix, glFlush
  
from OpenGL.GLU import gluLookAt, gluPerspective
from OpenGL.GLUT import glutSwapBuffers
from OpenGL.GL import GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_TEXTURE_2D, \
                      GL_LIGHTING, GL_LIGHT0, GL_PROJECTION, \
                      GL_MODELVIEW, GL_DEPTH_TEST, GL_COLOR_MATERIAL, \
                      GL_SMOOTH, GL_POLYGON, \
                      GLfloat, GL_TRIANGLES, GL_UNSIGNED_BYTE, GL_RGBA, \
                      GL_LINEAR, GL_TEXTURE_MIN_FILTER, GL_TEXTURE_MAG_FILTER, \
                      GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE, GL_TEXTURE_WRAP_T, GL_COMPILE, \
                      GL_FRONT, GL_BLEND, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_TRIANGLE_FAN, \
                      GL_QUADS

from pywavefront.visualization import draw_material, VERTEX_FORMATS
from pywavefront import Wavefront

from pyglet import image

sys.path.append('..')
from archive.gearVRC import gearFusionData, gearButtonData, gearMotionData, dict2obj

DEG2RAD = math.pi / 180.0
TWOPI  = 2.0 * math.pi

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

        new_fusion   = False
        new_button   = False
        new_motion   = False
        timeout      = False
        
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
                # attempt reconnecting
                socket.connect(self.zmqPort)

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
        self.bluetooth_on = False
        
        self.logger.log(logging.INFO, 'GL Widget setup'  )
                
    def initializeGL(self):
        # Set up OpenGL settings
        glClearColor(0., 0., 0., 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_TEXTURE_2D)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        
        # Load the wavefront object
        modelFileName = os.path.join(os.path.dirname(__file__), 'model/gear_vr_controller.obj')
        self.object = Wavefront(modelFileName, strict=False, collect_faces=True, create_materials=True)

        # Load the texture for the wavefront object
        self.load_texture()
                            
        # Calculate the camera position and distance
        min_bound = np.min(self.object.vertices, axis=0)
        max_bound = np.max(self.object.vertices, axis=0)
        self.object_center = (min_bound + max_bound) / 2.0
        object_size = np.linalg.norm(max_bound - min_bound)
        camera_distance = 2.5 * object_size
        self.camera_position = self.object_center + np.array([0, 0, camera_distance])

        # Create display list
        ############################################

        self.display_list = glGenLists(1)
        glNewList(self.display_list, GL_COMPILE)
        
        # Create the triangles
        ############################################
        faces     = self.object.mesh_list[0].faces
        materials = self.object.materials['controller']
        if materials.vertex_format == 'T2F_N3F_V3F':
            vertices_x = materials.vertices[5:-1:8]
            vertices_y = materials.vertices[6:-1:8]
            vertices_z = materials.vertices[7:-1:8]
            textures_u = materials.vertices[0:-1:8]
            textures_v = materials.vertices[1:-1:8]
            normals_x  = materials.vertices[2:-1:8]
            normals_y  = materials.vertices[3:-1:8]
            normals_z  = materials.vertices[4:-1:8]
        else:
             assert False, 'Vertex format not supported: {}'.format(materials.vertex_format)   
             
        glBegin(GL_TRIANGLES)
        for face in faces:
            vertex_index, texture_index, normal_index = face
            # Render the vertex with texture coordinates
            glTexCoord2f(textures_u[texture_index], textures_v[texture_index])
            glVertex3f(vertices_x[vertex_index], vertices_y[vertex_index], vertices_z[vertex_index])
            glNormal3f(normals_x[normal_index], normals_y[normal_index], normals_z[normal_index])
        glEnd()        
        
        glEndList() 

        self.was_any_button_pressed = False
        
        self.logger.log(logging.INFO, 'GL Widget initialized'  )
        
    def load_texture(self):
        ''''''
        
        material = self.object.materials['controller']

        if material.gl_floats is None:
            material.gl_floats = (GLfloat * len(material.vertices))(*material.vertices)
            material.triangle_count = len(material.vertices) / material.vertex_size

        vertex_format = VERTEX_FORMATS.get(material.vertex_format)
        if not vertex_format:
            raise ValueError("Vertex format {} not supported.".format(material.vertex_format))

        # load texture
        textureFileName = material.texture.path
        texture_image = image.load(textureFileName)
        self.textureWidth = texture_image.width
        self.textureHeight = texture_image.height
        pixels = texture_image.get_data('RGBA', pitch=self.textureWidth * 4)

        # Load the texture
        #  create texture structure
        glEnable(GL_TEXTURE_2D)
        self.texture_id = glGenTextures(1)                   # create one texture image
        glBindTexture(GL_TEXTURE_2D, self.texture_id)   # use the texture image
        #  set texture parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        #  pass the texture image to glTexImage2D
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.textureWidth, self.textureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels)
                         
    def paintGL(self):

        # Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glClearColor(*self.background_color, 1.0)
        glLoadIdentity()

        glCallList(self.display_list)

        # Set up the Camera Position and Orientation
        ############################################
        
        gluLookAt(self.camera_position[0], self.camera_position[1], self.camera_position[2],
                  self.object_center[0],   self.object_center[1],   self.object_center[2],
                  0, 1, 0)
        
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_MODELVIEW)

        # Obtain the rendering parameters
        ############################################
        if self.fusion is not None:
            q = self.fusion.q

        if self.motion is not None:
            position = self.motion.position
        
        if self.button is not None:
            button = self.button

        # Apply rendering conditions based on
        #  the controller's orientation and position
        ############################################
        
        # OpenGL rendering convention
        # x is the axis pointing to the right
        # y is the axis pointing up
        # z is the axis pointing towards the user
        #
        # The controller IMU is modeled in the following way:
        # x is the axis pointing forward
        # y is the axis pointing to the right
        # z is the axis pointing down
        
        glTranslatef(position.y, -position.z, -position.x)
                
        # Rotating the controller with Quaternion
        angle = 2 * math.acos(q.w) * 180.0 / math.pi
        scale = math.sqrt(1 - q.x * q.x)
        axis_x = q.x / scale
        axis_y = q.y / scale
        axis_z = q.z / scale
        glRotatef(angle, axis_y, -axis_z, -axis_x)
        
        # Render the Object
        ##############################
        # for name, material in self.object.materials.items():
        #     draw_material(material)

        # Render the texture on a rectangle
        self.update_texture(button)

        glFlush() # ??
        # glutSwapBuffers() ?
                
        self.logger.log(logging.DEBUG, 'Scene painted'  )

    def resizeGL(self, width, height):
        # Adjust the viewport and projection matrix
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fov = 45.0
        aspect_ratio = float(width) / height
        near = 0.01
        far = 100.0
        gluPerspective(fov, aspect_ratio, near, far)
        glEnable(GL_DEPTH_TEST) # might or might not be needed
        glMatrixMode(GL_MODELVIEW)

        self.logger.log(logging.DEBUG, 'Scene resized'  )

    def update_texture(self, button):

        is_any_button_pressed = False
        
        glDisable(GL_DEPTH_TEST)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, self.texture_id)        

        # Render the texture on a rectangle
        glColor4f(1.0, 1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glTexCoord2f(0, 0)
        glVertex2f(  0, 0)
        glTexCoord2f(1, 0)
        glVertex2f(  1, 0)
        glTexCoord2f(1, 1)
        glVertex2f(  1, 1)
        glTexCoord2f(0, 1)
        glVertex2f(  0, 1)
        glEnd()
                
        if (button.touchX > 0) and (button.touchY > 0):
            # Touch pad indicates 0/0 when its not touched

            glColor4f(1.0, 0.0, 0.0, 1.0)
            glBegin(GL_TRIANGLE_FAN)
            glPushMatrix()
            glTranslatef(197/self.textureWidth, 60/self.textureHeight, 0.0)
            glRotatef(-45, 0, 0, 1)
            cx = ((button.touchX - 157.5) / 157.5 * 30) / self.textureWidth
            cy = ((button.touchY - 157.5) / 157.5 * 30) / self.textureHeight
            glVertex2f(cx, cy)
            rx = 5. / self.textureWidth
            ry = 5. / self.textureHeight
            num_segments = 100
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cy + ry * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
            glPopMatrix()
            is_any_button_pressed = True
                         
        if button.volume_up:

            glColor4f(1.0, 0.0, 0.0, 0.5)
            glBegin(GL_POLYGON)
            cx = 106 / self.textureWidth
            cy =  13 / self.textureHeight
            glVertex2f(cx, cy)
            rx =  11 / self.textureWidth
            ry =  11 / self.textureHeight
            num_segments = 100
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cy + ry * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
            is_any_button_pressed = True
        
        if button.volume_down:

            glColor4f(1.0, 0.0, 0.0, 0.5)
            glBegin(GL_POLYGON)
            cx = 140 / self.textureWidth
            cy =  13 / self.textureHeight
            glVertex2f(cx, cy)
            rx =  11 / self.textureWidth
            ry =  11 / self.textureHeight
            num_segments = 100
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cy + ry * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
            is_any_button_pressed = True
        
        if button.back:

            glColor4f(1.0, 0.0, 0.0, 0.5)
            glBegin(GL_POLYGON)
            cx =  24 / self.textureWidth
            cy =  18 / self.textureHeight
            glVertex2f(cx, cy)
            rx =  20 / self.textureWidth
            ry =  20 / self.textureHeight
            num_segments = 100
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cy + ry * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
            is_any_button_pressed = True
                
        if button.home:

            glColor4f(1.0, 0.0, 0.0, 0.5)
            glBegin(GL_POLYGON)
            cx = 124 / self.textureWidth
            cy =  44 / self.textureHeight
            glVertex2f(cx, cy)
            rx =  20 / self.textureWidth
            ry =  20 / self.textureHeight
            num_segments = 100
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cy + ry * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
            is_any_button_pressed = True
            
        if button.trigger:
            x = [113,138,152,154,154,140,117,125,122,111,122,108] / self.textureWidth
            y = [ 61, 61,113,170,230,256,256,220,163,120,163,101] / self.textureHeight        
            glColor4f(1.0, 0.0, 0.0, 0.5)
            glBegin(GL_POLYGON)
            for vx, vy in zip(x, y):
                glVertex2f(vx, vy)
            glEnd()
            is_any_button_pressed = True

        if button.touch:

            glColor4f(1.0, 0.0, 0.0, 0.5)
            glBegin(GL_POLYGON)
            cx = 197 / self.textureWidth
            cy =  60 / self.textureHeight
            glVertex2f(cx, cy)
            rx =  30 / self.textureWidth
            ry =  30 / self.textureHeight
            num_segments = 100
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cy + ry * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
            is_any_button_pressed = True
            
        if self.bluetooth_on:

            glColor4f(0.0, 0.0, 1.0, 1.0)
            glBegin(GL_TRIANGLE_FAN)
            cx = 197 / self.textureWidth
            cy = 208 / self.textureHeight
            glVertex2f(cx, cy)
            rx = 1. / self.textureWidth
            ry = 1. / self.textureHeight
            num_segments = 20
            for i in range(num_segments + 1):
                angle = TWOPI * (i / num_segments)
                x = cx + rx * math.cos(angle)
                y = cx + rx * math.sin(angle)
                glVertex2f(x, y)
            glEnd()
        
        glDisable(GL_BLEND)
        glDisable(GL_TEXTURE_2D)
        
        # if is_any_button_pressed or self.was_any_button_pressed != is_any_button_pressed:
        #     glReadBuffer(GL_FRONT)
        #     data = glReadPixels(0, 0, self.textureWidth, self.textureHeight, GL_RGBA, GL_UNSIGNED_BYTE)
        #     # update material map image
        #     self.material.map.image.set_data('RGBA', self.textureWidth, self.textureHeight, data)
        #     self.material.needsUpdate = True

        # self.was_any_button_pressed = is_any_button_pressed            
        glEnable(GL_DEPTH_TEST)
        
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