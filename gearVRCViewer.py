#!/usr/bin/python3

###############################################################################
# Samsung gearVR Controller Viewer
###############################################################################
# Renders the controller as a 3D object based on the controllers pose.
# Receives pose through ZMQ from gearVRC.py
# Highlights the buttons based on user input.
#
# Urs Utzinger 2023
###############################################################################
# This work uses ideas from  Gear VRC reverse engineering:
#   https://github.com/jsyang/gearvr-controller-webbluetooth
#   In particular the idea of updating the texture based on user
#   pressing buttons.
#
# chat.openai.com was extensively used for OpenGL programming
# Code autocompletion was provided by Github Copilot
###############################################################################
# Prerequisite:
#   -python packages
#      pyqt5, pyopengl, pywavefront, pyglet, zmq, msgpack, numpy
#   -some data structures and methods from gearVRC.py
###############################################################################

# Various
from gearVRC import gearFusionData, gearButtonData, gearMotionData, dict2obj
import sys, os, pathlib, math
import logging, argparse
import zmq, msgpack
import numpy as np
import ctypes

# PyQT5
from PyQt5.QtCore import QThread, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget, QWidget, \
    QVBoxLayout, QSizePolicy, QHBoxLayout
from PyQt5.QtGui import QIcon, QOpenGLShader, QOpenGLShaderProgram

# OpenGL
from OpenGL.GL import glClearColor, glEnable, glClear, glViewport, glMatrixMode, \
    glLoadIdentity, glTranslatef, glShadeModel, glRotatef, glColor4f, glGenTextures, \
    glBindTexture, glTexImage2D, glDisable, glBlendFunc, glMaterialfv, \
    glDrawArrays, glPushClientAttrib, glPushAttrib, glInterleavedArrays, \
    glPopAttrib, glPopClientAttrib, glMaterialf, glTexParameteri, glCullFace, \
    glGenFramebuffers, glBindFramebuffer, glFramebufferTexture2D, glCheckFramebufferStatus, \
    glUseProgram, glGetUniformLocation, glBindVertexArray, \
    glBufferData, glUniform4f, glBlendEquation, glCreateShader, glShaderSource, \
    glCompileShader, glCreateProgram, glAttachShader, glLinkProgram, glGenBuffers, \
    glBindBuffer, glEnableVertexAttribArray, glVertexAttribPointer, glGenVertexArrays, \
    glDrawBuffers, glOrtho, glBegin, glEnd, glTexCoord2f, glVertex2f, glGetShaderiv, \
    glGetShaderInfoLog, glDeleteShader, glGetProgramiv, glGetProgramInfoLog, \
    glGetTexLevelParameteriv, glGetTexImage, glGetError, glFlush, glDrawBuffer, \
    glActiveTexture, glUniform1i, glGetIntegerv, glPopMatrix, glPushMatrix, \
    glCopyImageSubData, glEnableClientState, glDisableClientState, glVertexPointer, \
    glDeleteBuffers, glDeleteVertexArrays, glLightfv
                                         
from OpenGL.GLU import gluLookAt, gluPerspective

from OpenGL.GL import GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_TEXTURE_2D, \
    GL_LIGHTING, GL_LIGHT0, GL_PROJECTION, GL_MODELVIEW, GL_DEPTH_TEST, \
    GL_COLOR_MATERIAL, GL_SMOOTH, GL_POLYGON, GLfloat, GL_TRIANGLES, \
    GL_UNSIGNED_BYTE, GL_RGBA, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE, \
    GL_TEXTURE_WRAP_T, GL_BLEND, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, \
    GL_DIFFUSE, GL_AMBIENT, GL_SPECULAR, GL_REPEAT, \
    GL_BACK, GL_FRONT_AND_BACK, GL_CLIENT_VERTEX_ARRAY_BIT, GL_CURRENT_BIT, \
    GL_ENABLE_BIT, GL_LIGHTING_BIT, GL_CULL_FACE, \
    GL_EMISSION, GL_SHININESS, GL_FRAMEBUFFER, GL_RGBA8, GL_FRAMEBUFFER_COMPLETE, \
    GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, \
    GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, GL_COLOR_ATTACHMENT5, \
    GL_COLOR_ATTACHMENT6, GL_COLOR_ATTACHMENT7, \
    GL_TEXTURE_MIN_FILTER, GL_LINEAR, \
    GL_TEXTURE_MAG_FILTER, GL_VERTEX_SHADER, GL_FRAGMENT_SHADER, \
    GL_ARRAY_BUFFER, GL_DYNAMIC_DRAW, GL_FUNC_ADD, GL_FLOAT, GL_FALSE, \
    GL_TRUE, GL_COMPILE_STATUS, GL_LINK_STATUS, GL_TEXTURE_WIDTH, \
    GL_TEXTURE_HEIGHT, GL_TEXTURE_INTERNAL_FORMAT, GL_ALL_ATTRIB_BITS, \
    GL_CLIENT_ALL_ATTRIB_BITS, GL_VIEWPORT_BIT, GL_TRANSFORM_BIT, \
    GL_ACCUM_BUFFER_BIT, GL_EVAL_BIT, GL_LINE_BIT, GL_LIST_BIT, \
    GL_PIXEL_MODE_BIT, GL_POINT_BIT, GL_POLYGON_BIT, GL_TEXTURE_BIT, \
    GL_TEXTURE1, GL_TEXTURE0, GL_TEXTURE_BINDING_2D, \
    GL_FRAMEBUFFER_BINDING, GL_CURRENT_PROGRAM, GL_VERTEX_ARRAY, \
    GL_POSITION, GL_LIGHT1, GL_POSITION, GL_DIFFUSE
                               
# Pywavefront is used to load the 3D model
from pywavefront.visualization import VERTEX_FORMATS
from pywavefront import Wavefront

# pyglet is used to load the texture image
from pyglet import image

# sys.path.append('..')

###############################################################################
# GLOBALS
###############################################################################

ZMQTIMEOUT = 10000  # in milliseconds
DEG2RAD = math.pi / 180.0
TWOPI = 2.0 * math.pi
FIELDOFVIEW = 20.0  # in degrees

###############################################################################
# Shaders
###############################################################################

# Vertex shader code for drawing polygons on texture
VERTEX_SHADER_DRAW = """
#version 330 core
// vertex position input
in vec2 position;
void main()
{
    // Pass the vertex position and color to the fragment shader
    gl_Position = vec4(position, 0.0, 1.0);
}
"""

# Fragment shader code for drawing polygons on texture
FRAGMENT_SHADER_DRAW = """
#version 330 core
// Output color of the fragment shader
out vec4 FragColor;       
 // Input color from the vertex shader 
uniform vec4 color;
void main()
{
    // Pass the vertex color to the output
    FragColor = color;
}
"""
###################################################################
# Utility functions
###################################################################

# Function to create vertices for a circle
# input: (as measured with imageJ app)
#   - center coordinates cx,cy 
#     0/0 is upper left
#   - radius rx, ry in absolute coordinates
#   - scale is maximum vale for x and y
# output: (as used by OpenGL)
#   - relative coordinates from -1 ... 1, 
#     0/0 center, -1/-1 lower left, 1/1 upper right
def circle_vertices(cx, cy, rx, ry, num_segments, scalex, scaley):
    angle_step = 2 * math.pi / num_segments
    vertices = np.empty((num_segments, 2), dtype=np.float32)
    cx2, cy2, rx2, ry2 = cx * 2.0, cy * 2.0, rx * 2.0, ry * 2.0
    for i in range(num_segments ):
        angle = i * angle_step
        x =       (cx2 + rx2 * math.cos(angle)) / scalex  - 1.0
        y = 1.0 - (cy2 + ry2 * math.sin(angle)) / scaley
        vertices[i, 0] = x
        vertices[i, 1] = y
    return vertices

def getTextureParameters(texture_id):
    glBindTexture(GL_TEXTURE_2D, texture_id)
    width = glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH)
    height = glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT)
    internal_format = glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_INTERNAL_FORMAT)
    if width > 0 or height > 0:
        pixels = glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE)
    else:
        pixels = None
    glBindTexture(GL_TEXTURE_2D, 0)

    return width, height, internal_format, pixels

##############################################################################
# ZMQWorker
###############################################################################

class zmqWorker(QObject):

    dataReady = pyqtSignal(object)
    BLEstatus = pyqtSignal(bool)
    finished  = pyqtSignal()

    def __init__(self, logger, parent=None):
        super(zmqWorker, self).__init__(parent)

        self.logger = logger
        self.running = False
        self.paused = False
        self.zmqPort = 'tcp://localhost:5556'

        self.new_fusion = False
        self.new_button = False
        self.new_motion = False
        self.timeout = False

        self.zmqTimeout = ZMQTIMEOUT

        self.logger.log(logging.INFO, 'zmqWorker initialized')

    def start(self):

        self.running = True
        self.paused = False

        self.new_fusion = False
        self.new_button = False
        self.new_motion = False

        context = zmq.Context()
        poller  = zmq.Poller()
        
        data_motion = None
        data_button = None
        data_fusion = None

        socket = context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
        socket.setsockopt(zmq.SUBSCRIBE, b"button")
        socket.setsockopt(zmq.SUBSCRIBE, b"motion")
        socket.connect(self.zmqPort)
        poller.register(socket, zmq.POLLIN)

        self.logger.log(logging.INFO, 'zmqWorker started on {}'.format(self.zmqPort))

        while self.running:
            try:
                events = dict(poller.poll(timeout=self.zmqTimeout))
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
                            pass  # not a topic we need
                    else:
                        self.logger.log(
                            logging.ERROR, 'zmqWorker malformed message')
                else:  # ZMQ TIMEOUT
                    self.logger.log(logging.ERROR, 'zmqWorker timed out')
                    poller.unregister(socket)
                    socket.close()
                    socket = context.socket(zmq.SUB)
                    socket.setsockopt(zmq.SUBSCRIBE, b"fusion")
                    socket.setsockopt(zmq.SUBSCRIBE, b"button")
                    socket.setsockopt(zmq.SUBSCRIBE, b"motion")
                    socket.connect(self.zmqPort)
                    poller.register(socket, zmq.POLLIN)
                    self.new_fusion = False
                    self.new_button = False
                    self.new_motion = False

                if (self.new_fusion and self.new_button and self.new_motion):
                    if not self.paused:
                        self.dataReady.emit([data_button, data_fusion, data_motion])
                        self.BLEstatus.emit(True)
                        self.new_fusion = False
                        self.new_button = False
                        self.new_motion = False
                else:
                    if not self.paused:
                        self.BLEstatus.emit(False)
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
                self.new_fusion = False
                self.new_button = False
                self.new_motion = False

        self.logger.log(logging.DEBUG, 'zmqWorker finished')
        socket.close()
        context.term()
        self.finished.emit()

    def stop(self):
        self.running = False
        self.paused = False

    def set_zmqPort(self, port):
        self.zmqPort = port

    def pause(self):
        self.paused = not self.paused

###############################################################################
# Texture Display Widget for Debugging OpenGL
###############################################################################

class ImageDisplayWidget(QOpenGLWidget): # QWidget
    
    def __init__(self, logger, parent=None):
        super(ImageDisplayWidget, self).__init__(parent)
        
        self.logger              = logger
        self.imageTextureID      = None
        self.imageTextureWidth   = 256
        self.imageTextureHeight  = 256

        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        self.setFixedSize(self.imageTextureWidth, self.imageTextureHeight)

        self.logger.log(logging.INFO, 'Image Display Widget is setup'  )

    @pyqtSlot(object)
    def receiveTexture(self, pixelData):
        # Unpack the data
        self.imageTextureWidth, self.imageTextureHeight, internal_format, pixels = pixelData
        if pixels is not None:
            self.logger.log(logging.DEBUG, 'Received image {}x{} with has internal format {}: pixels in RGBA {}..'.format(\
                self.imageTextureWidth, self.imageTextureHeight, internal_format, pixels[0:5]) )
        else:
            self.logger.log(logging.DEBUG, 'Received image {}x{} with has internal format {}'.format(\
                self.imageTextureWidth, self.imageTextureHeight, internal_format) )

        # Check for valid texture ID
        if self.imageTextureID is not None:

            # Bind existing texture ID for update            
            glBindTexture(GL_TEXTURE_2D, self.imageTextureID)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.imageTextureWidth, self.imageTextureHeight, \
                                        0, GL_RGBA, GL_UNSIGNED_BYTE, pixels)

            # Set texture parameters
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

            # Unbind the texture after update
            # glBindTexture(GL_TEXTURE_2D, 0)
        
        # Trigger a repaint of the widget to reflect the changes
        self.update()
        
        self.logger.log(logging.DEBUG, 'Image display texture is updated'  )
        
    def initializeGL(self):

        # Generate texture ID
        self.imageTextureID  = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.imageTextureID)

        # Allocate memory for texture
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.imageTextureWidth, self.imageTextureHeight, \
                                    0, GL_RGBA, GL_UNSIGNED_BYTE, None)

        # Set properties of texture
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        #######################################################################
        # Create vertex coordinates to render texture to image display widget
        #######################################################################

        self.fullTextureVertices = np.array([
            # Triangle 1
            [0.0,                     0.0],
            [self.imageTextureWidth,  0.0],
            [self.imageTextureWidth,  self.imageTextureHeight],
            # Triangle 2
            [0.0,                     0.0],
            [self.imageTextureWidth,  self.imageTextureHeight],
            [0.0,                     self.imageTextureHeight],
        ], dtype=np.float32)

        self.fullTextureVerticesData        = self.fullTextureVertices.astype(np.float32).tobytes()
        self.fullTextureVerticesBufferSize  = ctypes.sizeof(ctypes.c_float) * self.fullTextureVertices.size
        self.fullTextureVerticesNumVertices = len(self.fullTextureVertices)

        glClearColor(0.0, 0.0, 0.0, 0.0)
        glEnable(GL_TEXTURE_2D)
        
        self.logger.log(logging.INFO, 'Image Display Widget is initialized'  )
        
    def resizeGL(self, width, height):
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, width, 0, height, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glEnable(GL_TEXTURE_2D)
        
        self.logger.log(logging.INFO, 'Image Display Widget is resized'  )

    def paintGL(self):

        if self.imageTextureID is None:
            self.logger.log(logging.DEBUG, 'No Texture, Image Display Scene not painted'  )
            return
        
        width, height, internal_format, pixels = getTextureParameters(self.imageTextureID)
        if pixels is not None:
            self.logger.log(logging.DEBUG, 'Displaying image {} {}x{} with internal format {}: pixels in RGBA {}..'.format(\
                                          self.imageTextureID, width, height, internal_format, pixels[0:5]) )
        else:
            self.logger.log(logging.DEBUG, 'Displaying image {} {}x{} with internal format {}'.format(\
                                          self.imageTextureID, width, height, internal_format) )
                    
        #Setup OpenGL settings
        glViewport(0, 0, self.width(), self.height())
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # type: ignore

        glLoadIdentity()

        if self.imageTextureID is not None:
            # Bind the texture and draw it on a rectangle
            glEnable(GL_TEXTURE_2D)
            glBindTexture(GL_TEXTURE_2D, self.imageTextureID)
                        
            glBegin(GL_TRIANGLES)
            # Triangle 1
            glTexCoord2f(0, 0); glVertex2f(            0,             0)  # Vertex 1 (top-left) - Triangle 1
            glTexCoord2f(1, 0); glVertex2f( self.width(),             0)  # Vertex 2 (top-right) - Triangle 1
            glTexCoord2f(1, 1); glVertex2f( self.width(), self.height())  # Vertex 3 (bottom-right) - Triangle 1
            # Triangle 2
            glTexCoord2f(0, 0); glVertex2f(            0,             0)  # Repeated Vertex 1 (top-left) - Triangle 2
            glTexCoord2f(1, 1); glVertex2f( self.width(), self.height())  # Repeated Vertex 3 (bottom-right) - Triangle 2
            glTexCoord2f(0, 1); glVertex2f(            0, self.height())  # Vertex 4 (bottom-left) - Triangle 2
            glEnd()
            
            glFlush()
                                        
            glBindTexture(GL_TEXTURE_2D, 0)
            glDisable(GL_TEXTURE_2D)
                        
            er= glGetError()
            if er == 0:
                self.logger.log(logging.DEBUG, 'Image display Scene is painted')
            else:   
                self.logger.log(logging.DEBUG, 'Image display Scene is painted with error {}'.format(glGetError()))
    
###############################################################################
# Main Display Widget
###############################################################################

class ObjectRenderingWidget(QOpenGLWidget):
    
    newTextureAvailable = pyqtSignal(object)
    
    def __init__(self, logger, position=False, debug=False, renderButtons=False, parent=None):
        super(ObjectRenderingWidget, self).__init__(parent)

        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.logger                 = logger                   
        self.usePosition            = position  # move the object based on positional data
        self.debug                  = debug     # displays texture rendering
        self.renderButtons          = renderButtons

        # Initialize variables
        self.object                 = None
        self.renderingObject        = None
        self.originalTextureID      = None
        self.originalTextureWidth   = 0
        self.originalTextureHeight  = 0
        self.bluetooth_on           = False

        # Rendering parameters
        self.backgroundColor        = (200./255, 200./255, 205./255)
        self.fusion                 = gearFusionData()
        self.button                 = gearButtonData()
        self.motion                 = gearMotionData()
        self.q                      = None
        self.position               = None
        
        # Shader parameters
        self.blendingFBO            = None
        self.displayTextureID       = None
        self.originalTextureID      = None
        self.originalTextureWidth   = 0
        self.originalTextureHeight  = 0
        self.originalTextureClamp   = False

        self.logger.log(logging.INFO, 'Main Display Widget is setup')

    def initializeGL(self):
        
        #######################################################################
        # Load the 3D object
        #######################################################################
        
        modelFileName = os.path.join(os.path.dirname(__file__), 'model/gear_vr_controller.obj')
        # PyWavefront objects internal structure is complex and texture data is
        #   not automatically loaded.
        # The example visualization code uses pyglet image to load the texture image, which 
        #   creates a data structure containing texture pixel data and texture ID.
        # For rendering an array of vertices is created.
        self.object = Wavefront(modelFileName, collect_faces=True, create_materials=True, strict=False)

        #######################################################################
        # Calculate the camera position and distance based on the object size
        #######################################################################
        
        if self.object is not None:
            min_bound = np.min(self.object.vertices, axis=0)
            max_bound = np.max(self.object.vertices, axis=0)
            self.object_center = (min_bound + max_bound) / 2.0
            object_size = np.linalg.norm(max_bound - min_bound)
            camera_distance = 2.5 * object_size
            self.camera_position = self.object_center + \
                camera_distance*np.array([0.0, 1.0, 1.0])
            # x is the axis pointing to the right
            # y is the axis pointing up
            # z is the axis pointing towards the user
        else:
            self.object_center   = np.array([0.0, 0.0, 0.0])
            self.camera_position = np.array([0.0, 0.0, 0.0])

        #######################################################################
        # Convert the vertices to OpenGL GLfloat array
        #######################################################################
        
        if self.object is not None:
            # Converting the vertices to GLfloat array
            self.renderingObject = self.object.materials['controller'] # Object
            if self.renderingObject.gl_floats is None:
                self.renderingObject.gl_floats = (GLfloat * len(self.renderingObject.vertices))(*self.renderingObject.vertices)
                self.renderingObject.triangle_count = len(self.renderingObject.vertices) / self.renderingObject.vertex_size
            #
            self.vertex_format = VERTEX_FORMATS.get(self.renderingObject.vertex_format)
        else:
            self.renderingObject = None
            self.vertex_format = None

        #######################################################################
        # Load textures
        #######################################################################
        if self.renderingObject is not None:    
            self.originalTextureID,     \
            self.originalTextureWidth,  \
            self.originalTextureHeight, \
            self.originalTextureClamp = \
                self.load_object_texture(self.renderingObject)

        else:
            self.originalTextureID      = None
            self.originalTextureWidth   = 0
            self.originalTextureHeight  = 0
            self.originalTextureClamp   = False

        #######################################################################
        # If we want to render device button highlights we need to create:
        #  - a frame buffer and attached to it the original texture and a 
        #     texture to merge original texture with button highlights
        #  - a vertex array object and vertex buffer object
        #  - a vertex and fragment shaders
        #######################################################################

        # Define pre-made vertices
        # ------------------------
        # This creates circles and polygons for the device buttons
        self.create_button_vertices()
        
        # Create frame buffer with two texture, 
        #   original texture (static)  originalTextureID
        #   display texture (modified) displayTextureID
        # -----------------------------------------------

        # Create and bind new FBO
        # -----------------------
        self.blendingFBO = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)

        # displayTextureID
        # ----------------
        # Generate new display texture and attach it to the FBO
        self.displayTextureID  = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
        # Allocate memory for the texture and specify its format size, use original texture dimensions
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.originalTextureWidth, self.originalTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        # Set the display texture parameters
        if self.originalTextureClamp:
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        else:
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)            
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR) 
        # Attach the display texture to the FBO               
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, self.displayTextureID, 0)    

        # originalTextureID
        # ------------------
        # Attach original texture to the FBO
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, self.originalTextureID, 0)

        # We only want to draw to the display texture
        # -------------------------------------------
        glDrawBuffer(GL_COLOR_ATTACHMENT0)

        # Check if the frame buffer is complete
        if glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError("Texture frame buffer is not complete!")

        # Cleanup: unbind the FBOs and textures
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glBindTexture(GL_TEXTURE_2D, 0) 

        #######################################################################
        # Create vertex drawing shaders to render polygons onto texture
        #######################################################################

        # Create and compile vertex and fragment shaders
        self.drawShaderProgram = QOpenGLShaderProgram()
        self.drawShaderProgram.addShaderFromSourceCode(QOpenGLShader.Vertex, VERTEX_SHADER_DRAW)
        self.drawShaderProgram.addShaderFromSourceCode(QOpenGLShader.Fragment, FRAGMENT_SHADER_DRAW)
        self.drawShaderProgram.link()

        self.lightPosition0 = [15.0, 15.0, -15.0, 1.0]
        self.lightDiffuse0 =  [ 0.67, 0.67, 0.67, 1.0] 
        self.lightPosition1 = [15.0, -15.0, 15.0, 1.0]
        self.lightDiffuse1 =  [ 0.8, 0.8, 0.8, 1.0] 
                    
        glLightfv(GL_LIGHT0, GL_POSITION, light_position)
        
        self.logger.log(logging.INFO, 'Main GL Widget is initialized')

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
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_MODELVIEW)

        self.logger.log(logging.DEBUG, 'Scene resized')

    def paintGL(self):
        
        #######################################################################
        # Update the display texture based on user interaction with the device
        #######################################################################
                
        if self.renderButtons:        
            # Copy original texture to display texture
            glCopyImageSubData(
                self.originalTextureID, GL_TEXTURE_2D, 0, 0, 0, 0,       # Source
                self.displayTextureID,  GL_TEXTURE_2D, 0, 0, 0, 0,       # Destination
                self.originalTextureWidth, self.originalTextureHeight, 1 # Size
            )
                                    
            if self.button is not None:
                # Update display texture with button highlights
                self.render_buttons_to_display(self.button)

            # Update texture in image display widget for debugging
            if self.displayTextureID is not None:
                if self.debug:
                    width, height, internal_format, pixels = getTextureParameters(self.displayTextureID)
                    self.newTextureAvailable.emit([width,height, internal_format, pixels])

        else:
            # Update texture in image display widget for debugging
            if self.debug:
                if self.originalTextureID is not None:
                    width,height, internal_format, pixels = getTextureParameters(self.originalTextureID)
                    self.newTextureAvailable.emit([width,height, internal_format, pixels])
        
        #######################################################################
        # Prepare System for Rendering Object
        #######################################################################
        # set view port
        # set matrices:
        #  FinalVertex = ProjectionMatrix * ViewMatrix * ModelMatrix * Vertex
        #  - projection matrix (perspective or orthographic projection)
        #  - view matrix (camera position and orientation)
        #  - model matrix (position & scale & orientation of model)
        # activating textures and materials
        # render object

        # Clear
        # -----
        # Set up general OpenGL settings
        glClearColor(*self.backgroundColor, 1.0)  # glClearColor(0.2, 0.2, 0.2, 1.0)
        # Clear the display buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT) # type: ignore
        glShadeModel(GL_SMOOTH)
        
        # Set view port to match widgets size
        # ------------------------------------
        glViewport(0,0,self.width(),self.height())

        # Setup the projection matrix (perspective or orthographic projection)
        #----------------------------
        # Set up perspective or orthographic projection using glFrustum, gluPerspective, or glOrtho
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        fov = FIELDOFVIEW
        aspect_ratio = float(self.width()) / self.height()
        near, far = 0.1, 100.0
        gluPerspective(fov, aspect_ratio, near, far)
    
        # Set up view matrix
        # ------------------
        # Perform camera transformations (e.g., translation, rotation) using glTranslate* and glRotate*
        gluLookAt(self.camera_position[0], self.camera_position[1], self.camera_position[2],
                    self.object_center[0],   self.object_center[1],   self.object_center[2],
                    0, 1, 0)

        # Set up the model matrix (object's position, scale, and orientation)
        # -------------------------------------------------------------------
        # Perform object transformations (e.g., translation, rotation) using glTranslate* and glRotate*
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        if self.fusion is not None:
            self.q = self.fusion.q

        # Move the controller in space based on computed position
        #   this likely has large drift and sensor might run out of window
        if self.usePosition:
            if self.motion is not None:
                self.position = self.motion.position

            if self.position is not None:                
                glTranslatef(self.position.y, -self.position.z, -self.position.x)

        # Rotating the controller with Quaternion
        if self.q is not None:
            angle = 2 * math.acos(self.q.w) * 180.0 / math.pi
            scale = math.sqrt(1 - self.q.x * self.q.x)
            axis_x = self.q.x / scale
            axis_y = self.q.y / scale
            axis_z = self.q.z / scale
            glRotatef(angle, axis_y, -axis_z, -axis_x)

        #######################################################################
        # Render the Object
        #######################################################################
        # - bind texture for 3D object
        # - set material properties
        # - (bind VAO containing objects vertex data), not used
        # - draw the 3D object
        # - (unbind VAO and texture), not used
        
        if self.renderingObject is not None:

            # Store current rendering environment
            glPushClientAttrib(GL_CLIENT_VERTEX_ARRAY_BIT)
            glPushAttrib(GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT) # type: ignore

            glEnable(GL_CULL_FACE)
            glEnable(GL_DEPTH_TEST)
            glCullFace(GL_BACK)

            # Texture
            # -------
            glEnable(GL_TEXTURE_2D)
            if self.renderButtons:
                glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
            else:
                glBindTexture(GL_TEXTURE_2D, self.originalTextureID)

            if self.originalTextureClamp:
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            else:
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)

            # Material
            # --------
            glEnable(GL_COLOR_MATERIAL)
            if self.renderingObject.has_normals:
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   (GLfloat * 4)(*self.renderingObject.diffuse))
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   (GLfloat * 4)(*self.renderingObject.ambient))
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  (GLfloat * 4)(*self.renderingObject.specular))
                glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  (GLfloat * 4)(*self.renderingObject.emissive))
                glMaterialf (GL_FRONT_AND_BACK, GL_SHININESS, min(128.0, self.renderingObject.shininess))
                glLightfv(GL_LIGHT0, GL_POSITION, self.lightPosition0)
                glLightfv(GL_LIGHT1, GL_POSITION, self.lightPosition1)
                glLightfv(GL_LIGHT0, GL_DIFFUSE,  self.lightDiffuse0)
                glLightfv(GL_LIGHT1, GL_DIFFUSE,  self.lightDiffuse1)
                glEnable(GL_LIGHT0)
                glEnable(GL_LIGHT1)
                glEnable(GL_LIGHTING)
            else:
                glDisable(GL_LIGHTING)
                glColor4f(*self.renderingObject.ambient)

            # Render the object
            # -----------------
            glInterleavedArrays(self.vertex_format, 0, self.renderingObject.gl_floats)
            glDrawArrays(GL_TRIANGLES, 0, int(self.renderingObject.triangle_count))
            
            # Restore previous rendering environment
            glPopAttrib()
            glPopClientAttrib()

            self.logger.log(logging.DEBUG, 'Object scene painted')
        else:
            self.logger.log(logging.DEBUG, 'No object, scene not painted')

    def update_system(self, data):
        # Trigger the paintGL routine to update the graphics
        [self.button, self.fusion, self.motion] = data
        self.update()
        self.logger.log(logging.DEBUG, 'Buttons, fusion and motion updated')

    def update_BLEstatus(self, data):
        self.bluetooth_on = data
        self.logger.log(logging.DEBUG, 'Bluetooth updated')
       
    def load_object_texture(self, renderingObject):
        '''
        Load object texture
        '''

        #######################################################################
        # Load the texture from file
        #######################################################################
        # This is code from pywavefront.visualization.py.
        # pyglet loads the texture image
        
        texture = renderingObject.texture or renderingObject.texture_ambient

        if texture and renderingObject.has_uvs:

            textureFileName = renderingObject.texture.path
            img = image.load(textureFileName)
            texture.image = img.get_texture()

            glEnable(texture.image.target)
            glBindTexture(texture.image.target, texture.image.id)
            if texture.options.clamp == "on":
                glTexParameteri(texture.image.target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
                glTexParameteri(texture.image.target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            else:
                glTexParameteri(texture.image.target, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameteri(texture.image.target, GL_TEXTURE_WRAP_T, GL_REPEAT)

            textureWidth  = texture.image.width
            textureHeight = texture.image.height
            textureID     = texture.image.id
            textureClamp  = True if texture.options.clamp == "on" else False

        else:
            textureID     = None
            textureWidth  = 0
            textureHeight = 0
            textureClamp  = False

        return textureID, textureWidth, textureHeight, textureClamp
            
    def render_buttons_to_display(self, button):
        '''
        paintGL function to render all texture polygons to display texture
        '''
        
        ###################################################
        # Prepare the buffers and projections
        ###################################################

        # Generate the Vertex Array Object (VAO)
        self.vao = glGenVertexArrays(1)
        # Generate and bind a Vertex Buffer Object (VBO)
        self.vbo = glGenBuffers(1)
          
        # Set up the viewport and clear the screen
        # x/y is lower left corner, width/height is size of viewport
        glViewport(0, 0, self.originalTextureWidth, self.originalTextureHeight)
        # Set the draw buffer to the display texture
        # This is in initialize GL
            
        # Set up the orthographic projection for rendering the textures
        # this is default and does not need to be executed
        #glMatrixMode(GL_PROJECTION)
        #glLoadIdentity()
        # Set up projection matrix using glOrtho to map -1 to 1 to 0 to 1 for texture coordinates
        #glOrtho(0, self.originalTextureWidth, 0, self.originalTextureHeight, -1, 1)
        # # Set up the modelview matrix
        # glMatrixMode(GL_MODELVIEW)
        # glLoadIdentity()

        # Activate the FBO to make it the current frame buffer
        currentFBO = glGetIntegerv(GL_FRAMEBUFFER_BINDING)
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)
                
        # Enable blending
        glEnable(GL_BLEND)
        # glBlendEquation(GL_FUNC_ADD)  # Use simple addition for blending, is default
        # Use standard alpha blending
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        
        # Activate the shader
        self.drawShaderProgram.bind()

        ###################################################
        # Draw highlighted back and home button
        ###################################################
        if button.back:
            # Red color for the polygon
            self.draw_polygon(self.backVertices, (1.0, 0.0, 0.0, 0.5))

        if button.home:
            # Red color for the polygon
            self.draw_polygon(self.homeVertices, (1.0, 0.0, 0.0, 0.5))

        if button.volume_up:
            # Red color for the polygon
            self.draw_polygon(self.volumeUpVertices, (1.0, 0.0, 0.0, 0.5))

        if button.volume_down:
            # Red color for the polygon
            self.draw_polygon(self.volumeDownVertices, (1.0, 0.0, 0.0, 0.5))

        if button.trigger:
            # Red color for the polygon
            self.draw_polygon(self.trigger1Vertices, (1.0, 0.0, 0.0, 0.5))
            self.draw_polygon(self.trigger2Vertices, (1.0, 0.0, 0.0, 0.5))
            self.draw_polygon(self.trigger3Vertices, (1.0, 0.0, 0.0, 0.5))
            self.draw_polygon(self.trigger4Vertices, (1.0, 0.0, 0.0, 0.5))

        if button.touch:
            # Red color for the polygon
            self.draw_polygon(self.touchVertices, (1.0, 0.0, 0.0, 0.5))

        if button.touchX > 0 and button.touchY > 0:
            # touch pad coordinates are in the range of 0 to 315
            # top left corner is 0,0
            # The region of interest enclosing the touchpad is 60x60 pixels
            # The center of the touch pad is at self.touchCx and self.touchCy which is 197,60
            # We have vertices available with circle at 0/0 and radius 5
            #
            # Absolute coordinates of the touch locations in the texture image (0/0 is upper left):
            tCx = self.touchCx + (button.touchX - 157.5) / 157.5 * 30
            tCy = self.touchCy + (button.touchY - 157.5) / 157.5 * 30
            # relative coordinates for polygon vertex coordinates in OpenGL coordinates:
            offsetX =     2.0 * tCx / self.originalTextureWidth - 1.0
            offsetY = 1 - 2.0 * tCy / self.originalTextureHeight 
            # The pad vertices wer centered around 0/0. Lets update them by the offset.
            padVertices = self.padVertices + np.array([offsetX, offsetY])
            # Green color for the polygon
            self.draw_polygon(padVertices, (0.0, 1.0, 0.0, 0.5))

        if self.bluetooth_on:
            # Blue color for the polygon
            self.draw_polygon(self.ledVertices, (0.0, 0.0, 1.0, 1.0))
    
        ###################################################
        # Clean up
        ###################################################

        # Unbind the shader program
        self.drawShaderProgram.release()

        # Unbind frame buffer
        glDisable(GL_BLEND)
        glBindFramebuffer(GL_FRAMEBUFFER, currentFBO)

        # Restore the original viewport and projection matrix
        glViewport(0, 0, self.width(), self.height())  # Restore the original viewport size

        glDeleteBuffers(1, [self.vbo])
        glDeleteVertexArrays(1, [self.vao])

    def draw_polygon(self, vertices, color):
        '''
        Draw the vertices with the given color
        Vertices at numpy arrays with shape [num_vertices, 2]
        '''
        verticesData = vertices.astype(np.float32).tobytes()
        bufferSize   = ctypes.sizeof(ctypes.c_float) * vertices.size
        numVertices  = len(vertices)

        # Generate and bind a Vertex Array Object (VAO)
        glBindVertexArray(self.vao)            
        # Generate and bind a Vertex Buffer Object (VBO)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo)

        # Upload vertices to buffer
        glBufferData(GL_ARRAY_BUFFER, bufferSize, verticesData, GL_DYNAMIC_DRAW)
        # Specify how the positions are layed out in the buffer
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, None)
        # Enable the vertex attribute array at position 0 (positions)
        glEnableVertexAttribArray(0)

        # Unbind VBO (Note: You can safely unbind GL_ARRAY_BUFFER here)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        
        # Draw the polygon
        self.drawShaderProgram.setUniformValue("color", *color)                
        glDrawArrays(GL_POLYGON, 0, numVertices)

        # clean up
        # nothing
                                
    def create_button_vertices(self):

        #######################################################################
        # Descriptors for the controller features
        #######################################################################

        # Coordinates of the controller features on the texture image
        # 0/0 is at the upper left corner of the image
        # C is center and R is radius.

        self.backCx,       self.backCy       =  24,  18  # Back Button
        self.backRx,       self.backRy       =  16,  16
        self.homeCx,       self.homeCy       = 123,  43  # Home Button
        self.touchCx,      self.touchCy      = 198,  60  # Touch Pad
        self.touchRx,      self.touchRy      =  30,  30
        self.padRx,        self.padRy        =   3,   3
        self.volumeUpCx,   self.volumeUpCy   = 106,  12  # Volume Up
        self.volumeDownCx, self.volumeDownCy = 140,  12  # Volume Down
        self.volumeRx,     self.volumeRy     =  11,  11
        self.ledCx,        self.ledCy        = 197, 208
        self.ledRx,        self.ledRy        =   1,   1
        # Polygon must be concave
        self.trigger1X = np.array([113, 121, 134, 142, 148, 147, 139, 119, 110, 110])
        self.trigger1Y = np.array([ 65,  62,  62,  79,  94, 122, 130, 139, 118,  76])
        self.trigger2X = np.array([119, 139, 147, 153, 124])
        self.trigger2Y = np.array([139, 130, 137, 168, 165])
        self.trigger3X = np.array([124, 153, 156, 155, 151, 124])
        self.trigger3Y = np.array([165, 168, 179, 214, 230, 225])
        self.trigger4X = np.array([124, 151, 143, 116])
        self.trigger4Y = np.array([225, 230, 256, 256])

        sx = self.originalTextureWidth  # scale by the width
        sy = self.originalTextureHeight # scale by the height

        # Convert vertex polygon coordinates to OpenGL coordinates
        # In OpenGL 0/0 is the center of polygon vertex coordinates and the 
        #   lower left corner is -1/-1
        
        # back button polygon
        #####################
        cx, cy = self.backCx, self.backCy
        rx, ry = self.backRx, self.backRy
        n = 2*(rx+1) + 2*(ry+1) # number of segments
        self.backVertices = circle_vertices(cx, cy, rx, ry, n, sx, sy)

        # home button polygon
        ######################
        cx, cy = self.homeCx, self.homeCy
        rx, ry = self.backRx, self.backRy
        n = 2*(rx+1) + 2*(ry+1)
        self.homeVertices = circle_vertices(cx, cy, rx, ry, n, sx, sy)

        # touch button polygon
        ######################
        cx, cy = self.touchCx, self.touchCy
        rx, ry = self.touchRx, self.touchRy
        n = 2*(rx+1) + 2*(ry+1)
        self.touchVertices = circle_vertices(cx, cy, rx, ry, n, sx, sy)

        # volume up button polygon
        ######################
        cx, cy = self.volumeUpCx, self.volumeUpCy
        rx, ry = self.volumeRx, self.volumeRy
        n = 2*(rx+1) + 2*(ry+1)
        self.volumeUpVertices = circle_vertices(cx, cy, rx, ry, n, sx, sy)

        # volume down button polygon
        ######################
        cx, cy = self.volumeDownCx, self.volumeDownCy
        rx, ry = self.volumeRx, self.volumeRy
        n = 2*(rx+1) + 2*(ry+1)
        self.volumeDownVertices = circle_vertices(cx, cy, rx, ry, n, sx, sy)

        # led polygon
        ######################
        # Make it a rectangle
        cx, cy = self.ledCx, self.ledCy
        rx, ry = self.ledRx, self.ledRy
        ledX =       2.0 * np.array([cx-rx, cx+rx, cx+rx, cx-rx]) / sx - 1.0
        ledY = 1.0 - 2.0 * np.array([cy-ry, cy-ry, cy+ry, cy+ry]) / sy
        self.ledVertices = np.array([ledX, ledY]).T 

        # trigger polygon
        #################
        # x =     2 * x / scalex - 1.0
        # y = 1 - 2 * y / scaley
        trigger1X =       2.0 * self.trigger1X / sx - 1.0
        trigger1Y = 1.0 - 2.0 * self.trigger1Y / sy
        self.trigger1Vertices = np.array([trigger1X, trigger1Y]).T 
        trigger2X =       2.0 * self.trigger2X / sx - 1.0
        trigger2Y = 1.0 - 2.0 * self.trigger2Y / sy
        self.trigger2Vertices = np.array([trigger2X, trigger2Y]).T 
        trigger3X =       2.0 * self.trigger3X / sx - 1.0
        trigger3Y = 1.0 - 2.0 * self.trigger3Y / sy
        self.trigger3Vertices = np.array([trigger3X, trigger3Y]).T 
        trigger4X =       2.0 * self.trigger4X / sx - 1.0
        trigger4Y = 1.0 - 2.0 * self.trigger4Y / sy
        self.trigger4Vertices = np.array([trigger4X, trigger4Y]).T 

        # pad polygon
        #############
        # The final location depends on the touch position
        #  for now, we put the circle in the middle of image
        cx, cy = sx/2., sy/2.
        rx, ry = self.padRx, self.padRy
        n = 2*(rx+1) + 2*(ry+1)
        self.padVertices = circle_vertices(cx, cy, rx, ry, n, sx, sy)
   
###############################################################################
# Main Window
###############################################################################

class MainWindow(QMainWindow):

    ###########################################################################
    # Init
    ###########################################################################

    def __init__(self, args, logger):
        super(MainWindow, self).__init__()

        self.logger = logger
        self.logger.log(logging.INFO, 'Starting gearVR Controller Viewer...')

        self.zmqPort       = args.zmqport
        self.renderButtons = args.buttons

        # Add the two widgets side by side
        mainLayout = QVBoxLayout()
        secondaryLayout = QHBoxLayout()

        # Create the Main widget
        self.objectRenderingWidget = ObjectRenderingWidget(parent=self, logger=self.logger, \
            position=args.position, debug=args.debug, renderButtons=self.renderButtons)
        mainLayout.addWidget(self.objectRenderingWidget)
                
        # add the image display widget on the center bottom if debug is on
        if args.debug:
            self.imageDisplayWidget = ImageDisplayWidget(logger=self.logger, parent=self)
            if self.imageDisplayWidget is not None:
                secondaryLayout.addWidget(self.imageDisplayWidget)
                self.objectRenderingWidget.newTextureAvailable.connect(self.imageDisplayWidget.receiveTexture) # type: ignore
            
            mainLayout.addLayout(secondaryLayout)
        else:
            self.imageDisplayWidget = None
            
        # Set the main layout as the central widget
        self.central_widget = QWidget(self)
        self.central_widget.setLayout(mainLayout)
        self.setCentralWidget(self.central_widget)
        
        # Set the initial size and position of the window
        if self.imageDisplayWidget is not None:
            self.setGeometry(100, 100, 800, 900)
        else:
            self.setGeometry(100, 100, 800, 600)
            
        self.setWindowTitle("gear VR Controller Viewer")
        
        current_directory = str(pathlib.Path(__file__).parent.absolute())
        path = current_directory + '/gearViewer.png'
        self.setWindowIcon(QIcon(path))

        self.zmqWorker = zmqWorker(logger=self.logger)
        self.zmqWorkerThread = QThread()
        self.zmqWorker.moveToThread(self.zmqWorkerThread)
        self.zmqWorker.dataReady.connect(self.objectRenderingWidget.update_system)
        self.zmqWorker.BLEstatus.connect(self.objectRenderingWidget.update_BLEstatus)
        self.zmqWorker.finished.connect(self.worker_finished)
        self.zmqWorkerThread.started.connect(self.zmqWorker.start)
        self.zmqWorkerThread.finished.connect(self.worker_thread_finished)
        self.zmqWorker.set_zmqPort(args.zmqport)
        self.zmqWorkerThread.start()

        self.logger.log(logging.INFO, 'Main initialized with port: {}'.format(args.zmqport))

    ###########################################################################
    # Start Stop Finish ZMQ Worker
    ###########################################################################

    def stop_worker(self):
        self.zmqWorker.stop()
        self.worker_finished()

    def worker_finished(self):
        self.zmqWorkerThread.quit()
        self.zmqWorkerThread.wait()

    def worker_thread_finished(self):
        self.zmqWorkerThread.deleteLater()
        self.zmqWorker.deleteLater()

###############################################################################
# Main
###############################################################################

if __name__ == '__main__':
    app = QApplication(sys.argv)

    parser = argparse.ArgumentParser(description="gearVRC Viewer")

    parser.add_argument(
        '-z',
        '--zmq',
        dest='zmqport',
        type=str,
        metavar='<zmqport>',
        help='port used by ZMQ, \'tcp://localhost:5556\'',
        default="tcp://localhost:5556"
    )

    parser.add_argument(
        '-p',
        '--position',
        action = 'store_true',
        help='Move sensor when position data is available, not recommended',
        default = False
    )

    parser.add_argument(
        '-d',
        '--debug', 
        action='store_true', 
        help='Debugging on or off, displays texture map',
        default = False
    )

    parser.add_argument(
        '-b',
        '--buttons', 
        action='store_true', 
        help='Render button presses on or off',
        default = True
    )

    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    log_level = logging.DEBUG if args.debug else logging.INFO
    logger.setLevel(log_level)
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    handler = logging.StreamHandler()
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    mainWindow = MainWindow(args=args, logger=logger)
    mainWindow.show()

    sys.exit(app.exec_())
