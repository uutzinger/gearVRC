#!/usr/bin/python3

################################################################
# Samsung gearVR Controller Viewer
################################################################
# Urs Utzinger 2023
#
# This work uses ideas from  Gear VRC reverse engineering: 
#   https://github.com/jsyang/gearvr-controller-webbluetooth
# In particular the idea of updating the texture to highlight.
# 
# pywavefront is used to load the 3D model
# pywavcefront.visulaizer was used to inspect how the 
# loaded models are rendered
# pyglet is used to load the texture image
#
# chat.openai.com was used extensively to learn about OpenGL 
# prorgammin in Python
################################################################
# Prerequisite:
# python packages:
#   pyqt5, pyopengl, pywavefront, pyglet
#   zmq, msgpack, numpy
# data structures and methods defined in 
#   gearVRC.py
################################################################

import sys, os, pathlib, math
import logging, argparse
import zmq, msgpack
import numpy as np

from PyQt5.QtCore import QThread, QObject, pyqtSignal,
from PyQt5.QtWidgets import QApplication, QMainWindow, QOpenGLWidget
from PyQt5.QtGui import QIcon

from OpenGL.GL import glClearColor, glEnable, glClear, glViewport, glMatrixMode, glLoadIdentity, glTranslatef, \
                      glShadeModel, glRotatef, glColor4f, glBegin, glVertex2f, glEnd, \
                      glGenTextures, glBindTexture, glTexImage2D, glDisable, glBlendFunc, glBlendFunc, \
                      glMaterialfv, glMatrixMode, glDrawArrays, glPushClientAttrib, \
                      glPushAttrib, glInterleavedArrays, glPopAttrib, glPopClientAttrib, glMaterialf, \
                      glTexParameterf, glCullFace, glGenFramebuffers, glBindFramebuffer, \
                      glFramebufferTexture2D, glCheckFramebufferStatus, glTexSubImage2D, \
                      glCopyTexImage2D, glTexSubImage2D, glTexParameteri

from OpenGL.GLU import gluLookAt, gluPerspective
from OpenGL.GL import GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_TEXTURE_2D, \
                      GL_LIGHTING, GL_LIGHT0, GL_PROJECTION, GL_MODELVIEW, GL_DEPTH_TEST, \
                      GL_COLOR_MATERIAL, GL_SMOOTH, GL_POLYGON, GLfloat, GL_TRIANGLES, \
                      GL_UNSIGNED_BYTE, GL_RGBA, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE, \
                      GL_TEXTURE_WRAP_T, GL_BLEND, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, \
                      GL_DIFFUSE, GL_AMBIENT, GL_SPECULAR, GL_UNSIGNED_BYTE, GL_REPEAT, \
                      GL_BACK, GL_FRONT_AND_BACK, GL_CLIENT_VERTEX_ARRAY_BIT, GL_CURRENT_BIT, \
                      GL_ENABLE_BIT, GL_LIGHTING_BIT, GL_CULL_FACE, GL_DEPTH_TEST, \
                      GL_BACK, GL_EMISSION, GL_SHININESS, GL_FRAMEBUFFER, \
                      GL_FRAMEBUFFER_COMPLETE, GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1, \
                      GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4, \
                      GL_COLOR_ATTACHMENT5, GL_COLOR_ATTACHMENT6, GL_COLOR_ATTACHMENT7, \
                      GL_UNSIGNED_BYTE, GL_RGBA, GL_TEXTURE_MIN_FILTER, GL_LINEAR, \
                      GL_TEXTURE_MAG_FILTER, GL_LINEAR
  
# Pywavefront is used to load the 3D model
from pywavefront.visualization import VERTEX_FORMATS
from pywavefront import Wavefront

# pyglet is used to load the texture image
from pyglet import image

# Import data structures and methods defined in gearVRC.py to handle the data we receive from the device
sys.path.append('..')
from gearVRC import gearFusionData, gearButtonData, gearMotionData, dict2obj

###################################################################
# GLOBALS
###################################################################
ZMQTIMEOUT      = 1000 # in milliseconds
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
                        self.new_fusion = False
                        self.new_button = False
                        self.new_motion = False

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
        self.texture    = None
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
        
        # Load the 3D object
        modelFileName = os.path.join(os.path.dirname(__file__), 'model/gear_vr_controller.obj')
        # Wavefront was suggested by chat.openai.com for loading obj files.
        # Once loaded the object internal structure is confusing. 
        # For example loading the model does not load the textures. 
        # It does not have option load the material descriptions. 
        # The textures are loaded when the model is rendered. 
        # It uses pyglet image to load the image. 
        # For rendering the object the visualization module access the model "material".
        # Once the model is loaded its vertex, texture and normal are combined in an array for faster rendering.
        self.object = Wavefront(modelFileName, collect_faces=True, create_materials=True)

        # This is copied from visualizer.py in pywavefront:
        material = self.object.materials['controller']
        if material.gl_floats is None:
            material.gl_floats = (GLfloat * len(material.vertices))(*material.vertices)
            material.triangle_count = len(material.vertices) / material.vertex_size
        self.vertex_format = VERTEX_FORMATS.get(material.vertex_format)
                
        # Calculate the camera position and distance based on the object size
        min_bound = np.min(self.object.vertices, axis=0)
        max_bound = np.max(self.object.vertices, axis=0)
        self.object_center = (min_bound + max_bound) / 2.0
        object_size = np.linalg.norm(max_bound - min_bound)
        camera_distance = 2.5 * object_size
        self.camera_position = self.object_center + camera_distance*np.array([0.0, 0.1, 1.0])
        # x is the axis pointing to the right
        # y is the axis pointing up
        # z is the axis pointing towards the user
        
        # Load textures
        load_and_create_textures(self, material)
        
        # Create texture to highlight buttons
        self.create_button_textures()
                
        self.logger.log(logging.INFO, 'GL Widget initialized'  )
        
    def paintGL(self):

        # Default frame buffer and texture for display
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glBindTexture(GL_TEXTURE_2D, 0)
        
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
        #
        # This is commented out as the position calculations are in accurate
        #   and have large run off.
        #
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

            # Blend the button textures based on user interaction with device
            self.update_texture(button)
            ############################################

            # Select the blended texture
            #   the blended texture is now in displayTextureID
            #   not in self.texture.image.id anymore
            glBindTexture(self.texture.image.target, self.displayTextureID)

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
                glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE,   (GLfloat * 4)(*material.diffuse))
                glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT,   (GLfloat * 4)(*material.ambient))
                glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  (GLfloat * 4)(*material.specular))
                glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION,  (GLfloat * 4)(*material.emissive))
                glMaterialf(GL_FRONT_AND_BACK,  GL_SHININESS, min(128.0, material.shininess))
                glEnable(GL_LIGHT0)
                glEnable(GL_LIGHTING)
            else:
                glDisable(GL_LIGHTING)
                glColor4f(*material.ambient)
                        
            glInterleavedArrays(self.vertex_format, 0, material.gl_floats)
            glDrawArrays(GL_TRIANGLES, 0, int(material.triangle_count))

            glPopAttrib()
            glPopClientAttrib()
            
            glDisable(self.texture.image.target)
        ###########################
        
        self.logger.log(logging.DEBUG, 'Scene painted'  )

    def resizeGL(self, width, height):
        # Adjust the viewport and projection matrix
        # Suggested by chat.openai.com
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

    def load_and_create_textures(self, material):
        '''
        Load the object texture from image file which is available once the object is loaded
        The object texture is available in self.texture.image.id
        
        Create a frame buffer and allocate textures.
        
        Draw the content of the buttons onto the frame buffer and store the buffer in the fbo associated textures.
        
        The FBO will have 7 color attachments:
            0: displayTextureID  # will be used for storing the merged texture
            1: backTextureID     # back button
            2: volumeTextureID   # volume button
            3: triggerTextureID  # trigger button
            4: touchTextureID    # touch pad
            5: padTextureID      # touch pad finger
            6: ledTextureID      # led
        '''
        
        ####################################################################################
        # Load the texture from file
        ####################################################################################
        # This is code copied from pywavefront.visualization.py. 
        # It uses pyglet to load the image
        
        self.texture = material.texture or material.texture_ambient

        if self.texture and material.has_uvs:

            textureFileName = material.texture.path
            img = image.load(textureFileName)
            self.texture.image = img.get_texture()
            
            self.textureWidth  = self.texture.image.width
            self.textureHeight = self.texture.image.height
            self.textureID     = self.texture.image.id
                
            glEnable(self.texture.image.target) # This is constant 3553 which is GL_TEXTURE_2D
            glBindTexture(self.texture.image.target, self.texture.image.id)
            if self.texture.options.clamp == "on": # for the gear controller this is on
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            else:
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_S, GL_REPEAT)
                glTexParameterf(self.texture.image.target, GL_TEXTURE_WRAP_T, GL_REPEAT) 
        else: 
            glDisable(self.texture)

        ####################################################################################
        # Create frame buffer and texture for merging and managing textures
        ####################################################################################

        # 1) Create new FBO
        self.blendingFBO = glGenFramebuffers(1)
        
        # 2) Make the FBO the active frame buffer
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)  
        
        # 3) Generate new texture ids
        self.displayTextureID       = glGenTextures(1) # Colorattachment 0
        self.backTextureID          = glGenTextures(1) # Colorattachment 2
        self.volumeTextureID        = glGenTextures(1) # Colorattachment 3
        self.padTextureID           = glGenTextures(1) # Colorattachment 6
        self.touchTextureID         = glGenTextures(1) # Colorattachment 5
        self.triggerTextureID       = glGenTextures(1) # Colorattachment 4
        self.ledTextureID           = glGenTextures(1) # Colorattachment 7
        self.originalTextureID      = glGenTextures(1) # Colorattachment 8
        

        # Repeat for each texture
        #########################
        
        # Display Texture, to be used to render the object 
        
        # 4) Bind the texture to make active texture
        glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
        
        # 5) Allocate memory for the texture and specify its format size, use original texture dimensions
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.textureWidth, self.textureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)

        # 6) Set the texture paramters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            
        # Back Button Texture
        glBindTexture(GL_TEXTURE_2D, self.backTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.backTextureWidth, self.backTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        # Volume Button Texture
        glBindTexture(GL_TEXTURE_2D, self.volumeTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.volumeTextureWidth, self.volumeTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
 
        # Trigger Button Texture
        glBindTexture(GL_TEXTURE_2D, self.triggerTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.triggerTextureWidth, self.triggerTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
 
        # Touch Pad Texture
        glBindTexture(GL_TEXTURE_2D, self.touchTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.touchTextureWidth, self.touchTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        # Touch Pad Finger Texture
        glBindTexture(GL_TEXTURE_2D, self.padTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.padTextureWidth, self.padTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
 
        # LED Texture
        glBindTexture(GL_TEXTURE_2D, self.ledTextureID)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, self.ledTextureWidth, self.ledTextureHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, None)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)

        # 7) Attach the new textures to the FBO
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)  
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, self.displayTextureID, 0)         
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, self.backTextureID, 0)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_2D, self.volumeTextureID, 0)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_2D, self.triggerTextureID, 0)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_2D, self.touchTextureID, 0)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT5, GL_TEXTURE_2D, self.padTextureID, 0)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT6, GL_TEXTURE_2D, self.ledTextureID, 0)
        # also attach the original texture from the object to the FBO
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT7, GL_TEXTURE_2D, self.TextureID, 0)

        # 8) check if the framebuffer is complete
        if glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE:
            raise RuntimeError("Texture frame buffer is not complete!")
        
        ####################################################################################
        # Create the content in the textures
        ####################################################################################

        # Center Coordinates of the controller features on the texture image
        # These coordinates are from https://github.com/jsyang/gearvr-controller-webbluetooth/blob/master/ControllerDisplay.js
        # The coordinates assume 0/0 at the upper left corner of the image
        # However in OpenGL 0/0 is at the lower left corner of the image
        # We will adjust these coordinates later.
        # C is center and R is radius.
        
        self.backCx,  self.backCy            =  24, 18 # Back Button
        self.backRx,  self.backRy            =  20, 20
        self.homeCx,  self.homeCy            = 124, 44 # Home Button
        self.touchCx, self.touchCy           = 197, 60 # Touch Pad
        self.touchRx, self.touchRy           =  30, 30        
        self.padRx,   self.padRy             =  5, 5
        self.volumeUpCx, self.volumeUpCy     = 106, 13 # Volume Up
        self.volumeDownCx, self.volumeDownCy = 140, 13 # Volume Down
        self.volumeRx, self.volumeRy         =  11, 11
        self.triggerX = [113.,138.,152.,154.,154.,140.,117.,125.,122.,111.,122.,108.]
        self.triggerY = [ 61., 61.,113.,170.,230.,256.,256.,220.,163.,120.,163.,101.] 
        self.ledRx, self.ledRy               =  2, 2

        # Activate frame buffer
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)
        # The color used when we clear the frame buffer
        glClearColor(0.0, 0.0, 0.0, 0.0) # set color to black with 0% transparency
        
        # Create Texture for highlighted back and home button
        ######################################################       
        self.backTextureWidth  = 2*self.backRx +1
        self.backTextureHeight = 2*self.backRy +1
        # Set the viewport to match the texture size
        glViewport(0, 0, self.backTextureWidth, self.backTextureHeight)
        # Clear the frame buffer
        glClear(GL_COLOR_BUFFER_BIT)
        # Draw the button onto the frame buffer
        glColor4f(1.0, 0.0, 0.0, 0.5) # set color to red with 50% transparency
        glBegin(GL_POLYGON)
        cx, cy =  self.backRx, self.backRy
        rx, ry =  self.backRx, self.backRy
        num_segments = 64
        for i in range(num_segments + 1):
            angle = TWOPI * (i / num_segments)
            x = cx + rx * math.cos(angle)
            y = cy + ry * math.sin(angle)
            glVertex2f(x, y)
        glEnd()
        # Activate the back texture
        glBindTexture(GL_TEXTURE_2D, self.backTextureID)
        # Copy frame buffer back to the texture
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, self.backTextureWidth, self.backTextureHeight, 0)

        # Create Texture for the highlighted touch pad button
        #####################################################       
        self.touchTextureWidth  = 2*self.touchRx + 1
        self.touchTextureHeight = 2*self.touchRy + 1
        glViewport(0, 0, self.touchTextureWidth, self.touchTextureHeight)
        glClear(GL_COLOR_BUFFER_BIT)
        glColor4f(1.0, 0.0, 0.0, 0.5)
        glBegin(GL_POLYGON)
        cx, cy =  self.touchRx, self.touchRy
        glVertex2f(cx, cy)
        rx, ry =  self.touchRx, self.touchRy
        num_segments = 64
        for i in range(num_segments + 1):
            angle = TWOPI * (i / num_segments)
            x = cx + rx * math.cos(angle)
            y = cy + ry * math.sin(angle)
            glVertex2f(x, y)
        glEnd()
        glBindTexture(GL_TEXTURE_2D, self.touchTextureID)
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, self.touchTextureWidth, self.touchTextureHeight, 0)
                
        # Create Texture for highlighted volume buttons
        ###############################################
        self.volumeTextureWidth  = 2*self.volumeRx + 1
        self.volumeTextureHeight = 2*self.volumeRy + 1
        glViewport(0, 0, self.volumeTextureWidth, self.volumeTextureHeight)
        glClear(GL_COLOR_BUFFER_BIT)
        glColor4f(1.0, 0.0, 0.0, 0.5)
        glBegin(GL_POLYGON)
        cx, cy =  self.volumeRx, self.volumeRy
        glVertex2f(cx, cy)
        rx, ry =  self.volumeRx, self.volumeRy
        num_segments = 64
        for i in range(num_segments + 1):
            angle = TWOPI * (i / num_segments)
            x = cx + rx * math.cos(angle)
            y = cy + ry * math.sin(angle)
            glVertex2f(x, y)
        glEnd()
        glBindTexture(GL_TEXTURE_2D, self.volumeTextureID)
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, self.touchTextureWidth, self.touchTextureHeight, 0)

        # Create Texture for highlighting finger position on touch pad
        ##############################################################
        self.padTextureWidth  = 2*self.padRx + 1
        self.padTextureHeight = 2*self.padRy + 1
        glViewport(0, 0, self.padTextureWidth, self.padTextureHeight)
        glBindFramebuffer(GL_FRAMEBUFFER, self.padFBO)
        glClear(GL_COLOR_BUFFER_BIT)
        glColor4f(1.0, 0.0, 0.0, 0.5)
        glBegin(GL_POLYGON)
        cx, cy =  self.padRx, self.padRy
        glVertex2f(cx, cy)
        rx, ry =  self.padRx, self.padRy
        num_segments = 64
        for i in range(num_segments + 1):
            angle = TWOPI * (i / num_segments)
            x = cx + rx * math.cos(angle)
            y = cy + ry * math.sin(angle)
            glVertex2f(x, y)
        glEnd()
        glBindTexture(GL_TEXTURE_2D, self.padTextureID)
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, self.padTextureWidth, self.padTextureHeight, 0)

        # Create Texture for Bluetooth LED
        ##################################
        self.ledTextureWidth  = 2*self.ledRx + 1
        self.ledTextureHeight = 2*self.ledRy + 1
        glViewport(0, 0, self.ledTextureWidth, self.ledTextureHeight)
        glClear(GL_COLOR_BUFFER_BIT)
        glColor4f(0.0, 0.0, 1.0, 1.0)
        glBegin(GL_POLYGON)
        cx, cy =  self.ledRx, self.ledRy
        glVertex2f(cx, cy)
        rx, ry =  self.ledRx, self.ledRy
        num_segments = 4
        for i in range(num_segments + 1):
            angle = TWOPI * (i / num_segments)
            x = cx + rx * math.cos(angle)
            y = cy + ry * math.sin(angle)
            glVertex2f(x, y)
        glEnd()
        glBindTexture(GL_TEXTURE_2D, self.ledTextureID)
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, self.ledTextureWidth, self.ledTextureHeight, 0)

        # Create Texture for highlighted Trigger
        ########################################
        # We will need to take care of the flipped coordinates in y direction
        self.triggerXMin = np.min(np.array(self.triggerX))
        self.triggerXMax = np.max(np.array(self.triggerX))
        self.triggerYMin = np.min(np.array(self.triggerY))
        self.triggerYMax = np.max(np.array(self.triggerY))
        self.triggerTextureWidth  = self.triggerXMax - self.triggerXMin + 1
        self.triggerTextureHeight = self.triggerYMax - self.triggerYMin + 1
        glViewport(0, 0, self.triggerTextureWidth, self.triggerTextureHeight)
        glClear(GL_COLOR_BUFFER_BIT)
        glColor4f(1.0, 0.0, 0.0, 0.5)
        glBegin(GL_POLYGON)
        for vx, vy in zip(self.triggerX, self.triggerY):
            x = vx - self.triggerXMin
            # flip y coordinates
            # y = self.triggerTextureHeight - (vy-offsetY) - 1 # flipped y coordinate
            # This is simplified to:
            y = self.triggerYMax - vy
            glVertex2f(x, y)
        glEnd()
        glBindTexture(GL_TEXTURE_2D, self.triggerTextureID)
        glCopyTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 0, 0, self.ledTextureWidth, self.ledTextureHeight, 0)

        # 9) unbind the FBOs and textures
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glBindTexture(GL_TEXTURE_2D, 0)
        
        # 10) Create and compile vertex and fragment shaders for the copy operation.

        vertex_shader = "\
#version 330 core

layout (location = 0) in vec2 position;
layout (location = 1) in vec2 texCoord;

out vec2 TexCoord;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    TexCoord = texCoord;
}"

    fragement_shader = "\

# version 330 core

in vec2 TexCoord;

uniform sampler2D originalTexture;

out vec4 fragColor;

void main()
{
    fragColor = texture(originalTexture, TexCoord);
}"

    veretex_shader = "\
#version 330 core

layout (location = 0) in vec2 position;
layout (location = 1) in vec2 texCoord;

out vec2 TexCoord;

void main()
{
    gl_Position = vec4(position, 0.0, 1.0);
    TexCoord = texCoord;
}"

    fragement_shader = "\
#version 330 core

in vec2 TexCoord;

uniform sampler2D targetTexture;
uniform sampler2D smallerTextures[6]; // Array to hold smaller textures

out vec4 fragColor;

void main()
{
    vec4 targetColor = texture(targetTexture, TexCoord);
    
    // Blend smaller textures on top of the target color by adding their content
    for (int i = 0; i < 6; ++i) {
        targetColor += texture(smallerTextures[i], TexCoord);
    }
    
    fragColor = targetColor;
}""
        

    def update_texture(self, button):
        '''
        Update the texture based on the button state
        
        In each iteration
            - copy the original texture into the display texture in the displayFBO frame buffer
            - render the button texture onto the fbo frame buffer with blending at appropriate positions
            - copy the fbo frame buffer to the display texture

        The coordinates for the objects location in the texture assume 0/0 at top left corner of the image
        However in OpenGL 0/0 is at the bottom left corner of the image
        We will need to adjust accordingly
        
        How to blend the textures when new texture pixel data is in CPU memory:
        =======================================================================        
        A) Initialize the final texture
            1) Make the display texture the active target 
            glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
            2) glTexImage2D is used to update the entire texture with the new pixel data
                (target=GL_TEXTURE_2D, level=0, internalformat=GL_RGBA, width, height, border=0,
                format=GL_RGBA, type=GL_UNSIGNED_BYTE, pixels=the new pixel data)
               glTexSubIMage2D is faster as it does not need to allocate memory

        B) blend other texture onto the final texture
            1) Enable blending: glEnable(GL_BLEND)
            2) Define the blend function: gleBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            3) Define the blend equation: glBlendEquation(GL_FUNC_ADD)
            4) glTexSubImage2D is used to update portions of an existing texture with the new pixel data
                (target=GL_TEXTURE_2D, level=0, xoffset, yoffset, width, height, format=GL_RGBA, 
                type=GL_UNSIGNED_BYTE, pixels=the new pixel data)
                coordinates are absolute.
            5) repeat for other textures
            6) Disable blending with: glDisable(GL_BLEND)
            
            
        How to blend the textures when texture is in GPU memory:
        ========================================================
        This is called render to texture or offscreen rendering
        
        A) Create FBO and attach the target texture I want to update as color attachment
        B) Attach the original texture as an other color attachment to the same FBO
        C) Attach the other textures as additional color attachments to the same FBO
        
        D) Set blending or rendering operations using shaders.
            You can use custom fragment shaders to define the blending operations.
            You can access the original texture and other textures using texture sampling.
            1) glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        E) Bind the FBO as the active frame buffer
            1) glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)
        F) Render a full screen quad (two triangles covering the entire viewport) with the shader program we setup.
           this will execute the rendering operations defined in the shader program 
            1) Set the output of the shader to be the color attachment corresponding to the target texture
        G) Unbind the FBO
            1) glBindFramebuffer(GL_FRAMEBUFFER, 0)
        H) The target texture attached to the FBO is updated
        
        '''
        
        # Lets attempt to do this with the texture in GPU memory
        # All texture are attached to blendingFBO
                
        # 1) Activate the FBO to make it current frame buffer
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO) 

        # 2) Set the viewport to match the original texture size
        glViewport(0, 0, self.textureWidth, self.textureHeight)  # Set the viewport to match the texture size
        glClear(GL_COLOR_BUFFER_BIT)
        
        glUseProgram(copyShaderProgramID)
        
        
        # 2) Copy original texture to a new texture
        ###########################################
        glBindTexture(GL_TEXTURE_2D, self.textureID)
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, self.textureID, 0)

        glBlitFramebuffer(
            0, 0, self.textureWidth, self.textureHeight,  # Source region (entire texture)
            0, 0, self.textureWidth, self.textureHeight,  # Destination region (entire framebuffer)
            GL_COLOR_BUFFER_BIT, GL_NEAREST  # Blit the color buffer using nearest neighbor filtering
        )
        
        # Define the blend function
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glBlendEquation(GL_FUNC_ADD)
        
        if button.back:
            # Calculate the offset and width/height of the back button in relative coordinates of the display texture
            # We need -1 because the the height is one pixel larger than the location of last minus the first pixel
            oX =                                  (self.backCx - self.backRx)  / self.displayTextureWidth
            oY = (self.displayTextureHeight - 1 - (self.backCy + self.backRy)) / self.displayTextureHeight
            w  = self.backTextureWidth  / self.displayTextureWidth
            h  = self.backTextureHeight / self.displayTextureHeight
            # C) Bind the texture of the back button
            glBindTexture(GL_TEXTURE_2D, self.backTexture)
            # E) Use glTexSubImage2D to update the displayFBO texture with the backButton texture
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)
            
        if button.home:
            # uses same FBO as back button
            oX =                                  (self.homeCx - self.backRx)  / self.displayTextureWidth
            oY = (self.displayTextureHeight - 1 - (self.homeCy + self.backRy)) / self.displayTextureHeight
            w  = self.backTextureWidth  / self.displayTextureWidth
            h  = self.backTextureHeight / self.displayTextureHeight
            glBindTexture(GL_TEXTURE_2D, self.homeTexture)
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)

        if button.volume_up:
            oX =                                  (self.volumeUpCx - self.volumeRx)  / self.displayTextureWidth
            oY = (self.displayTextureHeight - 1 - (self.volumeUpCy + self.volumeRy)) / self.displayTextureHeight
            w  = self.volumeTextureWidth  / self.displayTextureWidth
            h  = self.volumeTextureHeight / self.displayTextureHeight
            glBindTexture(GL_TEXTURE_2D, self.volumeUpTexture)
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)

        if button.volume_down:
            # uses same FBO as volume up button
            oX =                                  (self.volumeDownCx - self.volumeRx)  / self.displayTextureWidth
            oY = (self.displayTextureHeight - 1 - (self.volumeDownCy + self.volumeRy)) / self.displayTextureHeight
            w  = self.volumeTextureWidth  / self.displayTextureWidth
            h  = self.volumeTextureHeight / self.displayTextureHeight
            glBindTexture(GL_TEXTURE_2D, self.volumeDownTexture)
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)

        if button.touch:
            oX =                                  (self.touchCx - self.touchRx)  / self.displayTextureWidth
            oY = (self.displayTextureHeight - 1 - (self.touchCy + self.touchRy)) / self.displayTextureHeight
            w  = self.touchTextureWidth  / self.displayTextureWidth
            h  = self.touchTextureHeight / self.displayTextureHeight
            glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)

        if button.trigger:
            oX =                                self.triggerXMin / self.displayTextureWidth
            oY = (self.displayTextureHeigh -1 - self.triggerYMax) / self.displayTextureHeight
            w  = self.triggerTextureWidth  / self.displayTextureWidth
            h  = self.triggerTextureHeight / self.displayTextureHeight
            glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)
            
        if button.touchX > 0 and button.touchY > 0:
            # touch pad coordinates are in the range of 0 to 315
            # top left corner is 0,0
            # The texture image is 60x60
            # We create circle of radius 5 (padRx)
            # Absolute coordinates of the touch locations on the texture are:
            tCy = self.touchCx + (button.touchX - 157.5) / 157.5 * 30
            tCx = self.touchCy + (button.touchX - 157.5) / 157.5 * 30
            # relative coordinates of the lower left corner where touch texture needs to be copied to
            oX =                                 (tCx - self.padRx)  / self.displayTextureWidth
            oY = (self.displayTextureHeigh - 1 - (tCy + self.padRy)) / self.displayTextureHeight
            w  = self.padTextureWidth / self.displayTextureWidth
            h  = self.padTextureHeight / self.displayTextureHeight
            glBindTexture(GL_TEXTURE_2D, self.displayTextureID)
            glTexSubImage2D(GL_TEXTURE_2D, 0, oX, oY, w, h, GL_RGBA, GL_UNSIGNED_BYTE, None)
            glDisable(GL_BLEND)

        # Save the frame buffer to texture
        glBindFramebuffer(GL_FRAMEBUFFER, self.blendingFBO)
        GL_FRAMEBUFFER_COMPLETE
        
        # 11) Unbind the FBO and texture
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glBindTexture(GL_TEXTURE_2D, 0)

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