"""
This is a long, multiline description
"""

#------------------------
##    IMPORT MODULES   ##
#------------------------
import numpy as np
import sys
import os.path
import cv2


#------------------------
##   DATA STRUCTURES   ##
#------------------------

#------------------------
## FUNCTION DEFINITION ##
#------------------------

import ctypes
import pyglet
from pyglet.gl import *
from pyglet.window import mouse
from pywavefront import Wavefront
from pyglet.window import key

#meshes = Wavefront('earth.obj')
#meshes = Wavefront('datasets/red_book2/dataset/1528188687058_book_only.obj')



lightfv = ctypes.c_float * 4
label = pyglet.text.Label('Hello, world', font_name = 'Times New Roman', font_size = 12, x = 800, y = 700, anchor_x = 'center', anchor_y = 'center')


class FrontEnd():
    """ Class for visualization of 3D models
    """

    def __init__(self, initialize = True):
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.sx = 0
        self.sy = 0
        self.sz = -20
        self.tx = self.sx
        self.ty = self.sy
        self.tz = self.sz


        self.initialized = False
        if initialize:
            self.initialize()
            
    def loadMeshObj(self, path_to_mesh):
        self.mesh_obj = Wavefront(path_to_mesh)

    def initialize(self):
        
        #Create the window
        #self.window = pyglet.window.Window(1024, 720, caption = 'Demo', resizable = True)
        self.window = pyglet.window.Window(1024, 980, caption = '3D Model', resizable = False)

        #Setup keyboard state memorization
        self.keys = key.KeyStateHandler()
        self.window.push_handlers(self.keys)

        #Setup callbacks
        self.on_mouse_press = self.window.event(self.on_mouse_press)
        self.on_mouse_scroll = self.window.event(self.on_mouse_scroll)
        self.on_resize = self.window.event(self.on_resize)
        self.on_draw = self.window.event(self.on_draw)
        self.on_mouse_drag = self.window.event(self.on_mouse_drag)
        self.on_key_press = self.window.event(self.on_key_press)

        self.initialized = True


    def on_resize(self,width, height):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(40.0, float(width)/height, 1, 100.0)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_MODELVIEW)
        return True

    def on_draw(self):
        #print("drawing")
    
        self.window.clear()
        glLoadIdentity()
        glLightfv(GL_LIGHT0, GL_POSITION, lightfv(-40, 200, 100, 0.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, lightfv(0.2, 0.2, 0.2, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightfv(0.5, 0.5, 0.5, 1.0))
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)
        glMatrixMode(GL_MODELVIEW)

        #Rotations for sphere on axis - useful
        glTranslated(self.tx, self.ty, self.tz)
        glRotatef(self.roll, 0, 0, 1)
        glRotatef(self.pitch, 1, 0, 0)
        glRotatef(self.yaw, 0, 1, 0)
        self.mesh_obj.draw()

        cv2.waitKey(10)

    def on_mouse_press(self, x, y, button, modifiers):
        if button == mouse.LEFT:
            print('The left mouse button was pressed.')

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.tz += 1 * scroll_y

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        if buttons & mouse.LEFT:

            if self.keys[key.LCTRL]:
                self.roll += 1 * dx
                if self.roll > 360: 
                   self.roll = 0
            else:
                self.pitch -= 1 * dy
                if self.pitch > 360: 
                   self.pitch = 0

                self.yaw += 1 * dx
                if self.yaw > 360: 
                   self.yaw = 0

    def on_key_press(self, symbol, modifiers):
        if symbol == key.Q:
            print('Pressed q: Exiting')
            exit(0)


    #def on_key_release(symbol, modifiers):
        #pass


    def update(self, dt):
        print('Pressed something')
        if keys[key.q]:
            print('Pressed q')

        pass

    def __str__(self):
        return 'nothing for now'

        #mesh = om.TriMesh()
        #om.read_trimesh(mesh, filename='models/office/model/20180221_164439.obj')
        #render(self.mesh, 'wireframe')


        # iterate over all faces
        #for fh in mesh_2.faces():
            #print fh.idx()

        #print(mesh_2.request_face_normals())

        #mesh_2.request_vertex_normals()
        #print(mesh_2.has_vertex_normals())
        #mesh_2.update_normals()
        #mesh_2.request_vertex_texcoords2D()
        #print(mesh_2.has_vertex_texcoords2D())
            
        ## iterate over all adjacent faces
        ## iterate over all faces
        #for fh in mesh_2.faces():
            ##print fh.idx()
            ## iterate over the face's vertices
            #i = 0
            #for vh in mesh_2.fv(fh):
                ##print vh.idx()
                #mesh_2.set_texcoord2D(vh, np.random.random_sample((1, 2))[0])
             
        #print('Loaded ' + str(len(self.cameras)) + ' cameras')

    def visualize3DModel(self):
        if not self.initialized:
            self.initialize()

        pyglet.app.run()


