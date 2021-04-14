#!/usr/bin/env python
"""This script shows another example of using the PyWavefront module."""
# This example was created by intrepid94
import sys
sys.path.append('..')
import ctypes

import pyglet
from pyglet.gl import *
from pyglet.window import mouse
from pywavefront import Wavefront

roll = 0
pitch = 0
yaw = 0
sx = 0
sy = 0
sz = -20
tx = sx
ty = sy
tz = sz

#meshes = Wavefront('earth.obj')
meshes = Wavefront('models/office/model/20180221_164439.obj')

window = pyglet.window.Window(1024, 720, caption = 'Demo', resizable = True)

from pyglet.window import key
keys = key.KeyStateHandler()
window.push_handlers(keys)

lightfv = ctypes.c_float * 4
label = pyglet.text.Label('Hello, world', font_name = 'Times New Roman', font_size = 12, x = 800, y = 700, anchor_x = 'center', anchor_y = 'center')
@window.event
def on_resize(width, height):
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(40.0, float(width)/height, 1, 100.0)
    glEnable(GL_DEPTH_TEST)
    glMatrixMode(GL_MODELVIEW)
    return True

@window.event
def on_draw():
    global roll
    global pitch
    global yaw
    global tx
    global ty
    global tz

    window.clear()
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
    glTranslated(tx, ty, tz)
    glRotatef(roll, 0, 0, 1)
    glRotatef(pitch, 1, 0, 0)
    glRotatef(yaw, 0, 1, 0)
    meshes.draw()

#@window.event
#def on_mouse_press(x, y, button, modifiers):
    #if button == mouse.LEFT:
        #print('The left mouse button was pressed.')

@window.event
def on_mouse_scroll(x, y, scroll_x, scroll_y):
    global tz
    tz += 1 * scroll_y

@window.event
def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    if buttons & mouse.LEFT:

        if keys[key.LCTRL]:
            global roll
            roll += 1 * dx
            if roll > 360: 
               roll = 0
        else:
            global pitch
            pitch += 1 * dy
            if pitch > 360: 
               pitch = 0
            global yaw
            yaw += 1 * dx
            if yaw > 360: 
               yaw = 0

#@window.event
#def on_key_press(symbol, modifiers):
## Check if the spacebar is currently pressed:
#if keys[key.SPACE]:
    #pass


#@window.event
#def on_key_release(symbol, modifiers):
    #pass


def update(dt):
    pass
    #global rotation
    #rotation += 45 * dt
    #if rotation > 720: 
       #rotation = 0

pyglet.clock.schedule(update)

pyglet.app.run()
