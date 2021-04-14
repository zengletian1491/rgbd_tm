#!/usr/bin/env python
from pyglet.gl import *
#from Board import Board

class Frontend(pyglet.window.Window):
    def __init__(self, xs, ys):
        #self.GameInstance = Board(xs,ys)
        super(Frontend, self).__init__(width=512, height=512,visible=False)

    def on_draw(self):
        self.clear()


fe = Frontend(500, 600)


