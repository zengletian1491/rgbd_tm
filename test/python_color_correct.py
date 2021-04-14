#!/usr/bin/env python
from __future__ import division
import os
import sys
import cv2
import numpy as np
 
imrgb = np.float32(cv2.imread(sys.argv[1])) / 255.0
imyuv = cv2.cvtColor(imrgb, cv2.COLOR_BGR2YCrCb)
 
imyuv[:,:,(1,2)] -= 0.5
 
cv2.namedWindow("controls")
cv2.namedWindow("original", cv2.WINDOW_NORMAL)
cv2.namedWindow("corrected1", cv2.WINDOW_NORMAL)
cv2.namedWindow("corrected2", cv2.WINDOW_NORMAL)
 
mousepos = None
 
def onmouse(event, x, y, flags, userdata):
    global mousepos
    if event == cv2.EVENT_LBUTTONDOWN:
        print "onmouse", x, y
        mousepos = (x,y)
 
cv2.setMouseCallback("corrected1", onmouse)
cv2.setMouseCallback("corrected2", onmouse)
 
 
y_scale = 1.0
y_bias = 0
u_bias = 0
v_bias = 0
 
def on_y_scale(newpos):
    global y_scale
    y_scale = newpos / 100
 
def on_y_bias(newpos):
    global y_bias
    y_bias = (newpos-50) / 200
 
def on_u_bias(newpos):
    global u_bias
    u_bias = (newpos-50) / 200
 
def on_v_bias(newpos):
    global v_bias
    v_bias = (newpos-50) / 200
 
cv2.createTrackbar("y scale", "controls", 100, 200, on_y_scale)
cv2.createTrackbar("y bias", "controls", 50, 100, on_y_bias)
cv2.createTrackbar("u bias", "controls", 50, 100, on_u_bias)
cv2.createTrackbar("v bias", "controls", 50, 100, on_v_bias)
 
def correct_shit():
    res = imyuv.copy()
    res[:,:,1] += u_bias
    res[:,:,2] += v_bias
    res[:,:,:] *= y_scale
    res[:,:,0] += y_bias
    return res
 
def correct_good():
    res = imyuv.copy()
    res[:,:,1] += u_bias * res[:,:,0]
    res[:,:,2] += v_bias * res[:,:,0]
    res[:,:,:] *= y_scale
    res[:,:,0] += y_bias
    return res
 
while True:
    if mousepos is not None:
        (mx,my) = mousepos
        (py,pu,pv) = imyuv[my, mx, :]
        u_bias = -pu / py
        v_bias = -pv / py
        print "u bias", u_bias
        print "v bias", v_bias
        mousepos = None
        cv2.setTrackbarPos("u bias", "controls", int(0.5 + 50 + u_bias*200))
        cv2.setTrackbarPos("v bias", "controls", int(0.5 + 50 + v_bias*200))
     
    cv2.imshow("original", imrgb)

    imyuv1 = correct_shit()
    imyuv1[:,:,(1,2)] += 0.5 # bias
    rgb = cv2.cvtColor(imyuv1, cv2.COLOR_YCrCb2BGR)
    cv2.imshow("corrected1", rgb)
 
    imyuv2 = correct_good()
    imyuv2[:,:,(1,2)] += 0.5 # bias
    rgb = cv2.cvtColor(imyuv2, cv2.COLOR_YCrCb2BGR)
    print('rgb type ' + str(rgb.dtype))
    cv2.imshow("corrected2", rgb)
 
    print('y_scale ' + str(y_scale))
    print('y_bias ' + str(y_bias))
    print('v_bias ' + str(v_bias))
    print('u_bias ' + str(u_bias))

    key = cv2.waitKey(100)
    if key == -1: continue
    if key == 27: break
 
cv2.destroyAllWindows()
