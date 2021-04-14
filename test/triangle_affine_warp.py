#!/usr/bin/env python
"""
This is a long, multiline description
"""

#-------------------------------------------------------------------------------
#--- Add rgbd_tm module to the sys.path
#--- Will probably have to be done for the code which is using this API
#-------------------------------------------------------------------------------
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

#-------------------------------------------------------------------------------
#--- IMPORTS (standard, then third party, then my own modules)
#-------------------------------------------------------------------------------
import argparse  #to read command line arguments
import glob      #to list images in a folder
import numpy as np
import cv2


#-------------------------------------------------------------------------------
#--- HEADER
#-------------------------------------------------------------------------------
__author__ = "Miguel Riem de Oliveira"
__date__ = "July 2018"
__copyright__ = "Miguel Riem de Oliveira"
__credits__ = ["Miguel Riem de Oliveira"]
__license__ = ""
__version__ = "1.0"
__maintainer__ = "Miguel Oliveira"
__email__ = "m.riem.oliveira@gmail.com"
__status__ = "Development"

#-------------------------------------------------------------------------------
#--- FUNCTIONS
#-------------------------------------------------------------------------------


#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------

# Read input image and convert to float
img_in = cv2.imread("robot.png")
 
# Output image is set to white
img2 = 255 * np.ones(img_in.shape, dtype = img_in.dtype)
   
# Define input and output triangles 
tri1 = np.float32([[[360,200], [760,250], [450,600]]])
tri2 = np.float32([[[360,100], [760,150], [450,300]]])

# Find bounding box. 
r1 = cv2.boundingRect(tri1)
r2 = cv2.boundingRect(tri2)

# Offset points by left top corner of the 
# respective rectangles

tri1Cropped = []
tri2Cropped = []

for i in xrange(0, 3):
    tri1Cropped.append(((tri1[0][i][0] - r1[0]),(tri1[0][i][1] - r1[1])))
    tri2Cropped.append(((tri2[0][i][0] - r2[0]),(tri2[0][i][1] - r2[1])))

# Apply warpImage to small rectangular patches
img1Cropped = img_in[r1[1]:r1[1] + r1[3], r1[0]:r1[0] + r1[2]]

# Given a pair of triangles, find the affine transform.
warpMat = cv2.getAffineTransform( np.float32(tri1Cropped), np.float32(tri2Cropped) )

# Apply the Affine Transform just found to the src image
img2Cropped = cv2.warpAffine( img1Cropped, warpMat, (r2[2], r2[3]), None, flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101 )

# Get mask by filling triangle
mask = np.zeros((r2[3], r2[2], 3), dtype = np.float32)
cv2.fillConvexPoly(mask, np.int32(tri2Cropped), (1.0, 1.0, 1.0), 16, 0);
 
# Apply mask to cropped region
img2Cropped = img2Cropped * mask
 
# Copy triangular region of the rectangular patch to the output image
img2[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = img2[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] * ( (1.0, 1.0, 1.0) - mask )
img2[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = img2[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] + img2Cropped


#Draw triangles

x0, y0 = tri1[0][0]
x1, y1 = tri1[0][1]
cv2.line(img_in, (x0, y0), (x1, y1), (255,0,0), 2) 

x0, y0 = tri1[0][1]
x1, y1 = tri1[0][2]
cv2.line(img_in, (x0, y0), (x1, y1), (255,0,0), 2) 

x0, y0 = tri1[0][2]
x1, y1 = tri1[0][0]
cv2.line(img_in, (x0, y0), (x1, y1), (255,0,0), 2) 


cv2.namedWindow('img_in', cv2.WINDOW_NORMAL)
cv2.imshow('img_in', img_in)


cv2.namedWindow('img2', cv2.WINDOW_NORMAL)
cv2.imshow('img2', img2)


cv2.waitKey(0)


