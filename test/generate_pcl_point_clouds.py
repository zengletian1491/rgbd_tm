"""
This is a long, multiline description
"""

#import argparse  # to read command line arguments
#import cPickle as pickle
##to list images in a folder
#import glob
#import json
#import os  # we use os.path
#import os.path
#import shutil
#import yaml
#from collections import namedtuple
#from copy import deepcopy
#from math import sqrt, exp
#from matplotlib import cm
#from random import random
#from shapely.geometry import Polygon, Point, mapping, LineString  # for the z buffering
#import openmesh as om
#from numpy import amax, amin, amin
#from numpy.core.multiarray import ndarray
#from sensor_msgs.msg import CameraInfo
#from tqdm import tqdm
#from model_viewer import *
#from rgbd_camera import *

import argparse
import sys
import os
from PIL import Image
# ------------------------
##   DATA STRUCTURES   ##
# ------------------------

# ------------------------
## FUNCTION DEFINITION ##
# ------------------------

#focalLength = 938.0
#centerX = 319.5
#centerY = 239.5
#scalingFactor = 5000

def generate_pointcloud(rgb_file,depth_file,ply_file, focalLength, centerX, centerY, scalingFactor):

    rgb = Image.open(rgb_file)
    depth = Image.open(depth_file).convert('I')

    if rgb.size != depth.size:
        raise Exception("Color and depth image do not have the same resolution.")
    if rgb.mode != "RGB":
        raise Exception("Color image is not in RGB format")
    if depth.mode != "I":
        raise Exception("Depth image is not in intensity format")

    points = []    
    for v in range(rgb.size[1]):
        for u in range(rgb.size[0]):
            color = rgb.getpixel((u,v))
            Z = depth.getpixel((u,v)) / scalingFactor
            print(Z)
            if Z==0: continue
            X = (u - centerX) * Z / focalLength
            Y = (v - centerY) * Z / focalLength
            #points.append("%f %f %f %d %d %d 0\n"% X, Y, Z)
            points.append("%f %f %f\n"% X, Y, Z)

# ------------------------
###   BASE CLASSES    ###
# ------------------------

# ------------------------
### CLASS DEFINITION  ###
# ------------------------



#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    #---------------------------------------
    #--- Parse command line argument
    #---------------------------------------
    #ap = argparse.ArgumentParser()
    #ap.add_argument("-p", "--path_to_images", help = "path to the folder that contains the images", required=True)
    #ap.add_argument("-m", "--mesh_filename", help = "full filename to input obj file, i.e. the 3D model", required=True)
    #ap.add_argument("-i", "--path_to_intrinsics", help = "path to intrinsics yaml file", required=True)
    #ap.add_argument("-o", "--output_mesh_filename", help = "full filename of output obj file", required=True)
    #ap.add_argument("-z", "--z_inconsistency_threshold", help = "threshold value for max z inconsistency value", required=False, type=float, default = 0.05)
    #ap.add_argument("-vzb", "--view_z_buffering", help = "visualize z-buffering", action='store_true', default = False)
    #ap.add_argument("-vri", "--view_range_image", help = "visualize sparse and dense range images", action='store_true', default = False)
    #ap.add_argument("-sm", "--selection_method", help = "method for selecting the cameras for each triangle", type=int, default = 0)
    #args = vars(ap.parse_args())

    #---------------------------------------
    #--- INITIALIZATION
    #---------------------------------------
    

