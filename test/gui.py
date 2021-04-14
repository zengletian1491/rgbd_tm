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

import rgbd_tm.rgbd_tm as rgbd_tm

#-------------------------------------------------------------------------------
#--- HEADER
#-------------------------------------------------------------------------------
__author__ = "Miguel Riem de Oliveira"
__date__ = "December 2016"
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
    
    #texture_mapper = rgbd_tm.TextureMapper(mesh_filename = args['mesh_filename'], path_to_images = args['path_to_images'], output_mesh_filename = args['output_mesh_filename'], path_to_intrinsics = args['path_to_intrinsics'], z_inconsistency_threshold = args['z_inconsistency_threshold'], view_z_buffering=args['view_z_buffering'], view_range_image=args['view_range_image'], selection_method=args['selection_method'])
    texture_mapper = rgbd_tm.TextureMapper()



    #---------------------------------------
    #--- USING THE USER INTERFACE
    #---------------------------------------


    #---------------------------------------
    #--- USING THE API
    #---------------------------------------


