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

import argparse  # to read command line arguments
#to list images in a folder
import glob
import json
import shutil
import yaml
from collections import namedtuple
from copy import deepcopy
from matplotlib import cm
import openmesh as om
from tqdm import tqdm
from itertools import combinations
from scipy.spatial import distance  #Compute euclidean distance
from scipy.sparse import lil_matrix 
from scipy.optimize import least_squares

from rgbd_tm.rgbd_camera import *

# ------------------------
##   DATA STRUCTURES   ##
# ------------------------
global image_to_display

# ------------------------
## FUNCTION DEFINITION ##
# ------------------------

# ------------------------
###   BASE CLASSES    ###
# ------------------------

# ------------------------
### CLASS DEFINITION  ###
# ------------------------

class Inspector():
    """ Class to color correct
    """

    def __init__(self):
        """Implements a color corrector
           Args: 
           Returns:
              None for now 
        """
        # ---------------------------------------
        # --- Save command line arguments to internal variables
        # --- Convert the parameters to the correct types
        # ---------------------------------------
        ap = argparse.ArgumentParser()
        ap.add_argument("-p", "--path_to_images", help="path to the folder that contains the images", required=True)
        ap.add_argument("-i", "--path_to_intrinsics", help="path to intrinsics yaml file", required=True)
        ap.add_argument("-si", "--skip_images", help="skip images. Useful for fast testing", type=int, default=1)
        ap.add_argument("-ext", "--image_extension", help="extension of the image files, e.g., jpg or png", default='jpg')



        self.p = vars(ap.parse_args())
        print(self.p)
        #exit(0)

        # -------------------------------------#
        ### OpenGL Visualizer
        # -------------------------------------#

        # -------------------------------------#
        ### Configure the pinhole camera object
        # -------------------------------------#
        ci = self.cameraInfoFromYaml(self.p['path_to_intrinsics'])
        # for i in range(0, len(ci.D)): # We use the rectified image, i.e. distortion coeffs = 0
        # ci.D[i] = 0

        print('Loaded intrinsics from file ' + self.p['path_to_intrinsics'])
        # print(ci)

        # ---------------------------------------
        # --- Load Cameras
        # ---------------------------------------
        self.readCameras(ci, None, self.p['skip_images'])
        for camera in self.cameras:
            #camera.rgb.show(wait_for_key = True)
            camera.depth.matrix[0, :] = [1, 0, 0, 0]
            camera.depth.matrix[1, :] = [0, 0, 1, 0]
            camera.depth.matrix[2, :] = [0, -1, 0, 0]
            camera.depth.matrix[3, :] = [0, 0, 0, 1]
            #print('camera.depth.vertices')
            #print(camera.depth.vertices.shape)
            pass

        print('Loaded ' + str(len(self.cameras)) + ' cameras from folder ' + self.p['path_to_images'])
        self.p['num_cameras'] = len(self.cameras)

        #---------------------------------------
        #--- Load color corrected images
        #---------------------------------------
        
        for camera in self.cameras:
            image_file = os.path.dirname(camera.rgb.filename) + '/color_correction/' +  os.path.basename(camera.rgb.filename)
            print('Loading corrected image: ' + image_file)
            camera.rgb.color_corrected = cv2.imread(image_file, 1)
        

        #---------------------------------------
        #--- Start inspector
        #---------------------------------------


        
        cv2.namedWindow('Inspector', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('image number', 'Inspector' ,0,len(self.cameras)-1, self.nothing)
        cv2.createTrackbar('original vs corrected', 'Inspector' ,0,10, self.nothing)

        while True:
            key = cv2.waitKey(3)
            idx = cv2.getTrackbarPos('image number', 'Inspector')
            balance = float(cv2.getTrackbarPos('original vs corrected', 'Inspector'))/10.0


            orig_image = self.cameras[idx].rgb.image.astype(np.float)/255
            
            corrected_image = self.cameras[idx].rgb.color_corrected.astype(np.float)/255

            print('balance = ' + str(balance))
            image = cv2.addWeighted(orig_image, balance, corrected_image, (1-balance), 0)
            cv2.imshow('Inspector', image)


            if key == ord('c'):
                print('Pressed "c". Continuing.')
                break
            elif key == ord('q'):
                print('Pressed "q". Aborting.')
                exit(0)


    def nothing(self):
        print('Doing nothing')
        pass


        #cv2.createTrackbar("Max", "Colorbars",0,255,nothing) 

    def cameraInfoFromYaml(self, calib_file):
        """Loads a camera_info.yaml file. I created one by hand using the TangoRosStreamer app and rostopic echoing the messages. Then I just created the yaml file
           https://answers.ros.org/question/12193/how-to-get-and-use-camera-calibration-file/
           https://answers.ros.org/question/248563/publishing-camerainfo-messages-from-yaml-file/
        """

        # Load data from file
        with open(calib_file, "r") as file_handle:
            calib_data = yaml.load(file_handle)

        # Parse
        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def readCameras(self, rgb_camera_info, depth_camera_info=None, image_skip=1):
        """Produces a list of cameras from image files and txt files"""

        image_files = sorted(
            glob.glob(self.p['path_to_images'] + '/*.' + self.p['image_extension']))  # get a list of image files

        self.cameras = []
        for image_file in image_files[::image_skip]:
            camera_name = image_file[-7:-4]

            filename_txt = image_file[:-4] + '.txt'
            camera_matrix, depth_matrix, device_matrix, stamp = self.readOCTextFile(filename_txt)

            point_cloud_file = image_file[:-4] + '.ply'
            point_cloud = om.read_trimesh(point_cloud_file,
                                          binary=False,
                                          msb=False,
                                          lsb=False,
                                          swap=False,
                                          vertex_normal=False,
                                          vertex_color=False,
                                          vertex_tex_coord=False,
                                          halfedge_tex_coord=False,
                                          edge_color=False,
                                          face_normal=False,
                                          face_color=False,
                                          color_alpha=False,
                                          color_float=False)

            # print('Loaded point_cloud from file ' + point_cloud_file + '\nPC has ' + str(point_cloud.n_vertices()) + ' vertices.')

            vertices = np.array(point_cloud.points(), dtype=np.float).transpose()
            _, n_pts = vertices.shape
            vertices = np.vstack([vertices, np.ones((n_pts,), dtype=np.float)])  # homogeneize

            self.cameras.append(RGBDCameraT(camera_name, 'map',
                                            cv2.imread(image_file, 1), camera_matrix, 'cam' + image_file[:-4], stamp,
                                            rgb_camera_info, image_file,
                                            None, depth_matrix, 'depth' + image_file[:-4], stamp, depth_camera_info,
                                            point_cloud_file, vertices))

    def readOCTextFile(self, filename):
        """Reads the text files produced by the Open Constructor tool
        """
        data = open(filename, 'r').read().split('\n')

        tcamera = np.transpose(np.array([x.split(' ') for x in data[1:5]], dtype=np.float))
        tdepth = np.transpose(np.array([x.split(' ') for x in data[5:9]], dtype=np.float))
        tdevice = np.transpose(np.array([x.split(' ') for x in data[9:13]], dtype=np.float))
        stamp = float(data[13])

        return tcamera, tdepth, tdevice, stamp

    def createFolder(self, path, overwrite=False):
        d = os.path.dirname(path)
        if not self.folderExists(path):
            print("Creating folder " + path)
            os.makedirs(d)
        elif overwrite:
            print("Overwriting older " + path)
            shutil.rmtree(path)
            os.makedirs(d)

    def folderExists(self, path):
        print(path)
        d = os.path.dirname(path)
        if os.path.exists(d):
            print('cc')
            return True
        else:
            print('ee')
            return False

    def fileExists(self, name):
        if os.path.isfile(name):
            return True
        else:
            return False

    def keyPressManager(self):
        print('keyPressManager.\nPress "c" to continue or "q" to abort.')
        while True:
            key = cv2.waitKey(3)
            if key == ord('c'):
                print('Pressed "c". Continuing.')
                break
            elif key == ord('q'):
                print('Pressed "q". Aborting.')
                exit(0)




#-------------------------------------------------------------------------------
#--- MAIN
#-------------------------------------------------------------------------------
if __name__ == "__main__":

    inspector = Inspector()



