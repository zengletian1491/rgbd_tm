"""
This is a long, multiline description
"""

#-------------------------------------------------------------------------------
#--- Add  module to the sys.path
#--- Will probably have to be done for the code which is using this API
#-------------------------------------------------------------------------------
#import sys
#import os.path
#sys.path.append(os.path.join(os.path.dirname(__file__), '..'))


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
from model_viewer import *
from rgbd_camera import *
from itertools import combinations
from scipy.spatial import distance  #Compute euclidean distance
from scipy.sparse import lil_matrix 
from scipy.optimize import least_squares

import opt_utils.opt_utils as opt_utils

# ------------------------
##   DATA STRUCTURES   ##
# ------------------------

# ------------------------
## FUNCTION DEFINITION ##
# ------------------------

# ------------------------
###   BASE CLASSES    ###
# ------------------------

# ------------------------
### CLASS DEFINITION  ###
# ------------------------

class ColorCorrect():
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
        ap.add_argument("-z", "--z_inconsistency_threshold", help="threshold for max z inconsistency value", type=float,
                        default=0.05)
        ap.add_argument("-zfilt", "--use_z_filtering", help="use z-filtering technique", required=False,
                        action='store_true', default=False)
        ap.add_argument("-vzb", "--view_z_buffering", help="visualize z-buffering", action='store_true', default=False)
        ap.add_argument("-zbuf", "--use_z_buffering", help="use z-buffering", action='store_true', default=False)
        ap.add_argument("-ext", "--image_extension", help="extension of the image files, e.g., jpg or png",
                        default='jpg')
        ap.add_argument("-vri", "--view_range_image", help="visualize sparse and dense range images",
                        action='store_true', default=False)
        ap.add_argument("-vpv", "--view_projected_vertices", help="visualize projections of vertices onto images",
                        action='store_true', default=False)
        ap.add_argument("-sv", "--skip_vertices", help="skip vertices. Useful for fast testing", type=int,
                        default=1)
        ap.add_argument("-si", "--skip_images", help="skip images. Useful for fast testing", type=int,
                        default=1)
        ap.add_argument("-vo", "--view_optimization", help="To show images being corrected during optimization or not", action='store_true', default=False)

        self.p = vars(ap.parse_args())
        print(self.p)
        #exit(0)


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

        # ---------------------------------------
        # --- Project instantaneous point clouds to each of the camera images
        # ---------------------------------------
        print('Projecting local point clouds to images:')

        depth_image_folder = self.p['path_to_images'] + 'depthimage_speedup/'
        self.createFolder(depth_image_folder, overwrite=False)
        for camera in tqdm(self.cameras):
            depth_image_filename = depth_image_folder + os.path.basename(camera.rgb.filename)[:-4] + '.png'

            if self.fileExists(depth_image_filename):
                tmp = cv2.imread(depth_image_filename, flags=cv2.IMREAD_ANYDEPTH)
                print('Loaded depth image ' + depth_image_filename)
                camera.rgb.range_dense = tmp.astype(np.float) / 1000.0
                camera.rgb.range_sparse = camera.rgb.range_dense
                camera.rgb.overlay = deepcopy(camera.rgb.image)
            else:
                camera.depth.matrix[0, :] = [1, 0, 0, 0]
                camera.depth.matrix[1, :] = [0, 0, 1, 0]
                camera.depth.matrix[2, :] = [0, -1, 0, 0]
                camera.depth.matrix[3, :] = [0, 0, 0, 1]
                # print('camera.depth.vertices')
                # print(camera.depth.vertices.shape)

                camera.rgb.range_sparse, camera.rgb.mask, camera.rgb.range_dense, camera.rgb.vert_projected, camera.rgb.valid_vert, camera.rgb.dist_vert = self.getRangeImageFromPointCloud(
                    camera, rescale=0.5)

                # save image to file
                tmp = deepcopy(camera.rgb.range_dense)
                tmp = tmp * 1000.0  # to millimeters
                tmp = tmp.astype(np.uint16)
                cv2.imwrite(depth_image_filename, tmp)
                print('Saved depth image ' + depth_image_filename)

                _, n_pts = camera.depth.vertices.shape
                camera.rgb.overlay = deepcopy(camera.rgb.image)
                for i in range(0, n_pts):
                    if camera.rgb.valid_vert[i] == True:
                        x0 = camera.rgb.vert_projected[0, i]
                        y0 = camera.rgb.vert_projected[1, i]
                        cv2.line(camera.rgb.overlay, (x0, y0), (x0, y0), color=(255, 200, 0), thickness=2)

            if self.p['view_range_image'] is True:
                cv2.namedWindow('Original', cv2.WINDOW_NORMAL)
                cv2.imshow('Original', camera.rgb.image)

                cv2.namedWindow('Overlay', cv2.WINDOW_NORMAL)
                cv2.imshow('Overlay', camera.rgb.overlay)

                cv2.namedWindow('RangeSparse', cv2.WINDOW_NORMAL)
                self.normImShow('RangeSparse', camera.rgb.range_sparse, 5)

                cv2.namedWindow('RangeDense', cv2.WINDOW_NORMAL)
                self.normImShow('RangeDense', camera.rgb.range_dense, 5)

                self.keyPressManager()

        # ---------------------------------------
        # --- Compute the list of points that will be used by the cost function
        # ---------------------------------------
        cam_pairs = []
        for cam_a, cam_b in combinations(self.cameras, 2):
            print(cam_a.name + ' with ' + cam_b.name)

            # get a list of 3D points by concatenating the 3D measurements of cam_a and cam_b
            pts3D_in_map_a = np.dot(cam_a.depth.matrix, cam_a.depth.vertices[:, 0::self.p['skip_vertices']])
            pts3D_in_map_b = np.dot(cam_b.depth.matrix, cam_b.depth.vertices[:, 0::self.p['skip_vertices']])
            pts3D_in_map = np.concatenate([pts3D_in_map_a, pts3D_in_map_b], axis=1)

            # project 3D points to cam_a and to cam_b
            pts2D_a, pts_valid_a, pts_range_a = cam_a.rgb.projectToPixel3(pts3D_in_map)
            pts2D_a = np.where(pts_valid_a, pts2D_a, 0)
            range_meas_a = cam_a.rgb.range_dense[pts2D_a[1,:], pts2D_a[0,:]]
            z_valid_a = abs(pts_range_a - range_meas_a) < self.p['z_inconsistency_threshold']

            pts2D_b, pts_valid_b, pts_range_b = cam_b.rgb.projectToPixel3(pts3D_in_map)
            pts2D_b = np.where(pts_valid_b, pts2D_b, 0)
            range_meas_b = cam_b.rgb.range_dense[pts2D_b[1,:], pts2D_b[0,:]]
            z_valid_b = abs(pts_range_b - range_meas_b) < self.p['z_inconsistency_threshold']

            #Compute masks for the valid projections
            mask = np.logical_and(pts_valid_a, pts_valid_b)
            z_mask = np.logical_and(z_valid_a, z_valid_b)
            final_mask = np.logical_and(mask, z_mask)

            #Create a dictionary to describe this camera pair (to be used by the objective function)
            cam_pair = {}
            cam_pair['cam_a'] = cam_a.name
            cam_pair['cam_b'] = cam_b.name
            cam_pair['idx_a'] = [x.name for x in self.cameras].index(cam_a.name)
            cam_pair['idx_b'] = [x.name for x in self.cameras].index(cam_b.name)
            cam_pair['pts2D_a'] = pts2D_a[:, final_mask]
            cam_pair['pts2D_b'] = pts2D_b[:, final_mask]
            cam_pairs.append(cam_pair)

            if self.p['view_projected_vertices']:
                print("pts2d_a has " + str(np.count_nonzero(pts_valid_a)) + ' valid projections')
                print("pts2d_b has " + str(np.count_nonzero(pts_valid_b)) + ' valid projections')
                print("there are " + str(np.count_nonzero(mask)) + ' valid projection pairs')
                cam_a_image = deepcopy(cam_a.rgb.image)
                cam_b_image = deepcopy(cam_b.rgb.image)
                for i, val in enumerate(mask):
                    if pts_valid_a[i] == True:
                        x0 = pts2D_a[0, i]
                        y0 = pts2D_a[1, i]
                        cv2.line(cam_a_image, (x0, y0), (x0, y0), color=(80, 80, 80), thickness=2)

                    if pts_valid_b[i] == True:
                        x0 = pts2D_b[0, i]
                        y0 = pts2D_b[1, i]
                        cv2.line(cam_b_image, (x0, y0), (x0, y0), color=(80, 80, 80), thickness=2)

                    if val == True:
                        x0 = pts2D_a[0, i]
                        y0 = pts2D_a[1, i]
                        cv2.line(cam_a_image, (x0, y0), (x0, y0), color=(0, 0, 210), thickness=2)

                        x0 = pts2D_b[0, i]
                        y0 = pts2D_b[1, i]
                        cv2.line(cam_b_image, (x0, y0), (x0, y0), color=(0, 0, 210), thickness=2)

                    if z_mask[i] == True:
                        x0 = pts2D_a[0, i]
                        y0 = pts2D_a[1, i]
                        cv2.line(cam_a_image, (x0, y0), (x0, y0), color=(0, 210, 0), thickness=2)

                        x0 = pts2D_b[0, i]
                        y0 = pts2D_b[1, i]
                        cv2.line(cam_b_image, (x0, y0), (x0, y0), color=(0, 210, 0), thickness=2)

                cv2.namedWindow('cam_a', cv2.WINDOW_NORMAL)
                cv2.imshow('cam_a', cam_a_image)
                cv2.namedWindow('cam_b', cv2.WINDOW_NORMAL)
                cv2.imshow('cam_b', cam_b_image)

                if self.p['view_optimization']:
                    self.keyPressManager()

        print(cam_pairs)

        #Compute float and YCrCB images for each camera
        for camera in self.cameras:
            camera.rgb.fimage = np.float32(camera.rgb.image) / 255.0
            camera.rgb.ycrcb_image = cv2.cvtColor(camera.rgb.fimage, cv2.COLOR_BGR2YCrCb)
            camera.rgb.lab_image = cv2.cvtColor(camera.rgb.fimage, cv2.COLOR_BGR2LAB)
            print(camera.rgb.lab_image.dtype)

            # Print the minimum and maximum of lightness.
            #l_channel,a_channel,b_channel = cv2.split(camera.rgb.lab_image)
            #print np.min(l_channel) # 0
            #print np.max(l_channel) # 255

            ## Print the minimum and maximum of a.
            #print np.min(a_channel) # 42
            #print np.max(a_channel) # 226

            ## Print the minimum and maximum of b.
            #print np.min(b_channel) # 20
            #print np.max(b_channel) # 223

            #exit(0)
    
        # ---------------------------------------
        # --- PREPARE OPTIMIZATION
        # ---------------------------------------


        def adjustGamma(image, gamma=1.0):
            # build a lookup table mapping the pixel values [0, 255] to
            # their adjusted gamma values
            invGamma = 1.0 / gamma
            table = np.array([((i / 255.0) ** invGamma) for i in np.arange(0, 256)]).astype("float32")

            # apply gamma correction using the lookup table
            return cv2.LUT(image, table)

        def colorCorrectYCrCb(image, y_bias=0.0, y_scale=1.0, u_bias=0.0, v_bias=0.0):

            imyuv = image.copy()
            imyuv[:,:,(1,2)] -= 0.5 #bias
            imyuv[:,:,1] += u_bias * imyuv[:,:,0]
            imyuv[:,:,2] += v_bias * imyuv[:,:,0]
            imyuv[:,:,:] *= y_scale
            imyuv[:,:,0] += y_bias

            imyuv[:,:,(1,2)] += 0.5 # bias

            rgb = cv2.cvtColor(imyuv, cv2.COLOR_YCrCb2BGR)

            return rgb, imyuv
            #return np.uint8(rgb * 255.0)

        def colorCorrectLAB(image, l_bias=0.0, l_scale=1.0, a_bias=0.0, b_bias=0.0):

            imlab = image.copy()
            #imlab[:,:,(1,2)] -= 0.5 #bias
            imlab[:,:,1] += a_bias
            imlab[:,:,2] += b_bias
            #imlab[:,:,:] *= y_scale
            imlab[:,:,0] += l_bias

            #imyuv[:,:,(1,2)] += 0.5 # bias

            rgb = cv2.cvtColor(imlab, cv2.COLOR_LAB2BGR)

            return rgb, imlab
            #return np.uint8(rgb * 255.0)

        #---------------------------------------
        #--- Optimization function
        #---------------------------------------
        def objectiveFunction(x, cam_pairs, cameras, show=False):

            t = time.time()
            #apply gamma correction for all camera images given the gamma value (x)
            for i, camera in enumerate(cameras):
                idx = i * 256

                table = np.array(x[idx:idx+256])
                print(table)

                camera.rgb.gc_image = cv2.LUT(camera.rgb.image, table).astype(np.uint8)
            
            #Compute the error with the new corrected images
            e = []
            for i, cam_pair in enumerate(cam_pairs):
                pts2D_a = cam_pair['pts2D_a']
                pts2D_b = cam_pair['pts2D_b']

                cam_a = cameras[cam_pair['idx_a']]
                cam_b = cameras[cam_pair['idx_b']]

                pixs_a = cam_a.rgb.gc_image[pts2D_a[1,:], pts2D_a[0,:]]
                pixs_b = cam_b.rgb.gc_image[pts2D_b[1,:], pts2D_b[0,:]]

                diff = pixs_a - pixs_b
                dist = np.linalg.norm(diff)
                e.append(np.linalg.norm(dist))

            
            t = time.time();
            if show == True:
                #print("x =\n" + str(['{:.3f}'.format(i) for i in x]))
                print("e =\n" + str(['{:.3f}'.format(i) for i in e]))
                print("avg error =\n" + str('{:.3f}'.format(np.average(e))))

                #for i, camera in enumerate(cameras):
                    #cv2.namedWindow(camera.name + '_original', cv2.WINDOW_NORMAL)
                    #cv2.imshow(camera.name + '_original', camera.rgb.image)
                for i, camera in enumerate(cameras):
                    cv2.namedWindow(camera.name + '_corrected', cv2.WINDOW_NORMAL)
                    cv2.imshow(camera.name + '_corrected', camera.rgb.gc_image)
                #cv2.waitKey(0)

            #print('visualization in ' + str(time.time() - t) + ' secs')

            return e



        #---------------------------------------
        #--- Create first guess
        #---------------------------------------
        x0 = list(range(0,256)) * len(self.cameras)
        print('x0 = ' + str(len(x0)))

        #---------------------------------------
        #--- Jacobian sparse matrix
        #---------------------------------------
        n = len(x0) #Number of columns n of the sparse matrix (i.e. number of input paramters)
        print("n = " + str(n))
        m = len(cam_pairs) #Number of output residuals m, i.e. the size of the output vector
        print("m = " + str(m))

        SM = lil_matrix((m, n), dtype=int)

        for i, cam_pair in enumerate(cam_pairs): #each camera pair is a value of the error
            idx_a = cam_pair['idx_a']*4
            SM[i, idx_a:idx_a+4] = 1

            idx_b = cam_pair['idx_b']*4
            SM[i, idx_b:idx_b+4] = 1

        print(SM.shape)
        print("SM=\n" + str(SM.toarray()))

        #---------------------------------------
        #--- Set the bounds for the parameters
        #---------------------------------------
        #bounds : 2-tuple of array_like, optional
        #Lower and upper bounds on independent variables. Defaults to no bounds. Each array must match the size of x0 or be a scalar, in the latter case a bound will be the same for all variables. Use np.inf with an appropriate sign to disable bounds on all or some variables.

        Bmin = [0] * len(x0) #max gamma value for all cameras
        Bmax = [255.0] * len(x0) #min gamma value for all cameras
        print('Bmin ' + str(len(Bmin)))
        print('Bmax ' + str(len(Bmax)))

        bounds=(Bmin, Bmax)
        #print(bounds)

        #---------------------------------------
        #--- Test call of objective function
        #---------------------------------------
        #call objective function with initial guess (just for testing)
        e0 = objectiveFunction(x0, cam_pairs, self.cameras, show=self.p['view_optimization'])
        print("initial_residuals = " + str(e0))

        if self.p['view_optimization']:
            self.keyPressManager()


        #---------------------------------------
        #--- Optimization (minimization)
        #---------------------------------------
        print("\n\nStarting minimization")

        # --- Without sparsity matrix
        #res = least_squares(objectiveFunction3, x0, verbose=2, ftol=1e-6, xtol=1e-6, method='trf', bounds=bounds, args=(marker_detections, views, pinhole_camera_model))

        # --- With sparsity matrix
        res = least_squares(objectiveFunction, x0, verbose=2, jac_sparsity=SM, gtol=1e-6, ftol=1e-6, xtol=1e-6, method='trf', bounds=bounds, args=(cam_pairs, self.cameras, self.p['view_optimization']), diff_step = 10, x_scale='jac', tr_solver='lsmr')
           
        print("OPTIMIZATON FINISHED")

        #---------------------------------------
        #--- Present the results
        #---------------------------------------
        print("Computing final solution")
        ef = objectiveFunction(res.x, cam_pairs, self.cameras, show=True)
        self.keyPressManager()

        #print("Initial x0 =\n" + str(['{:.2f}'.format(x) for x in x0]))
        #print("Final res.x=\n" + str(['{:.2f}'.format(x) for x in res.x]))
        print("Initial average error = " + str(np.average(e0)) + " Final average error = " + str(np.average(ef)))

        # -------------------------------------#
        ### Prepare folder for results
        # -------------------------------------#
        self.p['path_results'] = self.p['path_to_images'] + 'color_correction/'
        self.createFolder(self.p['path_results'], overwrite=True)

        print('Loaded parameters: ' + str(self.p))
        print('Saving a json with parameters.')
        f = open(self.p['path_results'] + 'input_parameters.json', 'w')
        print >> f, json.dumps(self.p, indent=2, sort_keys=True)
        f.close()

        #---------------------------------------
        #--- Save the gamma corrected images
        #---------------------------------------
        for i, camera in enumerate(self.cameras):
            #gc_image = (colorCorrectYCrCb(camera.rgb.image, gamma = res.x[i]*x_scale) * 255).astype(np.uint8)
            idx = i * 4
            #gc_image,_ = colorCorrectYCrCb(camera.rgb.ycrcb_image, y_bias=res.x[idx], y_scale=res.x[idx+1], u_bias=res.x[idx+2], v_bias=res.x[idx+3])
            gc_image,_ = colorCorrectLAB(camera.rgb.lab_image, l_bias=res.x[idx], l_scale=res.x[idx+1], a_bias=res.x[idx+2], b_bias=res.x[idx+3])
            gc_image = (gc_image * 255)
            gc_image[gc_image > 255] = 255
            gc_image[gc_image < 0] = 0
            gc_image = gc_image.astype(np.uint8)
            cv2.imwrite(self.p['path_results'] + os.path.basename(camera.rgb.filename), gc_image)


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

    def getRangeImageFromPointCloud(self, camera, rescale=None):

        n_pts = camera.depth.vertices.shape[1]

        pts3D_in_map = np.dot(camera.depth.matrix, camera.depth.vertices)
        pts2D, pts_valid, pts_range = camera.rgb.projectToPixel3(pts3D_in_map)

        h, w, _ = camera.rgb.image.shape
        range_sparse = np.zeros((h, w), dtype=np.float32)
        mask = 255 * np.ones((range_sparse.shape[0], range_sparse.shape[1]), dtype=np.uint8)

        for i in range(0, n_pts):
            if pts_valid[i] == True:
                x0 = pts2D[0, i]
                y0 = pts2D[1, i]
                mask[y0, x0] = 0
                range_sparse[y0, x0] = pts_range[i]

        if not rescale is None:  # downsample the image before inpaiting
            rs_range_sparse = cv2.resize(range_sparse, (0, 0), fx=rescale, fy=rescale, interpolation=cv2.INTER_NEAREST)
            rs_mask = cv2.resize(mask, (0, 0), fx=rescale, fy=rescale, interpolation=cv2.INTER_NEAREST)
        else:
            rs_range_sparse = range_sparse
            rs_mask = mask

        range_dense = np.zeros((rs_range_sparse.shape[0], rs_range_sparse.shape[1]), dtype=np.float32)

        # Computing the dense depth map
        print('Computing inpaint ...')
        range_dense = cv2.inpaint(rs_range_sparse, rs_mask, 7, cv2.INPAINT_NS)
        print('Inpaint done')

        if not rescale is None:  # downsample the image before inpaiting
            range_dense = cv2.resize(range_dense, (0, 0), fx=1 / rescale, fy=1 / rescale, interpolation=cv2.INTER_CUBIC)

        return range_sparse, mask, range_dense, pts2D, pts_valid, pts_range

    def normImShow(self, window_name, image, max_val=None):
        if max_val is None:
            normalized = np.divide(image, np.amax(image))
        else:
            normalized = np.divide(image, max_val)
            normalized[normalized > 1.0] = 1.0  # Saturate values over 1.0

        cv2.imshow(window_name, normalized)


