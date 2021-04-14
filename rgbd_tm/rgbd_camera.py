"""
Classes for defining an rgbd camera
"""

#------------------------
##    IMPORT MODULES   ##
#------------------------
import numpy as np
import time
import cv2
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from scipy.linalg import norm

#------------------------
##   DATA STRUCTURES   ##
#------------------------

#------------------------
## FUNCTION DEFINITION ##
#------------------------

def tic():
    #matlab like tic and toc functions
    global startTime_for_tictoc
    startTime_for_tictoc = time.time()

def toc():
    #matlab like tic and toc functions
    if 'startTime_for_tictoc' in globals():
        print("Elapsed time is " + str(time.time() - startTime_for_tictoc) + " seconds.")
    else:
        print("Toc: start time not set")

def tocs():
    #matlab like tic and toc functions
    if 'startTime_for_tictoc' in globals():
        return str(time.time() - startTime_for_tictoc)
    else:
        print("Toc: start time not set")
        return None

#------------------------
###   BASE CLASSES    ###
#------------------------

#------------------------
### CLASS DEFINITION  ###
#------------------------

class TransformationT():
    """ Base class for geometric transformation
    """

    def __init__(self, matrix, frame_id, parent_frame_id, stamp=None):
        self.matrix = matrix
        self.frame_id = frame_id
        self.parent_frame_id = parent_frame_id
        self.stamp = stamp
    def __str__(self):
        return 'Transform from ' + str(self.parent_frame_id) + ' to ' + str(self.frame_id) + ' at time ' + str(self.stamp) + '\n' + str(self.matrix)

class ImageT():
    """ Base class for Image
    """

    def __init__(self, image):
        """Constructor for image class
            args:
                image (numpy array): An image in BGR or RGB format
        """
        self.image = image

        if image is not None:
            self.h, self.w, _ = self.image.shape
        else:
            self.h = float('nan')
            self.w = float('nan')


    def show(self, name='Default', position = (0, 0), size = (250, 250), wait_for_key=False):
        """Shows the current image with an opencv window
            args:
                name (string): 
                position (tuple(int,int): x,y position of the window's top left corner
                size (tuple(int,int)): width and height of the window
                wait_for_key (bool): show and wait for key before continuing
        """
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)

            
        cv2.imshow(name, self.image)
        #cv2.resizeWindow(name, size[0], size[1])
        #cv2.moveWindow(name, position[0], position[1])
        if wait_for_key is True:
            cv2.waitKey(0)

class CameraT(ImageT, TransformationT, PinholeCameraModel):
    """ Base class for Camera
    """

    def __init__(self, image, matrix, frame_id, parent_frame_id, stamp=None, camera_info=None, filename=None, vertices=None):
        """Constructor
            Args: 
                image (numpy array): An image in BGR or RGB format
                matrix (numpy array): 4x4 homogeneous transformation matrix
                frame_id (string): camera's frame_id
                parent_frame_id (string): name of the parent frame id, such that the transformation given in matrix relates the parent_frame_id with the camera frame_id 
                stamp (double): timesamp 
                camera_info (?): camera info structure 
        """

        if image is not None:
            ImageT.__init__(self, image)

        if vertices is not None:
            self.vertices = vertices

        TransformationT.__init__(self, matrix, frame_id, parent_frame_id, stamp=None)
        self.filename = filename
        self.camera_info = camera_info
        if self.camera_info is not None: #initialize pinhole camera model if camera_info exists
            PinholeCameraModel.__init__(self)
            PinholeCameraModel.fromCameraInfo(self, self.camera_info)
            self.has_intrinsics = True
            h, w = self.image.shape[:2]
            K = np.reshape(self.camera_info.K, (3,3))
            #print(self.camera_info.D)
            D = np.reshape(self.camera_info.D, (1,5))
        else:
            self.has_intrinsics = False


    def transformToWorld(self, pts):
        return np.dot(self.matrix, pts)

    def transformFromWorld(self, pts):
        return np.dot(np.linalg.inv(self.matrix), pts)

    def projectToPixel3(self, pts_world):
        """Matricial opencv based version. This is very fast"""

        tic()
        #Project the pts_world to the camera reference system
        #pts = np.dot(np.linalg.inv(self.matrix), pts_world)
        pts = self.transformFromWorld(pts_world)
        _, n_pts = pts.shape
        
        #Project to image pixels
        pixs = np.zeros((2,n_pts),dtype=np.int)
      
        #Get the intrinsics from the camera info structure
        #Distortion should be as follows: (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]])
        #See https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#
        k1, k2, p1, p2, k3 = self.camera_info.D
        fx, _, cx, _, fy, cy, _, _, _ = self.camera_info.K

        x = pts[0,:]
        y = pts[1,:]
        z = pts[2,:]
        #print(pts.shape)

        dists = norm(pts[0:3, :], axis = 0)

        #print(dist.shape)
        #print(dist)
        #exit(0)

        xl = np.divide(x,z)
        yl = np.divide(y,z)

        r2 = xl**2 + yl**2 #r square

        xll = xl * (1 + k1 * r2 + k2 * r2**2 + k3 *r2**3) + 2 * p1 * xl * yl + p2 * (r2 + 2 * xl**2)
        yll = yl * (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3) + p1 * (r2 + 2 * yl**2) + 2 * p2 * xl * yl

        pixs[0,:] = fx * xll + cx
        pixs[1,:] = fy * yll + cy

        #Compute mask of valid projections
        valid_z = z > 0

        W = self.camera_info.width
        valid_xpix = np.logical_and(pixs[0,:] >= 0 , pixs[0,:] < W)

        H = self.camera_info.height
        valid_ypix = np.logical_and(pixs[1,:] >= 0 , pixs[1,:] < H)

        valid_pixs = np.logical_and(valid_z, np.logical_and(valid_xpix, valid_ypix))

        #print("projectToPixel3 took " + tocs() + " secs")
        return pixs, valid_pixs, dists

    # def projectToPixel2(self, pts_world):
    #     """Deprecated opencv, for loop based projection. Use projectToPixel2 instead"""
    #
    #     tic()
    #     #Project the pts_world to the camera reference system
    #     pts = np.dot(np.linalg.inv(self.matrix), pts_world)
    #     _, n_pts = pts.shape
    #
    #     #Project to image pixels
    #     self.pixs = np.zeros((2,n_pts),dtype=np.int)
    #     self.pixs_mask = np.zeros((1,n_pts),dtype=np.bool)
    #
    #     #Get the intrinsics from the camera info structure
    #     #Distortion should be as follows: (k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6]])
    #     #See https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#
    #     k1, k2, p1, p2, k3 = self.camera_info.D
    #     fx, _, cx, _, fy, cy, _, _, _ = self.camera_info.K
    #
    #     for i in range(0, n_pts):
    #
    #         x = pts[0,i]
    #         y = pts[1,i]
    #         z = pts[2,i]
    #
    #         xl = x/z
    #         yl = y/z
    #         r2 = xl**2 + yl**2 #r square
    #
    #         xll = xl * (1 + k1 * r2 + k2 * r2**2 + k3 *r2**3) + 2 * p1 * xl * yl + p2 * (r2 + 2 * xl**2)
    #         yll = yl * (1 + k1 * r2 + k2 * r2**2 + k3 * r2**3) + p1 * (r2 + 2 * yl**2) + 2 * p2 * xl * yl
    #
    #         xpix = fx * xll + cx
    #         ypix = fy * yll + cy
    #
    #         self.pixs[0,i] = xpix
    #         self.pixs[1,i] = ypix
    #
    #     #print("projectToPixel2 took " + tocs() + " secs")

    # def projectToPixel(self, points):
    #     """Deprecated openconstructor, for loop based projection. Use projectToPixel3 instead"""
    #
    #     tic()
    #     points_camera = self.transformToCamera(points)
    #     _, n_pts = points_camera.shape
    #
    #     #Project to image pixels
    #     self.pixs = np.zeros((2,n_pts),dtype=np.int)
    #     self.pixs_mask = np.zeros((1,n_pts),dtype=np.bool)
    #
    #
    #
    #     #= np.array(1)
    #     #>>> np.matlib.repmat(a0, 2, 3)
    #     #array([[1, 1, 1],
    #                #[1, 1, 1]])
    #
    #     for i in range(0,n_pts):
    #         pt = tuple(points_camera[:3, i])
    #
    #         #print(points_camera[:, i])
    #         pix = self.project3dToPixel(pt)
    #         #print('pix = ' + str(pix))
    #
    #         K = np.reshape(self.camera_info.K, (3,3))
    #         z = pt[2]
    #
    #         hpix = np.dot(K, pt)
    #         #print('hpix = ' + str(hpix))
    #         xpix = hpix[0] / (hpix[2] * z)
    #         ypix = hpix[1] / (hpix[2] * z)
    #         #print('xpix = ' + str(xpix))
    #         #print('ypix = ' + str(ypix))
    #
    #         self.pixs[:,i] = pix
    #         #self.pixs[0,i] = xpix
    #         #self.pixs[1,i] = ypix
    #
    #         fx = K[0, 0]
    #         fy = K[1, 1]
    #         cx = K[0, 2]
    #         cy = K[1, 2]
    #
    #         xpix = pt[0] / pt[2]
    #         ypix = pt[1] / pt[2]
    #         xpix = xpix * fx + cx
    #         ypix = ypix * fy + cy
    #
    #         #print('xpix = ' + str(xpix))
    #         #print('ypix = ' + str(ypix))
    #
    #
    #         if pix[0] < 0 or pix[0] >= self.h or pix[1] < 0 or pix[1] >= self.w:
    #             self.pixs_mask[0,i] = True
    #         else:
    #             self.pixs_mask[0,i] = False
    #
    #     #xypix = self.points2imageFromT(self.matrix, PinholeCameraModel.K, points, PinholeCameraModel.D)
    #
    #     print("projectToPixel took " + tocs() + " secs")

    def transformToCamera(self, points):
        """Transforms a set of 3D points to the camera reference frame"""
        return np.dot(np.linalg.inv(self.matrix), points)


    def points2imageFromT(self, T, K, P, dist):

        # Calculation of the point in the image in relation to the chess reference
        xypix = []

        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]

        k1 = dist[0, 0]
        k2 = dist[0, 1]
        p1 = dist[0, 2]
        p2 = dist[0, 3]
        k3 = dist[0, 4]

        P = P[:, 0:3]

        for p in P:

            rot = np.matrix(T[0: 3, 0: 3])
            xyz = rot.dot(p) + T[0: 3, 3]

            xl = xyz[0, 0] / xyz[0, 2]
            yl = xyz[0, 1] / xyz[0, 2]

            r_square = xl**2 + yl**2

            xll = xl * (1 + k1 * r_square + k2 * r_square**2 + k3 *
            r_square**3) + 2 * p1 * xl * yl + p2 * (r_square + 2 * xl**2)
            yll = yl * (1 + k1 * r_square + k2 * r_square**2 + k3 *
            r_square**3) + p1 * (r_square + 2 * yl**2) + 2 * p2 * xl * yl

            u = fx * xll + cx
            v = fy * yll + cy

            xypix.append([u, v])

        return np.array(xypix)


    def __str__(self):
        if self.camera_info is not None:
            ss = 'Intrinsic data\nwidth = ' + str(self.camera_info.width)
            ss += '\nheight = ' + str(self.camera_info.height) 
            ss += '\nK = ' + str(self.camera_info.K) 
            ss += '\nD = ' + str(self.camera_info.D) 
            ss += '\nR = ' + str(self.camera_info.R) 
            ss += '\nP = ' + str(self.camera_info.P) 
        else:
            ss = 'No intrinsic data'

        return ss + '\n' + TransformationT.__str__(self)

class RGBDCameraT():
    """ Class for RGBD Camera
    """
    def __init__(self, name, parent_frame_id,
                 rgb_data, rgb_matrix, rgb_frame_id, rgb_stamp, rgb_camera_info, rgb_filename,
                 depth_data, depth_matrix, depth_frame_id, depth_stamp, depth_camera_info, depth_filename, point_cloud=None):
        self.name = name

        self.rgb = CameraT(rgb_data, rgb_matrix, rgb_frame_id, parent_frame_id, rgb_stamp, rgb_camera_info, rgb_filename)
        self.depth = CameraT(None, depth_matrix, depth_frame_id, parent_frame_id, depth_stamp, depth_camera_info, depth_filename, point_cloud)

    def __str__(self):
        return 'Camera ' + self.name + '\nRGB:\n' + self.rgb.__str__() + '\n' + 'Depth:\n' + self.depth.__str__()


