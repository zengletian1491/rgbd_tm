"""
This is a long, multiline description
"""

import argparse  # to read command line arguments
import cPickle as pickle
#to list images in a folder
import glob
import json
import shutil
import yaml
from collections import namedtuple
from copy import deepcopy
from math import sqrt, exp
from matplotlib import cm
from random import random
from shapely.geometry import Polygon, mapping # for the z buffering
import openmesh as om
from sensor_msgs.msg import CameraInfo
from tqdm import tqdm
from model_viewer import *
from rgbd_camera import *
from PIL import Image
from plyfile import PlyData, PlyElement

# ------------------------
##   DATA STRUCTURES   ##
# ------------------------
PropagationT = namedtuple("PropagationT", "fh parent_fh")

# ------------------------
## FUNCTION DEFINITION ##
# ------------------------

# ------------------------
###   BASE CLASSES    ###
# ------------------------

# ------------------------
### CLASS DEFINITION  ###
# ------------------------

class TextureMapper(FrontEnd):
    """ Class to texture map
    """

    def __init__(self):
        """Implements a texture mapper
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
        ap.add_argument("-m", "--mesh_filename", help="full filename to input obj file, i.e. the 3D model",
                        required=True)
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
        ap.add_argument("-vsa", "--view_selection_animation", help="visualize camera selection animation",
                        action='store_true', default=False)
        ap.add_argument("-sm", "--selection_method", help="method for selecting the cameras for each triangle",
                        type=int, default=0)
        ap.add_argument("-bfs", "--border_face_smoothing", help="Use border face smoothing", action='store_true', default=False)
        ap.add_argument("-ucci", "--use_color_corrected_images", help="Use previously color corrected images", action='store_true', default=False)
        ap.add_argument("-si", "--skip_images", help="skip images. Useful for fast testing", type=int, default=1)


        self.p = vars(ap.parse_args())

        # -------------------------------------#
        ### Selection methods
        # -------------------------------------#
        self.p['methods'] = {
            0: 'random',
            1: 'first_camera',
            2: 'last_camera',
            3: 'largest_projected_triangle_area',
            4: 'random_seed_propagation'
        }

        self.p['method_name'] = self.p['methods'].get(self.p['selection_method'])
        if self.p['method_name'] is None:
            raise KeyError('Method number ' + str(
                self.p['selection_method']) + ' does not exits. Select one of these methods: ' + str(self.methods))

        # -------------------------------------#
        ### Prepare folder for results
        # -------------------------------------#
        self.p['path_results'] = self.p['path_to_images'] + self.p['method_name'] + '/'
        self.createFolder(self.p['path_results'], overwrite=True)

        self.p['output_mesh_filename'] = self.p['path_results'] + 'mesh.obj'
        self.p['output_mtl_filename'] = self.p['output_mesh_filename'][:-4] + '.mtl'

        self.p['zbuf_speedup_file'] = self.p['path_to_images'] + 'zbuf_speedup/' + os.path.basename(
            self.p['mesh_filename'])[:-4] + '.pkl'
        self.createFolder(self.p['zbuf_speedup_file'], overwrite=False)

        print('Loaded parameters: ' + str(self.p))
        print('Saving a json with parameters.')
        f = open(self.p['path_results'] + 'input_parameters.json', 'w')
        print >> f, json.dumps(self.p, indent=2, sort_keys=True)
        f.close()

        # -------------------------------------#
        ### OpenGL Visualizer
        # -------------------------------------#
        FrontEnd.__init__(self, initialize=False)

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
        self.readCameras(ci, None, image_skip=self.p['skip_images'], use_color_corrected_images=self.p['use_color_corrected_images'])
        for camera in self.cameras:
            # print camera
            # camera.rgb.show(wait_for_key = True)
            pass

        print('Loaded ' + str(len(self.cameras)) + ' cameras from folder ' + self.p['path_to_images'])
        self.p['num_cameras'] = len(self.cameras)

        # ---------------------------------------
        # --- Setup Colormap
        # ---------------------------------------
        # cmap_cameras = cm.Set3(np.linspace(0, 1, len(self.cameras)))
        cmap_cameras = cm.YlOrRd(np.linspace(0, 1, len(self.cameras)))

        # ---------------------------------------
        # --- Load Mesh
        # ---------------------------------------
        self.mesh = om.read_trimesh(filename=self.p['mesh_filename'], \
                                    binary=False, \
                                    msb=False, \
                                    lsb=False, \
                                    swap=False, \
                                    vertex_normal=True, \
                                    vertex_color=False, \
                                    vertex_tex_coord=False, \
                                    halfedge_tex_coord=False, \
                                    edge_color=False, \
                                    face_normal=False, \
                                    face_color=False, \
                                    color_alpha=True, \
                                    color_float=True)

        self.mesh.request_face_texture_index()  # to make sure we load the mtl instructions from the obj

        print('Loaded mesh from file ' + self.p['mesh_filename'])

        self.printMeshInfo(self.mesh)


        # ---------------------------------------
        # --- Project mesh's triangles to the cameras
        # ---------------------------------------
        # create a np array with all the vertices, so they can easily be mapped onto each camera's reference frame
        self.pts_map = np.array(self.mesh.points(), dtype=np.float).transpose()
        _, self.n_pts = self.pts_map.shape

        # homogeneize
        self.pts_map = np.vstack([self.pts_map, np.ones((self.n_pts,), dtype=np.float)])

        print('Projecting vertices to cameras:')
        for camera in tqdm(self.cameras):
            camera.rgb.pixs, camera.rgb.valid_pixs, camera.rgb.dists = camera.rgb.projectToPixel3(self.pts_map)

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
                # print('Loaded depth image ' + depth_image_filename)
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
                    camera, rescale=1)

                # save image to file
                tmp = deepcopy(camera.rgb.range_dense)
                tmp = tmp * 1000.0  # to millimeters
                tmp = tmp.astype(np.uint16)
                cv2.imwrite(depth_image_filename, tmp)
                print('Saved depth image ' + depth_image_filename)


                                # saves the point clouds aligned with the obj file
                # _, n_pts = camera.depth.vertices.shape
                # point_cloud = om.TriMesh()
                # for i in range(0, n_pts):
                # pt = np.transpose(vertices_map[:-1,i])
                # vh0 = point_cloud.add_vertex(pt)

                # om.write_mesh(filename = 'test.ply', mesh = point_cloud, binary = False, msb = False, lsb = False, swap = False, vertex_normal = False,vertex_color = False,vertex_tex_coord = False,halfedge_tex_coord = False,edge_color = False,face_normal = False,face_color = False,color_alpha = False,color_float = False )

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
        # --- Save dense point cloud in pcd format
        # ---------------------------------------
        #print('Saving dense point clouds (pcl format):')

        #dense_point_cloud_folder = self.p['path_to_images'] + 'dense_point_cloud_speedup/'
        #self.createFolder(dense_point_cloud_folder, overwrite=False)
        #for camera in tqdm(self.cameras):
            #depth_image_filename = depth_image_folder + os.path.basename(camera.rgb.filename)[:-4] + '.png'
            #point_cloud_ply_filename = '/tmp/point_cloud.ply'
            #fx, _, cx, _, fy, cy, _, _, _ = camera.rgb.camera_info.K
            #print(camera.rgb.camera_info.K)
            #print('camera.rgb.range_dense=\n' + str(camera.rgb.range_dense))
            #camera.rgb.range_dense[camera.rgb.range_dense > 8] = -1
            ##generate_pointcloud(camera.rgb.filename, depth_image_filename, point_cloud_ply_filename, (fx+fy)/2, cx, cy, 1000)

            #import pypcd
            #x,y,z = generate_point_cloud2(camera.rgb.range_dense, cx, cy, fx, fy)
            #print(x)
            #print(y)
            #print(z)


            #xx = x.astype(np.float32).reshape(1, -1)
            #yy = y.astype(np.float32).reshape(1, -1)
            #zz = z.astype(np.float32).reshape(1, -1)

            #print(xx.size)
            #new_data = np.hstack((xx.T, yy.T, zz.T))

            ## Use the pypcd utility function to create a new point cloud from ndarray
            #new_cloud = pypcd.make_xyz_point_cloud(new_data)

            #import pprint
            ## pretty print the new metadata
            #pprint.pprint(new_cloud.get_metadata())
                ##{'count': [1, 1, 1, 1],
                ##'data': 'binary',
                ##'fields': ['x', 'y', 'z', 'rgb'],
                ##'height': 1,
                ##'points': 10000,
                ##'size': [4, 4, 4, 4],
                ##'type': ['F', 'F', 'F', 'F'],
                ##'version': 0.7,
                ##'viewpoint': [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                ##'width': 10000}

            ## Store the cloud uncompressed
            #new_cloud.save_pcd('new_cloud.pcd', compression='binary_compressed')
            ##new_cloud.save_pcd('new_cloud.pcd', compression='ascii')

            #arr = np.array([xf,yf,zf], dtype=[('x', 'f4'),('y', 'f4'), ('z', 'f4')])
            ##arr.astype(np.dtype({'names': ['x', 'y', 'z'], 'formats': [np.float32, np.float32, np.float32]}))
            #print(arr.dtype.names)
            #print(arr.shape)
            ##arr = np.swapaxes(arr,0,1)
            ##arr = np.swapaxes(arr,1,2)
            #print(arr.shape)

            #pc = pypcd.PointCloud.from_array(arr)

            ## save as binary compressed
            ##pc.save_pcd('bar.pcd', compression='binary_compressed')
            #pc.save_pcd('bar.pcd', compression='ascii')



        # ---------------------------------------
        # --- Features used to define which image will map each face
        # ---------------------------------------

        # ---------------------------------------
        # --- Compute valid camera indexes (cam_idxs) for each face
        # The is valid if all its vertices are projected inside the image
        # ---------------------------------------
        self.mesh.face_property('cam_idxs')
        print('Computing per camera valid faces:')
        for fh in tqdm(self.mesh.faces()):
            self.mesh.set_face_property('cam_idxs', fh, [])  # add empty list for each 'cameras' face property

            vidxs = []  # Compute vertice indexes for this face
            for vh in self.mesh.fv(fh):
                vidxs.append(vh.idx())

            for cam_idx, camera in enumerate(self.cameras):
                if camera.rgb.valid_pixs[vidxs[0]] and camera.rgb.valid_pixs[vidxs[1]] and camera.rgb.valid_pixs[
                    vidxs[2]]:
                    p = self.mesh.face_property('cam_idxs', fh)
                    p.append(cam_idx)
                    self.mesh.set_face_property('cam_idxs', fh, p)

        # ---------------------------------------
        # --- Compute face centers
        # ---------------------------------------
        print('Computing face centers:')
        for fh in tqdm(self.mesh.faces()):
            self.mesh.set_face_property('center', fh, [])  # add empty list for each 'center' face property

            x = [0, 0, 0]
            y = [0, 0, 0]
            z = [0, 0, 0]
            for i, vh in enumerate(self.mesh.fv(fh)):
                x[i], y[i], z[i] = self.mesh.point(vh)

            self.mesh.set_face_property('center', fh, [np.mean(x), np.mean(y), np.mean(z)])

        # ---------------------------------------
        # --- Compute face center_zray, a_zray, b_zray, c_zray
        # ---------------------------------------
        print('Ray tracing z for centers, a, b and c:')
        for fh in tqdm(self.mesh.faces()):
            cx, cy, cz = self.mesh.face_property('center', fh)
            x = [0, 0, 0]
            y = [0, 0, 0]
            z = [0, 0, 0]
            for i, vh in enumerate(self.mesh.fv(fh)):
                x[i], y[i], z[i] = self.mesh.point(vh)

            center_zray = []
            a_zray = []
            b_zray = []
            c_zray = []
            for cam_idx in self.mesh.face_property('cam_idxs', fh):
                camera = self.cameras[cam_idx]
                xcam, ycam, zcam = camera.rgb.matrix[0:3, 3]

                center_zray.append(self.getEuclideanDistance(cx, cy, cz, xcam, ycam, zcam))
                a_zray.append(self.getEuclideanDistance(x[0], y[0], z[0], xcam, ycam, zcam))
                b_zray.append(self.getEuclideanDistance(x[1], y[1], z[1], xcam, ycam, zcam))
                c_zray.append(self.getEuclideanDistance(x[2], y[2], z[2], xcam, ycam, zcam))

            self.mesh.set_face_property('center_zray', fh, center_zray)
            self.mesh.set_face_property('a_zray', fh, a_zray)
            self.mesh.set_face_property('b_zray', fh, b_zray)
            self.mesh.set_face_property('c_zray', fh, c_zray)

        # ---------------------------------------
        # --- Z buffering
        # ---------------------------------------
        if self.p['use_z_buffering'] == False:
            # Initialize property to true
            for fh in tqdm(self.mesh.faces()):
                cam_idxs = self.mesh.face_property('cam_idxs', fh)
                self.mesh.set_face_property('zbufs_valid', fh, [True] * len(cam_idxs))

        elif self.p['use_z_buffering'] == True:
            print('Computing z-buffering')

            if self.fileExists(self.p['zbuf_speedup_file']):  # load file if it exists

                pickle_obj = pickle.load(open(self.p['zbuf_speedup_file'], 'rb'))
                for i, fh in enumerate(tqdm(self.mesh.faces())):
                    self.mesh.set_face_property('zbufs_valid', fh, pickle_obj['zbuffering'][i])

                # Check if the loaded file is consistent with the current parameters
                if not self.p['num_cameras'] == pickle_obj['num_cameras']:
                    raise ValueError('Incorrect number of cameras. Perhaps the saved zbuffering file is not valid?')

                print('Loaded z-buffering from ' + self.p['zbuf_speedup_file'])

            else:  # file does not exist, compute z-bufering

                # create the 'zbufs_valid' propperty per face. It is a list with the same size as the list of 'cam_idxs', containing a bool which says if, for that face, each of the cameras is valid or not.
                for fh in tqdm(self.mesh.faces()):
                    cam_idxs = self.mesh.face_property('cam_idxs', fh)
                    self.mesh.set_face_property('zbufs_valid', fh, [True] * len(cam_idxs))

                for cam_idx, camera in enumerate(self.cameras):
                    print('Z-buffering camera ' + str(cam_idx) + ' of ' + str(len(self.cameras)))
                    camera.rgb.polygons = []
                    camera.rgb.polygons_z_dist = []
                    camera.rgb.polygons_face_handle = []

                    for fh in tqdm(self.mesh.faces()):
                        cam_idxs = self.mesh.face_property('cam_idxs', fh)
                        if not cam_idx in cam_idxs:
                            continue
                        else:
                            prop_idx = cam_idxs.index(cam_idx)  # the index of this camera in the face's properties
                            face_polygon = self.shapelyPolygonFromFaceProjection(self.mesh, camera, fh)
                            if not face_polygon.is_valid:
                                self.setFacePropertyValuePerIndex(self.mesh, fh, 'zbufs_valid', prop_idx, False)
                                continue

                        # Check if this new triangle intersects any other triangle already inserted in the list
                        z_dist = self.mesh.face_property('center_zray', fh)[prop_idx]
                        flg_insert = True
                        idxs_to_delete = []
                        for i, p in enumerate(camera.rgb.polygons):
                            if face_polygon.intersects(p):
                                if face_polygon.touches(p):  # no real intersection, just overlapping boundaries
                                    continue

                                if z_dist < camera.rgb.polygons_z_dist[
                                    i]:  # replace previous triangle with this one which is closer
                                    fh_to_remove = camera.rgb.polygons_face_handle[i]
                                    cam_idxs_to_remove = self.mesh.face_property('cam_idxs', fh_to_remove)
                                    prop_idx_to_remove = cam_idxs_to_remove.index(cam_idx)
                                    self.setFacePropertyValuePerIndex(self.mesh, fh_to_remove, 'zbufs_valid',
                                                                      prop_idx_to_remove, False)
                                    idxs_to_delete.append(i)
                                else:
                                    # camera.rgb.zbuf_valid[fh.idx(),0] = False
                                    zbufs_valid = self.mesh.face_property('zbufs_valid', fh)
                                    zbufs_valid[prop_idx] = False
                                    self.mesh.set_face_property('zbufs_valid', fh, zbufs_valid)
                                    flg_insert = False

                        # delete in reverse order
                        for i in sorted(idxs_to_delete, reverse=True):
                            del camera.rgb.polygons[i]
                            del camera.rgb.polygons_z_dist[i]
                            del camera.rgb.polygons_face_handle[i]

                        if flg_insert is True:
                            camera.rgb.polygons.append(face_polygon)
                            camera.rgb.polygons_z_dist.append(z_dist)
                            camera.rgb.polygons_face_handle.append(fh)
                            self.setFacePropertyValuePerIndex(self.mesh, fh, 'zbufs_valid', prop_idx, True)
                            # print('No intersection, adding new face. Now has ' + str(len(camera.rgb.polygons)))

                        # Visualize z-buffering
                        if self.p['view_z_buffering'] is True:
                            image = deepcopy(camera.rgb.image)
                            for i, p in enumerate(camera.rgb.polygons):
                                m = mapping(p)
                                coords = m['coordinates'][0]
                                x0 = int(round(coords[0][0]))
                                y0 = int(round(coords[0][1]))
                                x1 = int(round(coords[1][0]))
                                y1 = int(round(coords[1][1]))
                                x2 = int(round(coords[2][0]))
                                y2 = int(round(coords[2][1]))

                                color = (210, 0, 0)
                                cv2.line(image, (x0, y0), (x1, y1), color, 2)
                                cv2.line(image, (x1, y1), (x2, y2), color, 2)
                                cv2.line(image, (x2, y2), (x0, y0), color, 2)

                            cv2.namedWindow('Zbuffering', cv2.WINDOW_NORMAL)
                            cv2.imshow('Zbuffering', image)
                            cv2.waitKey(3)

                    if self.p['view_z_buffering'] is True:
                        cv2.waitKey(0)

                # Save to file for speeding up next time
                pickle_obj = self.p
                tmp_list = []  # a list of all the zbuf_valid per face
                for fh in tqdm(self.mesh.faces()):
                    tmp_list.append(self.mesh.face_property('zbufs_valid', fh))

                pickle_obj['zbuffering'] = tmp_list
                pickle.dump(pickle_obj, open(self.p['zbuf_speedup_file'], 'wb'), pickle.HIGHEST_PROTOCOL)

                print('Saved z-buffering to ' + self.p['zbuf_speedup_file'])



        # ---------------------------------------
        # --- Compute projected face areas
        # ---------------------------------------
        print('Computing projected triangle areas:')
        for fh in tqdm(self.mesh.faces()):
            idxs = []
            for vh in self.mesh.fv(fh):
                idxs.append(vh.idx())

            areas = []
            for cam_idx in self.mesh.face_property('cam_idxs', fh):
                camera = self.cameras[cam_idx]
                x0, y0 = camera.rgb.pixs[0:2, idxs[0]]
                x1, y1 = camera.rgb.pixs[0:2, idxs[1]]
                x2, y2 = camera.rgb.pixs[0:2, idxs[2]]
                areas.append(self.getFaceArea(x0, y0, x1, y1, x2, y2))

            self.mesh.set_face_property('cam_areas', fh, areas)  # add empty list for each 'cameras' face property

        # ---------------------------------------
        # --- Compute per face z_gap
        # ---------------------------------------
        print('Computing z gap ...')
        for fh in tqdm(self.mesh.faces()):
            idxs = []
            for vh in self.mesh.fv(fh):
                idxs.append(vh.idx())

            a_zrays = self.mesh.face_property('a_zray', fh)
            b_zrays = self.mesh.face_property('b_zray', fh)
            c_zrays = self.mesh.face_property('c_zray', fh)

            zgaps = []
            cam_idxs = self.mesh.face_property('cam_idxs', fh)
            for cam_idx in cam_idxs:  # iterate only valid cameras to speed up
                prop_idx = cam_idxs.index(cam_idx)
                camera = self.cameras[cam_idx]

                a_zray = a_zrays[prop_idx]
                b_zray = b_zrays[prop_idx]
                c_zray = c_zrays[prop_idx]

                xa, ya = camera.rgb.pixs[0:2, idxs[0]]
                xb, yb = camera.rgb.pixs[0:2, idxs[1]]
                xc, yc = camera.rgb.pixs[0:2, idxs[2]]

                a_zmeas = camera.rgb.range_dense[ya, xa]
                b_zmeas = camera.rgb.range_dense[yb, xb]
                c_zmeas = camera.rgb.range_dense[yc, xc]

                zgap = max([abs(a_zray - a_zmeas), abs(b_zray - b_zmeas), abs(c_zray - c_zmeas)])
                zgaps.append(zgap)

                # print('face ' + str(fh.idx()) + ' zray [a,b,c] = ' + str(a_zray) + ', ' + str(b_zray) + ' ' + str(c_zray) + ' zmeas = ' + str(a_zmeas) + ', ' + str(b_zmeas) + ', ' + str(c_zmeas) + ' consistency = ' + str(consistency))

            self.mesh.set_face_property('zgaps', fh, zgaps)

        # ---------------------------------------
        # --- Print mesh info
        # ---------------------------------------
        step = 3000
        for i, fh in enumerate(self.mesh.faces()):
            if (i % step) == 0:
                self.printFaceInfo(fh)

        # ---------------------------------------
        # --- Select triangles to use for texture mapping
        # ---------------------------------------
        print('Selecting per face cameras for texture mapping:')
        self.mesh.face_property('sel_cam_idx')  # name of the camera which maps color

        # --- Random ----------------------------------------------------------
        if self.p['method_name'] == 'random':
            print('Using selection method ' + self.p['method_name'])
            for fh in tqdm(self.mesh.faces()):
                self.setselCameraIdxFromCost(self.mesh, self.cameras, fh, self.costRandomCamera,
                                             self.p['use_z_buffering'], self.p['use_z_filtering'])

        # --- First Camera ----------------------------------------------------
        elif self.p['method_name'] == 'first_camera':
            print('Using selection method ' + self.p['method_name'])
            for fh in tqdm(self.mesh.faces()):
                self.setselCameraIdxFromCost(self.mesh, self.cameras, fh, self.costFirstCamera,
                                             self.p['use_z_buffering'], self.p['use_z_filtering'])

        # --- Last Camera -----------------------------------------------------
        elif self.p['method_name'] == 'last_camera':
            print('Using selection method ' + self.p['method_name'])
            for fh in tqdm(self.mesh.faces()):
                self.setselCameraIdxFromCost(self.mesh, self.cameras, fh, self.costLastCamera,
                                             self.p['use_z_buffering'], self.p['use_z_filtering'])

        # --- Maximum area triangle -------------------------------------------
        elif self.p['method_name'] == 'largest_projected_triangle_area':
            print('Using selection method ' + self.p['method_name'])
            for fh in tqdm(self.mesh.faces()):
                self.setselCameraIdxFromCost(self.mesh, self.cameras, fh, self.costProjectedTriangleArea,
                                             self.p['use_z_buffering'], self.p['use_z_filtering'])

        # --- Random seed propagation -------------------------------------------
        elif self.p['method_name'] == 'random_seed_propagation':
            print('Using selection method ' + self.p['method_name'])

            # TODO Camera in which we see the propagation is hardcoded. Should be better defined
            camera = self.cameras[2]
            cam_idx = 2

            if self.p['view_selection_animation'] is True:  # Draw
                image = deepcopy(camera.rgb.image)
                self.drawMesh(self.mesh, cam_idx, image, color_sel=None)

            # Create 'visited' property in every face and initialize it to False
            for fh in self.mesh.faces():
                self.mesh.set_face_property('to_be_visited', fh, False)
                self.mesh.set_face_property('visited', fh, False)

            # seed_fh = self.mesh.face_handle(randint(0, self.mesh.n_faces())) #randomize starting face
            seed_fh = self.mesh.face_handle(20)

            # TODO Define this is a better way
            self.mesh.set_face_property('parent_sel_cam_idx', seed_fh, 4)

            num_visits = 0
            lfaces = [seed_fh]  # a list of propagation tuples
            while len(lfaces) > 0:  # visit all faces in lfaces list
                curr_fh = lfaces[0]
                lfaces.pop(0)
                num_visits = num_visits + 1

                # Select camera for this face
                self.setselCameraIdxFromCost(self.mesh, self.cameras, curr_fh, self.costFunction1,
                                             self.p['use_z_buffering'], self.p['use_z_filtering'])
                sel_cam_idx = self.mesh.face_property('sel_cam_idx', curr_fh)

                # print('\nVisited face ' + str(curr_fh.idx()) + ' scam_idx = ' + str(sel_cam_idx))

                for fh in self.mesh.ff(curr_fh):  # iterate over all edge-neighboring faces
                    if self.mesh.face_property('to_be_visited', fh) == True:  # skip if already visited
                        continue

                    self.mesh.set_face_property('to_be_visited', fh, True)
                    self.mesh.set_face_property('parent_sel_cam_idx', fh, sel_cam_idx)

                    values = self.costFunction1(self.mesh, fh, self.p['use_z_buffering'], self.p['use_z_filtering'])
                    # print('Adding neighbor face ' + str(fh.idx()) + ' with ' + str(values))
                    cam_idxs = [x[0] for x in values]
                    costs = [x[1] for x in values]

                    if sel_cam_idx in cam_idxs:
                        cost_idx = cam_idxs.index(sel_cam_idx)
                        cost = costs[cost_idx]
                        if not np.isinf(cost):
                            lfaces.insert(0, fh)
                        else:
                            lfaces.append(fh)
                    else:
                        lfaces.append(fh)

                    if self.p['view_selection_animation'] is True:  # Draw
                        # if cam_idx in self.mesh.face_property('cam_idxs', fh):
                        self.drawFaceProjection(self.mesh, fh, camera, image, color=(255, 0, 255), text=str(fh.idx()))

                if self.p['view_selection_animation'] is True:  # Draw
                    # if cam_idx in self.mesh.face_property('cam_idxs', curr_fh):
                    if not sel_cam_idx is None:
                        # color = (25,25,25)
                        # else:
                        r, g, b = cmap_cameras[sel_cam_idx, 0:3] * 255
                        color = (int(b), int(g), int(r))
                        self.drawFaceProjection(self.mesh, curr_fh, camera, image, color=color, text=str(curr_fh.idx()))

                    # print('lfaces =  ' + str([int(x.idx()) for x in lfaces]))

                    # Draw
                    cv2.namedWindow('Propagation', cv2.WINDOW_NORMAL)
                    cv2.imshow('Propagation', image)
                    cv2.waitKey(5)
                    # cv2.waitKey(0)

                # print('lfaces has ' + str(len(lfaces)) + ' faces')

            print('Finished visiting ' + str(num_visits) + ' faces. Mesh contains ' + str(self.mesh.n_faces()))

        # ---------------------------------------
        # --- Detect border triangles
        # ---------------------------------------
        self.border_faces = []
        for curr_fh in self.mesh.faces():
            sel_cam_idx = self.mesh.face_property('sel_cam_idx', curr_fh)
            self.mesh.set_face_property('is_border', curr_fh, False)
            for fh in self.mesh.ff(curr_fh):  # iterate over all edge-neighboring faces
                fh_sel_cam_idx = self.mesh.face_property('sel_cam_idx', fh)
                if not sel_cam_idx == fh_sel_cam_idx and not fh_sel_cam_idx is None:
                    self.mesh.set_face_property('is_border', curr_fh, True)
                    self.border_faces.append(curr_fh.idx())
                    break

        # TODO Camera in which we see the propagation is hardcoded. Should be better defined
        # camera = self.cameras[0]
        cam_idx = 2
        image = deepcopy(self.cameras[cam_idx].rgb.image)
        self.drawMesh(self.mesh, cam_idx, image, color_all=(20, 20, 20))
        self.drawFacesOnImage(self.mesh, cam_idx, image,
                              lambda mesh, fh, cam_idx: mesh.face_property('is_border', fh) == True,
                              color=(0, 200, 200))

        cv2.namedWindow('BorderTriangles', cv2.WINDOW_NORMAL)
        cv2.imshow('BorderTriangles', image)
        image_file = self.p['path_results'] + 'border_' + os.path.basename(self.cameras[cam_idx].rgb.filename) 
        print('Saving border triangles image ' + image_file)
        cv2.imwrite(image_file, image)
        cv2.waitKey(5)

        # ---------------------------------------
        # --- Smooth texture on the border triangles (using trilinear cooridinates)
        # ---------------------------------------

        # Create new images where the smoothed faces will be copied to. OBJ parser will use these images as materials for texturing.
        for camera in self.cameras: 
            camera.rgb.image_smoothed = deepcopy(camera.rgb.image)


        if self.p['border_face_smoothing'] == True:
            win_names = ['CurrCamera', 'CurrFace', 'n0', 'n1', 'n2', 'n0 Camera', 'n1 Camera', 'n2 Camera', 'w_n0', 'w_n1',
                         'w_n2', 'Averaged']
            # win_names = ['CurrCamera', 'CurrFace', 'n0', 'n1', 'n2', 'n0 Camera', 'n1 Camera', 'n2 Camera', 'Averaged', 'CameraAveraged']
            for win_name in win_names:
                cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
                cv2.imshow(win_name, cv2.WINDOW_NORMAL)

            print('Smoothing ' + str(len(self.border_faces)) + ' border faces')
            for c_fh_idx in tqdm(self.border_faces): # only border faces
                c_fh = self.mesh.face_handle(c_fh_idx) # get face handle from idx

                if self.mesh.face_property('is_border', c_fh) == False: # check if this is a border face
                    raise("This face should be marked as border and it is not")

                sel_cam_idx = self.mesh.face_property('sel_cam_idx', c_fh)

                if sel_cam_idx is None:
                    continue

                c_fh_cam_idxs = self.mesh.face_property('cam_idxs', c_fh)

                # Visual debug stuff
                image = deepcopy(self.cameras[sel_cam_idx].rgb.image)
                images = [0 * image, 0 * image, 0 * image]

                # get the triangle's cropped image for c_fh and sel_cam_idx
                c_image, crop_rect, c_mask = self.warpFace2(self.mesh, self.cameras[sel_cam_idx], self.cameras[sel_cam_idx], c_fh, c_fh)

                global_weights = np.zeros((4,),dtype=np.float)
                #global_weights[0] = 0.0
                for n_idx, n_fh in enumerate(self.mesh.ff(c_fh)):  # iterate over all edge-ngbing faces
                    n_sel_cam_idx = self.mesh.face_property('sel_cam_idx', n_fh)
                    # print('Neighbor ' + str(n_idx) + ' (face ' + str(n_fh.idx()) + ') sel_cam_idx ' + str(n_sel_cam_idx))

                    if not n_sel_cam_idx in c_fh_cam_idxs or n_sel_cam_idx is None:  # if neighbor's selected camera is not available to paint c_fh ... or if ... n_sel_cam_idx is None
                        global_weights[n_idx] = 0.0
                        # print('Neighbor ' + str(n_idx) + ' sel_cam_idx is None or not avalable for c_fh')
                    else:
                        images[n_idx], _, _ = self.warpFace2(self.mesh, self.cameras[n_sel_cam_idx],
                                                          self.cameras[sel_cam_idx], c_fh, c_fh)
                        global_weights[n_idx] = 1.0

                #Normalize global weights
                global_weights /= sum(global_weights)
                # print('global_weights = ' + str(global_weights))

                # Create a structure that contains the coordinates of the edges
                idxs = []
                for vh in self.mesh.fv(c_fh):  # handle to the face's vertices
                    idxs.append(vh.idx())

                x0, y0 = self.cameras[sel_cam_idx].rgb.pixs[0:2, idxs[0]]
                x1, y1 = self.cameras[sel_cam_idx].rgb.pixs[0:2, idxs[1]]
                x2, y2 = self.cameras[sel_cam_idx].rgb.pixs[0:2, idxs[2]]

                cv2.line(image, (x2, y2), (x0, y0), (0, 0, 255), thickness=5)
                cv2.line(image, (x0, y0), (x1, y1), (0, 255, 0), thickness=5)
                cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), thickness=5)

                # change coordinates for the cropped image
                # print(crop_rect)
                dx = crop_rect[0]
                dy = crop_rect[1]
                x0_c = float(x0 - dx)
                x1_c = float(x1 - dx)
                x2_c = float(x2 - dx)
                y0_c = float(y0 - dy)
                y1_c = float(y1 - dy)
                y2_c = float(y2 - dy)

                #cv2.line(c_image, (x2_c, y2_c), (x0_c, y0_c), (0, 0, 255), thickness=1)
                #cv2.line(c_image, (x0_c, y0_c), (x1_c, y1_c), (0, 255, 0), thickness=1)
                #cv2.line(c_image, (x1_c, y1_c), (x2_c, y2_c), (255, 0, 0), thickness=1)

                # Create shapely edges
                #edges = [LineString([(x2_c, y2_c), (x0_c, y0_c)]), LineString([(x0_c, y0_c), (x1_c, y1_c)]),
                         #LineString([(x1_c, y1_c), (x2_c, y2_c)])]

                # For every pixel, compute distance to each edge and create a weight mask
                avg_image = c_image * 0  # initialize the output image
                h, w, nc = avg_image.shape
                w_n0 = np.zeros((h, w), dtype=np.float)  # type: ndarray
                w_n1 = np.zeros((h, w), dtype=np.float)
                w_n2 = np.zeros((h, w), dtype=np.float)

                for l in range(0, h):
                    for c in range(0, w):
                        # These coordinates are already cropped because we are iterating on the cropped image
                        denominator =  (y1_c - y2_c) * (x0_c - x2_c) + (x2_c - x1_c) * (y0_c - y2_c) 
                        Bv0 = ( (y1_c - y2_c) * (c - x2_c) + (x2_c - x1_c) * (l - y2_c) ) / denominator
                        Bv1 = ( (y2_c - y0_c) * (c - x2_c) + (x0_c - x2_c) * (l - y2_c) ) / denominator
                        Bv2 = 1 - Bv0 - Bv1

                        if Bv0 <= 0 or Bv1 <= 0 or Bv2 <= 0:
                            w_n0[l, c] = w_n1[l, c] = w_n2[l, c] = -1
                        else:
                            w_n0[l, c] = exp(Bv2 + Bv0)**8
                            w_n1[l, c] = exp(Bv0 + Bv1)**8
                            w_n2[l, c] = exp(Bv1 + Bv2)**8

                        #print('Bary ' + str(Bv0) + ' ' + str(Bv1) + ' ' + str(Bv2))


                #Normalize weights
                total = global_weights[0]*w_n0 + global_weights[1]*w_n1 + global_weights[2]*w_n2
                w_n0 /= total
                w_n1 /= total
                w_n2 /= total

                #Weighted sum for cycle
                for l in range(0, h):
                    for c in range(0, w):
                        if w_n0[l, c] == -1:
                            old_pixel_value = c_image[l,c]
                            avg_image[l,c,:] = old_pixel_value
                        else:
                            n0_pixel_value = images[0][l,c]
                            n1_pixel_value = images[1][l,c]
                            n2_pixel_value = images[2][l,c]

                            new_pixel_value = global_weights[0] * w_n0[l,c] * n0_pixel_value + \
                                              global_weights[1] * w_n1[l,c] * n1_pixel_value + \
                                              global_weights[2] * w_n2[l,c] * n2_pixel_value
                            avg_image[l,c,:] = new_pixel_value

                #Weighted sum (matrix based)

                #avg_image = avg_image * 0
                #avg_image[:,:,1] = avg_image[:,:,1] + 255


                #Copy triangular region of the rectangular patch to the output image
                image_out = self.cameras[sel_cam_idx].rgb.image_smoothed
                r2 = crop_rect
                image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] * ( (1.0, 1.0, 1.0) - c_mask )
                image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] + avg_image


                # Drawing triangles and warped face
                if 1 and c_fh_idx >= 60:

                    print('Face ' + str(c_fh.idx()) + ' sel_cam_idx ' + str(sel_cam_idx))
                    print('\nFace ' + str(c_fh.idx()) + ' cam_idxs ' + str(c_fh_cam_idxs))


                    for n_idx, n_fh in enumerate(self.mesh.ff(c_fh)):  # iterate over all edge-ngbing faces
                        # self.drawFaceProjection(self.mesh, n_fh, self.cameras[sel_cam_idx], image, color=(255,255,0), linewidth = 2, text='N' + str(n_idx) + '_cidx ' + str(n_sel_cam_idx))
                        self.drawFaceProjection(self.mesh, n_fh, self.cameras[sel_cam_idx], image, color=(255, 255, 0),
                                                linewidth=2, text=str(n_idx))
                        cv2.imshow('n' + str(n_idx), images[n_idx])
                        n_sel_cam_idx = self.mesh.face_property('sel_cam_idx', n_fh)
                        if not n_sel_cam_idx is None:
                            n_image = deepcopy(self.cameras[n_sel_cam_idx].rgb.image)
                            self.drawFaceProjection(self.mesh, c_fh, self.cameras[n_sel_cam_idx], n_image, color=(255, 0, 255), linewidth=2, text=None)
                            cv2.imshow('n' + str(n_idx) + ' Camera', n_image)

                    #for cam_idx, camera in enumerate(self.cameras):
                        #cv2.namedWindow('Camera ' + str(cam_idx), cv2.WINDOW_NORMAL)
                        #cv2.imshow('Camera ' + str(cam_idx), camera.rgb.image_smoothed)



                    self.normImShow('w_n0', w_n0)
                    self.normImShow('w_n1', w_n1)
                    self.normImShow('w_n2', w_n2)
                    #cv2.imshow()
                    #cv2.imshow('w_n1', w_n1)
                    #cv2.imshow('w_n2', w_n2)

                    cv2.imshow('CurrFace', c_image)

                    self.drawFaceProjection(self.mesh, c_fh, self.cameras[sel_cam_idx], image, color=(255, 0, 0),
                                            linewidth=1, text=None)

                    cv2.imshow('CurrCamera', image)

                    cv2.imshow('Averaged', avg_image)
                    #cv2.imshow('CameraAveraged', self.cameras[sel_cam_idx].rgb.image)

                    self.keyPressManager()


        cv2.waitKey(0)
        #exit(0)

        # ---------------------------------------
        # --- Smooth texture on the border triangles (using trilinear cooridinates)
        # ---------------------------------------

        # Create new images for each camera where the smoothed faces will be copied to
        #for camera in self.cameras: 
            #camera.rgb.image_smoothed = deepcopy(camera.rgb.image)


        #if self.p['border_face_smoothing'] == True:
            #win_names = ['CurrCamera', 'CurrFace', 'n0', 'n1', 'n2', 'n0 Camera', 'n1 Camera', 'n2 Camera', 'w_n0', 'w_n1',
                         #'w_n2', 'Averaged', 'CameraAveraged', 'Camera 0', 'Camera 1', 'Camera 2', 'Camera 3']
            ## win_names = ['CurrCamera', 'CurrFace', 'n0', 'n1', 'n2', 'n0 Camera', 'n1 Camera', 'n2 Camera', 'Averaged', 'CameraAveraged']
            #for win_name in win_names:
                #cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
                #cv2.imshow(win_name, cv2.WINDOW_NORMAL)


            #print('Smoothing ' + str(len(self.border_faces)) + ' border faces')
            #for c_fh_idx in tqdm(self.border_faces): # only border faces
                #c_fh = self.mesh.face_handle(c_fh_idx) # get face handle from idx

                #if self.mesh.face_property('is_border', c_fh) == False: # check if this is a border face
                    #raise("This face should be marked as border and it is not")

                #sel_cam_idx = self.mesh.face_property('sel_cam_idx', c_fh)

                #if sel_cam_idx is None:
                    #continue

                #c_fh_cam_idxs = self.mesh.face_property('cam_idxs', c_fh)

                ## Visual debug stuff
                #image = deepcopy(self.cameras[sel_cam_idx].rgb.image)
                #images = [0 * image, 0 * image, 0 * image]

                ## get the triangle's cropped image for c_fh and sel_cam_idx
                #c_image, crop_rect, c_mask = self.warpFace2(self.mesh, self.cameras[sel_cam_idx], self.cameras[sel_cam_idx],
                                                    #c_fh, c_fh)

                #global_weights = np.zeros((4,),dtype=np.float)
                ##global_weights[0] = 0.0
                #for n_idx, n_fh in enumerate(self.mesh.ff(c_fh)):  # iterate over all edge-ngbing faces
                    #n_sel_cam_idx = self.mesh.face_property('sel_cam_idx', n_fh)
                    ## print('Neighbor ' + str(n_idx) + ' (face ' + str(n_fh.idx()) + ') sel_cam_idx ' + str(n_sel_cam_idx))

                    #if not n_sel_cam_idx in c_fh_cam_idxs or n_sel_cam_idx is None:  # if neighbor's selected camera is not available to paint c_fh ... or if ... n_sel_cam_idx is None
                        #global_weights[n_idx] = 0.0
                        ## print('Neighbor ' + str(n_idx) + ' sel_cam_idx is None or not avalable for c_fh')
                    #else:
                        #images[n_idx], _, _ = self.warpFace2(self.mesh, self.cameras[n_sel_cam_idx],
                                                          #self.cameras[sel_cam_idx], c_fh, c_fh)
                        #global_weights[n_idx] = 1.0

                ##Normalize global weights
                #global_weights /= sum(global_weights)

                ## print('global_weights = ' + str(global_weights))

                ## Create a structure that contains the coordinates of the edges
                #idxs = []
                #for vh in self.mesh.fv(c_fh):  # handle to the face's vertices
                    #idxs.append(vh.idx())

                #x0, y0 = self.cameras[sel_cam_idx].rgb.pixs[0:2, idxs[0]]
                #x1, y1 = self.cameras[sel_cam_idx].rgb.pixs[0:2, idxs[1]]
                #x2, y2 = self.cameras[sel_cam_idx].rgb.pixs[0:2, idxs[2]]

                #cv2.line(image, (x2, y2), (x0, y0), (0, 0, 255), thickness=5)
                #cv2.line(image, (x0, y0), (x1, y1), (0, 255, 0), thickness=5)
                #cv2.line(image, (x1, y1), (x2, y2), (255, 0, 0), thickness=5)

                ## change coordinates for the cropped image
                ## print(crop_rect)
                #dx = crop_rect[0]
                #dy = crop_rect[1]
                #x0_c = x0 - dx
                #x1_c = x1 - dx
                #x2_c = x2 - dx
                #y0_c = y0 - dy
                #y1_c = y1 - dy
                #y2_c = y2 - dy

                ## cv2.line(c_image, (x2_c, y2_c), (x0_c, y0_c), (0, 0, 255), thickness=1)
                ## cv2.line(c_image, (x0_c, y0_c), (x1_c, y1_c), (0, 255, 0), thickness=1)
                ## cv2.line(c_image, (x1_c, y1_c), (x2_c, y2_c), (255, 0, 0), thickness=1)

                ## Create shapely edges
                #edges = [LineString([(x2_c, y2_c), (x0_c, y0_c)]), LineString([(x0_c, y0_c), (x1_c, y1_c)]),
                         #LineString([(x1_c, y1_c), (x2_c, y2_c)])]

                ## For every pixel, compute distance to each edge and create a weight mask
                #avg_image = c_image * 0  # initialize the output image
                #h, w, nc = avg_image.shape
                #w_n0 = np.zeros((h, w), dtype=np.float)  # type: ndarray
                #w_n1 = np.zeros((h, w), dtype=np.float)
                #w_n2 = np.zeros((h, w), dtype=np.float)

                #c_image_diagonal = sqrt(w*w + h*h) #maximum distance for normalization
                #if c_image_diagonal == 0:
                    #continue

                ##print("global_weights = " + str(global_weights))
                #for l in range(0, h):
                    #for c in range(0, w):
                        #p = Point(c, l)  # create a shapely point to compute distance to shapely line. These
                        ## coordinates are already cropped because we are iterating on the cropped image
                        #w_n0[l, c] = 1 - (edges[0].distance(p) / c_image_diagonal)**8
                        #w_n1[l, c] = 1 - (edges[1].distance(p) / c_image_diagonal)**8
                        #w_n2[l, c] = 1 - (edges[2].distance(p) / c_image_diagonal)**8

                ##Normalize weights
                #total = global_weights[0]*w_n0 + global_weights[1]*w_n1 + global_weights[2]*w_n2
                #w_n0 /= total
                #w_n1 /= total
                #w_n2 /= total

                ##Weighted sum for cycle
                #for l in range(0, h):
                    #for c in range(0, w):
                        ##old_pixel_value = c_image[l,c]
                        #n0_pixel_value = images[0][l,c]
                        #n1_pixel_value = images[1][l,c]
                        #n2_pixel_value = images[2][l,c]

                        #new_pixel_value = global_weights[0] * w_n0[l,c] * n0_pixel_value + \
                                          #global_weights[1] * w_n1[l,c] * n1_pixel_value + \
                                          #global_weights[2] * w_n2[l,c] * n2_pixel_value
                        #avg_image[l,c,:] = new_pixel_value

                ##Weighted sum (matrix based)

                ##avg_image = avg_image * 0
                ##avg_image[:,:,1] = avg_image[:,:,1] + 255


                ##Copy triangular region of the rectangular patch to the output image
                #image_out = self.cameras[sel_cam_idx].rgb.image_smoothed
                #r2 = crop_rect
                #image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] * ( (1.0, 1.0, 1.0) - c_mask )
                #image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] + avg_image


                ## Drawing triangles and warped face
                #if 0 and c_fh_idx >= 48:

                    #print('Face ' + str(c_fh.idx()) + ' sel_cam_idx ' + str(sel_cam_idx))
                    #print('\nFace ' + str(c_fh.idx()) + ' cam_idxs ' + str(c_fh_cam_idxs))


                    #for n_idx, n_fh in enumerate(self.mesh.ff(c_fh)):  # iterate over all edge-ngbing faces
                        ## self.drawFaceProjection(self.mesh, n_fh, self.cameras[sel_cam_idx], image, color=(255,255,0), linewidth = 2, text='N' + str(n_idx) + '_cidx ' + str(n_sel_cam_idx))
                        #self.drawFaceProjection(self.mesh, n_fh, self.cameras[sel_cam_idx], image, color=(255, 255, 0),
                                                #linewidth=2, text=str(n_idx))
                        #cv2.imshow('n' + str(n_idx), images[n_idx])
                        ##n_sel_cam_idx = self.mesh.face_property('sel_cam_idx', n_fh)
                        ##if not n_sel_cam_idx is None:
                            ##n_image = deepcopy(self.cameras[n_sel_cam_idx].rgb.image)
                            ##self.drawFaceProjection(self.mesh, c_fh, self.cameras[n_sel_cam_idx], n_image,
                                                    ##color=(255, 0, 255), linewidth=2, text=None)
                            ##cv2.imshow('n' + str(n_idx) + ' Camera', n_image)

                    #for cam_idx, camera in enumerate(self.cameras):
                        #cv2.namedWindow('Camera ' + str(cam_idx), cv2.WINDOW_NORMAL)
                        #cv2.imshow('Camera ' + str(cam_idx), camera.rgb.image_smoothed)


                    ##cv2.imshow('w_n0', w_n0)
                    ##cv2.imshow('w_n1', w_n1)
                    ##cv2.imshow('w_n2', w_n2)

                    #cv2.imshow('CurrFace', c_image)

                    #self.drawFaceProjection(self.mesh, c_fh, self.cameras[sel_cam_idx], image, color=(255, 0, 0),
                                            #linewidth=1, text=None)

                    #cv2.imshow('CurrCamera', image)

                    #cv2.imshow('Averaged', avg_image)
                    ##cv2.imshow('CameraAveraged', self.cameras[sel_cam_idx].rgb.image)

                    #self.keyPressManager()


        #cv2.waitKey(0)
        ##exit(0)

        # ---------------------------------------
        # --- Compute the texture coordinates given the sel_cam_idx and the projections of the triangles
        # ---------------------------------------

        print('Computing list of sel pixel coordinates ...')
        self.mesh.face_property('uv_idxs')
        self.pixcoords = []  # an empty list with all the pixel coordinates (to be converted to texture coordinates aftwewards)

        for fh in tqdm(self.mesh.faces()):
            sel_cam_idx = self.mesh.face_property('sel_cam_idx', fh)
            if sel_cam_idx is None:
                continue
            camera = self.cameras[sel_cam_idx]
            uv_idxs = []

            for vh in self.mesh.fv(fh):
                # Compute pixel coordinates
                pix = tuple(camera.rgb.pixs[:, vh.idx()])

                # Compute pixel coordinates index in pixcoord list (append to list if they do not exist)
                if pix not in self.pixcoords:
                    self.pixcoords.append(pix)
                    uv_idxs.append(len(self.pixcoords) - 1)
                else:
                    uv_idxs.append(self.pixcoords.index(pix))

            self.mesh.set_face_property('uv_idxs', fh, uv_idxs)

            # Create texcoords list from pixcoords list
        print('Computing texture coordinates from pixel coordinates...')
        self.texcoords = []
        W = self.cameras[0].rgb.w  # Assume all images have the same size
        H = self.cameras[0].rgb.h  # Assume all images have the same size
        for pixcoord in tqdm(self.pixcoords):
            texcoords = (float(pixcoord[0]) / W, 1 - float(pixcoord[1]) / H)
            self.texcoords.append(texcoords)  # uv tex coordinates

        # ---------------------------------------
        # --- Visualization
        # ---------------------------------------

        #self.drawFacesOnImages(0, draw_zbuffering=True, draw_zfilt=False, draw_valid=False, draw_selected=False)
        self.drawFacesOnImages(0)



        self.mesh.request_face_colors()
        print('has_face_colors = ' + str(self.mesh.has_face_colors()))

        for fh in tqdm(self.mesh.faces()):
            color = self.mesh.color(fh)
            sel_cam_idx = self.mesh.face_property('sel_cam_idx', fh)
            print(sel_cam_idx)
            if sel_cam_idx is None:
                color[0] = 0.2
                color[1] = 0.2
                color[2] = 0.2
            else:
                r, g, b = cmap_cameras[sel_cam_idx, 0:3]
                color[0] = r
                color[1] = g
                color[2] = b

            color[3] = 0.2
            self.mesh.set_color(fh, color)
            # print(self.mesh.color(fh))

        # ---------------------------------------
        # --- Write obj
        # ---------------------------------------
        self.writeObjFile()

        # ---------------------------------------
        # --- Save Texured Mesh
        # ---------------------------------------
        self.p['output_mesh_filename'] = 'tmp2.obj'

        #self.mesh = om.read_trimesh(self.p['mesh_filename'])

        #set_color(self: openmesh.TriMesh, arg0: openmesh.FaceHandle, arg1: numpy.ndarray[float32]) -> None

        om.write_mesh(filename=self.p['output_mesh_filename'], \
                      mesh=self.mesh, \
                      binary=False, \
                      msb=False, \
                      lsb=False, \
                      swap=False, \
                      vertex_normal=True, \
                      vertex_color=False, \
                      vertex_tex_coord=False, \
                      halfedge_tex_coord=False, \
                      edge_color=False, \
                      face_normal=False, \
                      face_color=True, \
                      color_alpha=False, \
                      color_float=False \
                      )

        # self.loadMeshObj(self.p['output_mesh_filename'])
        # self.visualize3DModel()
        print('Done writing tmp2.obj')

    def warpFace2(self, mesh, camera_in, camera_out, fh_in, fh_out):
        # Read input image and convert to float
        image_in = camera_in.rgb.image

        # Output image is set to white
        # image_out = 0 * np.ones(image_in.shape, dtype = image_in.dtype)

        # Define input and output triangles 
        idxs = []
        for vh in mesh.fv(fh_in):  # handle to the face's vertices
            idxs.append(vh.idx())
        xa, ya = camera_in.rgb.pixs[0:2, idxs[0]]
        xb, yb = camera_in.rgb.pixs[0:2, idxs[1]]
        xc, yc = camera_in.rgb.pixs[0:2, idxs[2]]
        tri1 = np.float32([[[xa, ya], [xb, yb], [xc, yc]]])

        idxs = []
        for vh in mesh.fv(fh_out):  # handle to the face's vertices
            idxs.append(vh.idx())
        xa, ya = camera_out.rgb.pixs[0:2, idxs[0]]
        xb, yb = camera_out.rgb.pixs[0:2, idxs[1]]
        xc, yc = camera_out.rgb.pixs[0:2, idxs[2]]
        tri2 = np.float32([[[xa, ya], [xb, yb], [xc, yc]]])

        # Find bounding box. 
        r1 = cv2.boundingRect(tri1)
        r2 = cv2.boundingRect(tri2)

        # Offset points by left top corner of the 
        # respective rectangles

        tri1Cropped = []
        tri2Cropped = []

        for i in xrange(0, 3):
            tri1Cropped.append(((tri1[0][i][0] - r1[0]), (tri1[0][i][1] - r1[1])))
            tri2Cropped.append(((tri2[0][i][0] - r2[0]), (tri2[0][i][1] - r2[1])))

        # Apply warpImage to small rectangular patches
        img1Cropped = image_in[r1[1]:r1[1] + r1[3], r1[0]:r1[0] + r1[2]]

        # Given a pair of triangles, find the affine transform.
        warpMat = cv2.getAffineTransform(np.float32(tri1Cropped), np.float32(tri2Cropped))

        # Apply the Affine Transform just found to the src image
        image_outCropped = cv2.warpAffine(img1Cropped, warpMat, (r2[2], r2[3]), None, flags=cv2.INTER_LINEAR,
                                          borderMode=cv2.BORDER_REFLECT_101)

        # Get mask by filling triangle
        mask = np.zeros((r2[3], r2[2], 3), dtype=np.float32)
        cv2.fillConvexPoly(mask, np.int32(tri2Cropped), (1.0, 1.0, 1.0), 16, 0);

        return (image_outCropped * mask).astype(image_in.dtype), r2, mask
        # cv2.namedWindow('image_out', cv2.WINDOW_NORMAL)
        # cv2.imshow('image_out', image222)

        # Apply mask to cropped region
        # image_outCropped = image_outCropped * mask

        # Copy triangular region of the rectangular patch to the output image
        # image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] * ( (1.0, 1.0, 1.0) - mask )
        # image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] = image_out[r2[1]:r2[1]+r2[3], r2[0]:r2[0]+r2[2]] + image_outCropped

    def warpFace(self, mesh, camera_in, camera_out, fh_in, fh_out):
        # Read input image and convert to float
        image_in = camera_in.rgb.image

        # Output image is set to white
        image_out = 0 * np.ones(image_in.shape, dtype=image_in.dtype)

        # Define input and output triangles 
        idxs = []
        for vh in mesh.fv(fh_in):  # handle to the face's vertices
            idxs.append(vh.idx())
        xa, ya = camera_in.rgb.pixs[0:2, idxs[0]]
        xb, yb = camera_in.rgb.pixs[0:2, idxs[1]]
        xc, yc = camera_in.rgb.pixs[0:2, idxs[2]]
        tri1 = np.float32([[[xa, ya], [xb, yb], [xc, yc]]])

        idxs = []
        for vh in mesh.fv(fh_out):  # handle to the face's vertices
            idxs.append(vh.idx())
        xa, ya = camera_out.rgb.pixs[0:2, idxs[0]]
        xb, yb = camera_out.rgb.pixs[0:2, idxs[1]]
        xc, yc = camera_out.rgb.pixs[0:2, idxs[2]]
        tri2 = np.float32([[[xa, ya], [xb, yb], [xc, yc]]])

        # Find bounding box. 
        r1 = cv2.boundingRect(tri1)
        r2 = cv2.boundingRect(tri2)

        # Offset points by left top corner of the 
        # respective rectangles

        tri1Cropped = []
        tri2Cropped = []

        for i in xrange(0, 3):
            tri1Cropped.append(((tri1[0][i][0] - r1[0]), (tri1[0][i][1] - r1[1])))
            tri2Cropped.append(((tri2[0][i][0] - r2[0]), (tri2[0][i][1] - r2[1])))

        # Apply warpImage to small rectangular patches
        img1Cropped = image_in[r1[1]:r1[1] + r1[3], r1[0]:r1[0] + r1[2]]

        # Given a pair of triangles, find the affine transform.
        warpMat = cv2.getAffineTransform(np.float32(tri1Cropped), np.float32(tri2Cropped))

        # Apply the Affine Transform just found to the src image
        image_outCropped = cv2.warpAffine(img1Cropped, warpMat, (r2[2], r2[3]), None, flags=cv2.INTER_LINEAR,
                                          borderMode=cv2.BORDER_REFLECT_101)

        # Get mask by filling triangle
        mask = np.zeros((r2[3], r2[2], 3), dtype=np.float32)
        cv2.fillConvexPoly(mask, np.int32(tri2Cropped), (1.0, 1.0, 1.0), 16, 0);

        # image222 = (image_outCropped * mask).astype(image_in.dtype)
        # cv2.namedWindow('image_out', cv2.WINDOW_NORMAL)
        # cv2.imshow('image_out', image222)

        # Apply mask to cropped region
        image_outCropped = image_outCropped * mask

        # Copy triangular region of the rectangular patch to the output image
        image_out[r2[1]:r2[1] + r2[3], r2[0]:r2[0] + r2[2]] = image_out[r2[1]:r2[1] + r2[3], r2[0]:r2[0] + r2[2]] * (
                (1.0, 1.0, 1.0) - mask)
        image_out[r2[1]:r2[1] + r2[3], r2[0]:r2[0] + r2[2]] = image_out[r2[1]:r2[1] + r2[3],
                                                              r2[0]:r2[0] + r2[2]] + image_outCropped

        return image_out
        # Draw triangles
        # x0, y0 = tri1[0][0]
        # x1, y1 = tri1[0][1]
        # cv2.line(image_in, (x0, y0), (x1, y1), (255,0,0), 2)

        # x0, y0 = tri1[0][1]
        # x1, y1 = tri1[0][2]
        # cv2.line(image_in, (x0, y0), (x1, y1), (255,0,0), 2)

        # x0, y0 = tri1[0][2]
        # x1, y1 = tri1[0][0]
        # cv2.line(image_in, (x0, y0), (x1, y1), (255,0,0), 2)

        cv2.namedWindow('image_in', cv2.WINDOW_NORMAL)
        cv2.imshow('image_in', image_in)
        cv2.namedWindow('image_out', cv2.WINDOW_NORMAL)
        cv2.imshow('image_out', image_out)

        # cv2.waitKey(0)

    def costRandomCamera(self, mesh, fh, use_z_buffering, use_z_filtering):
        """The camera with the smallest index has the smallest cost
        """
        # Collect necessary information top compute the cost
        cam_idxs = mesh.face_property('cam_idxs', fh)
        zbufs_valid = mesh.face_property('zbufs_valid', fh)
        zgaps = mesh.face_property('zgaps', fh)

        # compute partial costs
        if use_z_buffering:
            cost_zbufs_valid = np.array([1.0 if x == True else np.inf for x in zbufs_valid])
        else:
            cost_zbufs_valid = np.array([1.0] * len(cam_idxs))

        if use_z_filtering:
            cost_zgaps = np.array([1.0 if x <= self.p['z_inconsistency_threshold'] else np.inf for x in zgaps])
        else:
            cost_zgaps = np.array([1.0] * len(cam_idxs))

        cost_idxs = np.random.uniform(0.0, 1.0, len(cam_idxs))  # To avoid dividing by 0 (cam_idx == 0 )

        # Compute costs per cam_idx
        costs = cost_idxs * cost_zbufs_valid * cost_zgaps

        return zip(cam_idxs, costs)

    def costFirstCamera(self, mesh, fh, use_z_buffering, use_z_filtering):
        """The camera with the smallest index has the smallest cost
        """
        # Collect necessary information top compute the cost
        cam_idxs = mesh.face_property('cam_idxs', fh)
        zbufs_valid = mesh.face_property('zbufs_valid', fh)
        zgaps = mesh.face_property('zgaps', fh)

        # compute partial costs
        if use_z_buffering:
            print("sadasd")
            cost_zbufs_valid = np.array([1.0 if x == True else np.inf for x in zbufs_valid])
        else:
            cost_zbufs_valid = np.array([1.0] * len(cam_idxs))

        if use_z_filtering:
            cost_zgaps = np.array([1.0 if x <= self.p['z_inconsistency_threshold'] else np.inf for x in zgaps])
        else:
            cost_zgaps = np.array([1.0] * len(cam_idxs))

        cost_idxs = np.array(cam_idxs)  # To avoid dividing by 0 (cam_idx == 0 )

        # Compute costs per cam_idx
        costs = cost_idxs * cost_zbufs_valid * cost_zgaps


        return zip(cam_idxs, costs)

    def costLastCamera(self, mesh, fh, use_z_buffering, use_z_filtering):
        """The camera with the largest index has the smallest cost
        """
        # Collect necessary information top compute the cost
        cam_idxs = mesh.face_property('cam_idxs', fh)
        zbufs_valid = mesh.face_property('zbufs_valid', fh)
        zgaps = mesh.face_property('zgaps', fh)

        # compute partial costs
        if use_z_buffering:
            cost_zbufs_valid = np.array([1.0 if x == True else np.inf for x in zbufs_valid])
        else:
            cost_zbufs_valid = np.array([1.0] * len(cam_idxs))

        if use_z_filtering:
            cost_zgaps = np.array([1.0 if x <= self.p['z_inconsistency_threshold'] else np.inf for x in zgaps])
        else:
            cost_zgaps = np.array([1.0] * len(cam_idxs))

        cost_idxs = 1.0 / (np.array(cam_idxs) + 1)  # To avoid dividing by 0 (cam_idx == 0 )

        # Compute costs per cam_idx
        costs = cost_idxs * cost_zbufs_valid * cost_zgaps

        return zip(cam_idxs, costs)

    def costFunction1(self, mesh, fh, use_z_buffering, use_z_filtering):
        """ ...
        """
        # Collect necessary information top compute the cost
        cam_idxs = mesh.face_property('cam_idxs', fh)
        # areas = mesh.face_property('cam_areas', fh)
        zbufs_valid = mesh.face_property('zbufs_valid', fh)
        zgaps = mesh.face_property('zgaps', fh)
        parent_sel_cam_idx = mesh.face_property('parent_sel_cam_idx', fh)

        # Partial costs
        if use_z_buffering:
            cost_zbufs_valid = np.array([1.0 if x == True else np.inf for x in zbufs_valid])
        else:
            cost_zbufs_valid = np.array([1.0] * len(cam_idxs))

        if use_z_filtering:
            cost_zgaps = np.array([1.0 if x <= self.p['z_inconsistency_threshold'] else np.inf for x in zgaps])
        else:
            cost_zgaps = np.array([1.0] * len(cam_idxs))

        # cost_areas = 1.0 / np.array(areas)

        cost_parent_cam_idx = np.array([0.01 if parent_sel_cam_idx == x else 1.0 for x in cam_idxs])

        # Final costs per cam_idx
        costs = cost_parent_cam_idx * cost_zbufs_valid * cost_zgaps

        # print('parent_sel_cam_idx ' + str(parent_sel_cam_idx))
        # print('cam_idxs ' + str(cam_idxs))
        # print('zbufs_valid ' + str(zbufs_valid))
        # print('cost_zbufs_valid ' + str(cost_zbufs_valid))
        # print('zgaps ' + str(zgaps))
        # print('cost_zgaps ' + str(cost_zgaps))
        # print('cost_parent_cam_idx ' + str(cost_parent_cam_idx))
        # print('costs =  ' + str(costs))

        return zip(cam_idxs, costs)

    def costProjectedTriangleArea(self, mesh, fh, use_z_buffering, use_z_filtering):
        """The smaller the triangle's area the largest the cost
        """
        # Collect necessary information top compute the cost
        cam_idxs = mesh.face_property('cam_idxs', fh)
        areas = mesh.face_property('cam_areas', fh)
        zbufs_valid = mesh.face_property('zbufs_valid', fh)
        zgaps = mesh.face_property('zgaps', fh)

        # compute partial costs
        if use_z_buffering:
            cost_zbufs_valid = np.array([1.0 if x == True else np.inf for x in zbufs_valid])
        else:
            cost_zbufs_valid = np.array([1.0] * len(cam_idxs))

        if use_z_filtering:
            cost_zgaps = np.array([1.0 if x <= self.p['z_inconsistency_threshold'] else np.inf for x in zgaps])
        else:
            cost_zgaps = np.array([1.0] * len(cam_idxs))

        cost_areas = 1.0 / np.array(areas)

        # Compute costs per cam_idx
        costs = cost_areas * cost_zbufs_valid * cost_zgaps

        # print(cam_idxs)
        # print(areas)
        # print(zbufs_valid)
        # print(cost_zbufs_valid)
        # print(zgaps)
        # print(cost_zgaps)
        # print(costs)

        return zip(cam_idxs, costs)

    def setselCameraIdxFromCost(self, mesh, cameras, fh, cost_function, use_z_buffering=True, use_z_filtering=True):

        # Call the cost function (costs should be a list of (cam_idx, cost) tuples)
        costs = cost_function(mesh, fh, use_z_buffering, use_z_filtering)

        # Sort and filter costs and select cam_idx with smallest cost
        sorted_costs = sorted(costs, key=lambda x: x[1],
                              reverse=False)  # sort by the cost, second position in the tuple
        filtered_costs = [x for x in sorted_costs if not x[1] == np.inf]

        # print(costs)
        # print(sorted_costs)
        # print(filtered_costs)

        for cost in filtered_costs:
            cam_idx = cost[0]
            self.mesh.set_face_property('sel_cam_idx', fh, cam_idx)
            # print('sel_cam_idx is ' + str(cam_idx))
            break

    def setFacePropertyValuePerIndex(self, mesh, fh, prop_name, prop_idx, value):
        """Used to set a single element of the list that is a face property
        """
        # print('prop_name = ' + str(prop_name))
        # print('prop_idx = ' + str(prop_idx))
        values = mesh.face_property(prop_name, fh)  # read the list
        # print(values)
        values[prop_idx] = value  # change the list element at prop_index
        mesh.set_face_property(prop_name, fh, values)  # set the new list as the face property

    def shapelyPolygonFromFaceProjection(self, mesh, camera, fh):
        idxs = []
        for vh in self.mesh.fv(fh):
            idxs.append(vh.idx())

        xa, ya = camera.rgb.pixs[0:2, idxs[0]]
        xb, yb = camera.rgb.pixs[0:2, idxs[1]]
        xc, yc = camera.rgb.pixs[0:2, idxs[2]]

        return Polygon([(xa, ya), (xb, yb), (xc, yc)])

    def drawFaceProjection(self, mesh, fh, camera, image, color=(255, 0, 0), linewidth=2, text=None):
        idxs = []
        for vh in mesh.fv(fh):  # handle to the face's vertices
            idxs.append(vh.idx())

        x0, y0 = camera.rgb.pixs[0:2, idxs[0]]
        x1, y1 = camera.rgb.pixs[0:2, idxs[1]]
        x2, y2 = camera.rgb.pixs[0:2, idxs[2]]

        cv2.line(image, (x0, y0), (x1, y1), color, linewidth)
        cv2.line(image, (x1, y1), (x2, y2), color, linewidth)
        cv2.line(image, (x2, y2), (x0, y0), color, linewidth)

        if text is not None:
            cx = int((x0 + x1 + x2) / 3.0)
            cy = int((y0 + y1 + y2) / 3.0)
            cv2.putText(image, text, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.4, thickness=1, color=color)

    def writeObjFile(self):

        # Save textures to results folder
        print('Saving camera images to ' + self.p['path_results'])
        for camera in self.cameras:
            image_file = self.p['path_results'] + os.path.basename(camera.rgb.filename) 
            print('Saving image ' + image_file)
            cv2.imwrite(image_file, camera.rgb.image_smoothed)


        # Write the mtl file
        print('Writting mtl file to ' + self.p['output_mtl_filename'])
        with open(self.p['output_mtl_filename'], 'w') as f:
            f.write('#\n# Wavefront material file\n#Converted by Miguel Riem Oliveira\n#\n')

            for camera in self.cameras:
                print('material for camera ' + os.path.basename(camera.rgb.filename))
                f.write('newmtl material_' + camera.name + '\n' + \
                        'Ka 0.200000 0.200000 0.200000\n' + \
                        'Kd 0.639216 0.639216 0.639216\n' + \
                        'Ks 1.000000 1.000000 1.000000\n' + \
                        'Tr 1.000000\n' + \
                        'illum 2\n' + \
                        'Ns 0.000000\n' + \
                        'map_Kd ' +  os.path.basename(camera.rgb.filename) + '\n')

        # write the obj file
        print('Writting obj file to ' + self.p['output_mesh_filename'])
        with open(self.p['output_mesh_filename'], 'w') as f:
            f.write("# OBJ file\n")
            f.write("# Miguel Riem Oliveira\n")
            f.write("mtllib %s\n" % os.path.basename(self.p['output_mtl_filename']))
            # f.write("mtllib %s\n" % 'tmp.mtl')

            # Vertices
            f.write('\n#List of geometric vertices, with (x,y,z[,w]) coordinates, w is optional and defaults to 1.0.\n')
            for vh in self.mesh.vertices():
                x, y, z = self.mesh.point(vh)
                # f.write("v %f %f %f\n" % (x,y,z))
                f.write("v %.4f %.4f %.4f\n" % (x, y, z))

            # Normals
            normals = self.mesh.vertex_normals()  # normals has shape (n_vertexes, 3)
            f.write('\n#List of vertex normals in (x,y,z) form; normals might not be unit vectors.\n')
            for vh in self.mesh.vertices():
                nx, ny, nz = normals[vh.idx(), :]
                # f.write("vn %f %f %f\n" % (nx,ny,nz))
                f.write("vn %.4f %.4f %.4f\n" % (nx, ny, nz))

            # Texture coordinates
            f.write(
                '\n#List of texture coordinates, in (u, v [,w]) coordinates, these will vary between 0 and 1, w is optional and defaults to 0.\n')
            for texcoord in tqdm(self.texcoords):
                f.write("vt %.4f %.4f\n" % texcoord)

            last_used_camera = ''
            # Face elements
            f.write('\n#Polygonal face elements\n')
            for fh in tqdm(self.mesh.faces()):
                sel_cam_idx = self.mesh.face_property('sel_cam_idx', fh)
                if sel_cam_idx is None:
                    continue

                camera = self.cameras[sel_cam_idx]
                if not last_used_camera == camera.name:
                    f.write("usemtl material_%s\n" % camera.name)
                    last_used_camera = camera.name

                f.write('f')
                uv_idxs = self.mesh.face_property('uv_idxs', fh)
                for i, vh in enumerate(self.mesh.fv(fh)):
                    f.write(' %d/%d/%d ' % (vh.idx() + 1, uv_idxs[i] + 1,
                                            vh.idx() + 1))  # add +1 because in the obj format they start counting at 1 and not 0

                f.write('\n')

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

    def readCameras(self, rgb_camera_info, depth_camera_info=None, image_skip=1, use_color_corrected_images=False):
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

            if use_color_corrected_images == True:
                image_file_cc = os.path.dirname(image_file) + '/color_correction/' + os.path.basename(image_file)
                print(image_file)
                print(image_file_cc)
            else:
                image_file_cc = image_file

            self.cameras.append(RGBDCameraT(camera_name, 'map',
                                            cv2.imread(image_file_cc, 1), camera_matrix, 'cam' + image_file[:-4], stamp,
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

    def drawMesh(self, mesh, cam_idx, image, color_zbuf_invalid=(20, 20, 20), color_zgap_invalid=(120, 120, 120),
                 color_available=(210, 210, 210), color_sel=(0, 210, 0), color_all=None):

        def getProp(mesh, fh, prop_name, cam_idx):
            prop_idx = mesh.face_property('cam_idxs', fh).index(cam_idx)
            return mesh.face_property(prop_name, fh)[prop_idx]

        # Draw all faces
        if not color_all is None:
            self.drawFacesOnImage(self.mesh, cam_idx, image, lambda mesh, fh, cam_idx: True, color=color_all)
        else:
            # Draw zbuffering invalid faces
            if not color_zbuf_invalid is None:
                self.drawFacesOnImage(self.mesh, cam_idx, image,
                                      lambda mesh, fh, cam_idx: getProp(mesh, fh, 'zbufs_valid', cam_idx) == False,
                                      color=color_zbuf_invalid)

            # Draw zbuffering valid faces and zgap invalid face
            if not color_zgap_invalid is None:
                self.drawFacesOnImage(self.mesh, cam_idx, image,
                                      lambda mesh, fh, cam_idx: getProp(mesh, fh, 'zbufs_valid',
                                                                        cam_idx) == True and getProp(mesh, fh, 'zgaps',
                                                                                                     cam_idx) > self.p[
                                                                    'z_inconsistency_threshold'],
                                      color=color_zgap_invalid)

            # Draw available faces (zbuf valid, zgap valid, not sel)
            if not color_available is None:
                self.drawFacesOnImage(self.mesh, cam_idx, image,
                                      lambda mesh, fh, cam_idx: getProp(mesh, fh, 'zbufs_valid',
                                                                        cam_idx) == True and getProp(mesh, fh, 'zgaps',
                                                                                                     cam_idx) <= self.p[
                                                                    'z_inconsistency_threshold'] and not cam_idx == mesh.face_property(
                                          'sel_cam_idx', fh), color=color_available)

            # Draw sel faces
            if not color_sel is None:
                self.drawFacesOnImage(self.mesh, cam_idx, image,
                                      lambda mesh, fh, cam_idx: getProp(mesh, fh, 'zbufs_valid',
                                                                        cam_idx) == True and getProp(mesh, fh, 'zgaps',
                                                                                                     cam_idx) <= self.p[
                                                                    'z_inconsistency_threshold'] and cam_idx == mesh.face_property(
                                          'sel_cam_idx', fh), color=color_sel)

    def drawFacesOnImage(self, mesh, cam_idx, image, func, color):
        for fh in mesh.faces():
            cam_idxs = mesh.face_property('cam_idxs', fh)
            if cam_idx in cam_idxs:
                # print cam_idxs
                if func(mesh, fh, cam_idx):
                    self.drawFaceProjection(mesh, fh, self.cameras[cam_idx], image, color=color)

    def drawFacesOnImages(self, wait_key=0, save_images=True, draw_zbuffering=True, draw_zfilt=True, draw_valid=True, draw_selected=True):
        # ---------------------------------------
        # --- Draw face triangles on images
        # ---------------------------------------
        for cam_idx, camera in enumerate(self.cameras):
            print('Drawing triangles of camera ' + camera.name)
            image = camera.rgb.image

            # iterate over all faces and draw all
            for fh in self.mesh.faces():
                cam_idxs = self.mesh.face_property('cam_idxs', fh)
                if cam_idx in cam_idxs:
                    self.drawFaceProjection(self.mesh, fh, camera, image, color=(210, 0, 0))


            if draw_zbuffering:
                # iterate over all faces and draw z buffering invalid faces
                for fh in self.mesh.faces():
                    cam_idxs = self.mesh.face_property('cam_idxs', fh)
                    if cam_idx in cam_idxs:
                        prop_idx = cam_idxs.index(cam_idx)
                        if self.mesh.face_property('zbufs_valid', fh)[prop_idx] == False:
                            self.drawFaceProjection(self.mesh, fh, camera, image, color=(20, 20, 20))


            if draw_zfilt:
                # iterate over all faces and draw z buffering valid  and z-filt invalid faces
                for fh in self.mesh.faces():
                    cam_idxs = self.mesh.face_property('cam_idxs', fh)
                    if cam_idx in cam_idxs:
                        prop_idx = cam_idxs.index(cam_idx)
                        if self.mesh.face_property('zbufs_valid', fh)[prop_idx] == True and \
                                self.mesh.face_property('zgaps', fh)[prop_idx] > self.p['z_inconsistency_threshold']:
                            self.drawFaceProjection(self.mesh, fh, camera, image, color=(120, 120, 120))


            if draw_valid:
                # iterate over all faces and draw valid faces
                for fh in self.mesh.faces():
                    cam_idxs = self.mesh.face_property('cam_idxs', fh)
                    if cam_idx in cam_idxs:
                        prop_idx = cam_idxs.index(cam_idx)

                        if self.mesh.face_property('zbufs_valid', fh)[prop_idx] == True and \
                                self.mesh.face_property('zgaps', fh)[prop_idx] <= self.p[
                            'z_inconsistency_threshold'] and not cam_idx == self.mesh.face_property('sel_cam_idx', fh):
                            self.drawFaceProjection(self.mesh, fh, camera, image, color=(210, 210, 210))

            if draw_selected:
                # iterate over all faces and draw sel faces
                for fh in self.mesh.faces():
                    cam_idxs = self.mesh.face_property('cam_idxs', fh)
                    if cam_idx in cam_idxs:
                        prop_idx = cam_idxs.index(cam_idx)

                        if cam_idx == self.mesh.face_property('sel_cam_idx', fh):
                            self.drawFaceProjection(self.mesh, fh, camera, image, color=(0, 220, 0), linewidth=3)

            image_file = self.p['path_results'] + 'faces_' + os.path.basename(camera.rgb.filename) 
            print('Saving image ' + image_file)
            cv2.imwrite(image_file, image)
            camera.rgb.show(name=camera.name, wait_for_key=False)

        cv2.waitKey(wait_key)
        # print(wait_key)

    def getFaceArea(self, x1, y1, x2, y2, x3, y3):
        """Computes the are of a triangular face. 
           Equation from https://www.mathopenref.com/coordtrianglearea.html
        """
        return abs((x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2)) / 2)

    def printFaceInfo(self, fh):
        print('Face ' + str(fh.idx()) + ':'
              + '\n   cam_idxs = ' + str(self.mesh.face_property('cam_idxs', fh))
              + '\n   cam_areas = ' + str(self.mesh.face_property('cam_areas', fh))
              + '\n   center = ' + str([round(x, 2) for x in self.mesh.face_property('center', fh)])
              + '\n   ray z center = ' + str([round(x, 2) for x in self.mesh.face_property('center_zray', fh)]) + ', '
              + '\n   ray z a = ' + str([round(x, 2) for x in self.mesh.face_property('a_zray', fh)]) + ', '
              + '\n   ray z b = ' + str([round(x, 2) for x in self.mesh.face_property('b_zray', fh)]) + ', '
              + '\n   ray z c = ' + str([round(x, 2) for x in self.mesh.face_property('c_zray', fh)]) + ', ')

    def getEuclideanDistance(self, x0, y0, z0, x1, y1, z1):
        return sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2 + (z1 - z0) ** 2)

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

    def printMeshInfo(self, mesh):
        print('Mesh has ' + str(len(mesh.faces())) + ' faces and ' + str(len(mesh.vertices())) + ' vertices')
        print('has_vertex_texcoords2D() = ' + str(mesh.has_vertex_texcoords2D()))
        print('has_face_texture_index() = ' + str(mesh.has_face_texture_index()))
        print('has_face_vertex_normals() = ' + str(mesh.has_vertex_normals()))

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


def generate_point_cloud2(depth, cx, cy, fx, fy):
    """Transform a depth image into a point cloud with one point for each
    pixel in the image, using the camera transform for a camera
    centred at cx, cy with field of view fx, fy.

    depth is a 2-D ndarray with shape (rows, cols) containing
    depths from 1 to 254 inclusive. The result is a 3-D array with
    shape (rows, cols, 3). Pixels with invalid depth in the input have
    NaN for the z-coordinate in the result.

    """
    rows, cols = depth.shape
    c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
    #valid = (depth > 0) & (depth < 255)
    valid = (depth > 0) & np.isnan(depth)==False
    z = np.where(valid, depth, np.nan)
    x = np.where(valid, z * (c - cx) / fx, 0)
    y = np.where(valid, z * (r - cy) / fy, 0)
    #return np.dstack((x, y, z))
    return (x, y, z)

