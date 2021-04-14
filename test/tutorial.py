#!/usr/bin/env python
import openmesh as om
import numpy as np


mesh = om.TriMesh()

# add a a couple of vertices to the mesh
vh0 = mesh.add_vertex([0, 1, 0])
vh1 = mesh.add_vertex([1, 0, 0])
vh2 = mesh.add_vertex([2, 1, 0])
vh3 = mesh.add_vertex([0,-1, 0])
vh4 = mesh.add_vertex([2,-1, 0])

# add a couple of faces to the mesh
fh0 = mesh.add_face(vh0, vh1, vh2)
fh1 = mesh.add_face(vh1, vh3, vh4)
fh2 = mesh.add_face(vh0, vh3, vh1)

# add another face to the mesh, this time using a list
vh_list = [vh2, vh1, vh4]
fh3 = mesh.add_face(vh_list)

#  0 ==== 2
#  |\  0 /|
#  | \  / |
#  |2  1 3|
#  | /  \ |
#  |/  1 \|
#  3 ==== 4

# get the point with vertex handle vh0
point = mesh.point(vh0)

# get all points of the mesh
point_array = mesh.points()

# translate the mesh along the x-axis
point_array += np.array([1, 0, 0])

# write and read meshes
om.write_mesh('test.off', mesh)
print(mesh)

mesh_2 = om.read_trimesh('../models/office/model/20180221_164439.obj')
#mesh_2 = om.read_trimesh('../models/office/model/20180221_164439.ply')

print(type(mesh))

# iterate over all faces
#for fh in mesh_2.faces():
    #print fh.idx()

#print(mesh_2.request_face_normals())

mesh_2.request_vertex_normals()
print(mesh_2.has_vertex_normals())
mesh_2.update_normals()
mesh_2.request_vertex_texcoords2D()
print(mesh_2.has_vertex_texcoords2D())
    
# iterate over all adjacent faces
# iterate over all faces
for fh in mesh_2.faces():
    #print fh.idx()
    # iterate over the face's vertices
    i = 0
    for vh in mesh_2.fv(fh):
        #print vh.idx()
        #mesh_2.set_texcoord2D(vh, np.random.random_sample((1, 2))[0])
        mesh_2.set_texcoord2D(vh, np.array([0.2*i, 0.2*i]))
        
        #print( )

#write_mesh(...)
#write_mesh(*args, **kwargs)
#Overloaded function.

#1. write_mesh(filename: unicode, mesh: openmesh.TriMesh, binary: bool=False, msb: bool=False, lsb: bool=False, swap: bool=False, vertex_normal: bool=False, vertex_color: bool=False, vertex_tex_coord: bool=False, halfedge_tex_coord: bool=False, edge_color: bool=False, face_normal: bool=False, face_color: bool=False, color_alpha: bool=False, color_float: bool=False) -> None
#2. write_mesh(filename: unicode, mesh: openmesh.PolyMesh, binary: bool=False, msb: bool=False, lsb: bool=False, swap: bool=False, vertex_normal: bool=False, vertex_color: bool=False, vertex_tex_coord: bool=False, halfedge_tex_coord: bool=False, edge_color: bool=False, face_normal: bool=False, face_color: bool=False, color_alpha: bool=False, color_float: bool=False) -> None

om.write_mesh('../models/office/model/20180221_164439aaa.obj', mesh_2, vertex_normal=True, vertex_tex_coord=True,binary=True)

#import pywavefront
#meshes = pywavefront.Wavefront('../models/office/model/20180221_164439.obj')
#meshes.draw()
