#!/usr/bin/env python
"""

This is a long, multiline description
"""

#########################
##    IMPORT MODULES   ##
#########################
import sys
import rospy
import roslib
import math
import tf
import subprocess
import os
import xml.etree.ElementTree
import cv2
import numpy
import matplotlib
from matplotlib import pyplot as plt

try:
    import matplotlib.pyplot as plt
except:
    raise

import networkx as nx
import pydot


import ptucam.v3v_interface as v3v_interface
import ptucam.parse_pto as parse_pto

#########################
##      HEADER         ##
#########################
__author__ = "Miguel Riem de Oliveira"
__date__ = "December 2015"
__copyright__ = "Copyright 2015, V3V"
__credits__ = ["Miguel Riem de Oliveira"]
__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Miguel Oliveira"
__email__ = "m.riem.oliveira@gmail.com"
__status__ = "Development"


#########################
## FUNCTION DEFINITION ##
#########################

def callCorrectImagePair(src, tgt, cp, corr):
    """ Calls rosrun correct_image_pair compiled matlab function

    ./run_color_correct_image_pair.sh /usr/local/MATLAB/MATLAB_Compiler_Runtime/v83/ ../images/src.png ../images/tgt.png ../images/src_tgt_control_points.txt ../images/corr.png
    """

    cmd = "rosrun v3v_color_correction run_color_correct_image_pair.sh /usr/local/MATLAB/MATLAB_Compiler_Runtime/v83/ " + src + " " + tgt + " " + cp + " " + corr
    

    src_img = cv2.imread(src, cv2.IMREAD_UNCHANGED)
    tgt_img = cv2.imread(tgt, cv2.IMREAD_UNCHANGED)


    # Two subplots, the axes array is 1-d
    f, axarr = plt.subplots(2, sharex=True)
    axarr[0].imshow(src_img[:,:,::-1], cmap = 'gray', interpolation = 'bicubic')
    axarr[0].set_xticks([], minor=False)
    axarr[0].set_yticks([], minor=False)

    plt.show()



    print("Executing command: " + cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for line in p.stdout.readlines():
        print line,
        retval = p.wait()

    print "Preprocessing completed."

def findControlPoints(cps, src_idx, tgt_idx):
    matches=[]
    for cp in cps: #iterate all control points
        
        #check if this new line is the desired image pair 
        if (src_idx == cp.n.value) and  (tgt_idx == cp.N.value):
            matches.append([cp.x.value, cp.y.value, cp.X.value, cp.Y.value])

        #check if this new line is the desired image pair (inverted)
        if (src_idx == cp.N.value) and  (tgt_idx == cp.n.value):
            matches.append([cp.X.value, cp.Y.value, cp.x.value, cp.y.value])
    return matches

##########
## MAIN ##
##########

if __name__ == "__main__":

    #--------------------#
    ### Initialization ###
    #--------------------#

    # Use path given as command line arguments, else use current path
    current_path = '.'

    if len(sys.argv)>1:
        print(sys.argv[1])
        path = sys.argv[1]
    else:
        path = current_path

    #-------------------------------------#
    ### Create v3v project description  ###
    #-------------------------------------#
    acquisitions = v3v_interface.readV3VProjectFromFolderV2(path)
    
    #------------------------------------------------#
    ### Publish the transforms of the acquisitions ###
    #------------------------------------------------#
    print "Getting poses from hugin project."

    for a in acquisitions:

        apath = a[2]

        #Check if hugins panorama directory exists
        fold = apath + "/panorama/"
        d = os.path.dirname(fold)
        if not os.path.exists(d):
            print "Error: " + fold + " does not exist. You must create a panorama folder and initiate a hugins project (with all the images read) inside called project.pto "
            continue

        #Check if hugins panorama project_withposes.pto exists
        project_with_poses_filename = apath + "/panorama/project_withposes.pto"
        if not os.path.isfile(project_with_poses_filename):
            print "File " + project_with_poses_filename + " does not exist. Nothing to be done."
            continue

        #Check if hugins views directory exists (create if it does not)
        opp = apath + "/views/hugin/"
        d = os.path.dirname(opp)
        if not os.path.exists(d):
            print "Creating folder " + opp 
            os.makedirs(d)

        #Scan pto project
        print "Scanning file " + project_with_poses_filename
        ps = parse_pto.pto_scan(project_with_poses_filename)
        ps.make_member_access()

        #Create color_balanced_images folder if one does not exist
        fold_images = apath + "/color_balanced_images/"
        fold = fold_images
        d = os.path.dirname(fold)
        if not os.path.exists(d):
            print "Creating folder " + fold
            os.makedirs(d)

        #delete all images from the color_balanced_images folder
        #cmd = "rm " + fold + "*.png"
        #print("Executing command: " + cmd)
        #p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        #for line in p.stdout.readlines():
            #print line,
            #retval = p.wait()

        #Create color_balanced_images/control_points folder if one does not exist
        fold = apath + "/color_balanced_images/control_points/"
        d = os.path.dirname(fold)
        if not os.path.exists(d):
            print "Creating folder " + fold
            os.makedirs(d)

        #delete all control_points from the color_balanced_images/control_points folder
        cmd = "rm " + fold + "*.txt"
        print("Executing command: " + cmd)
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        for line in p.stdout.readlines():
            print line,
            retval = p.wait()


        #Create a list of images filenames
        fold = fold_images
        image_files = []
        image_lines = ps.i            # get a list of all i-lines
        for line in image_lines:
            image_files.append(fold + os.path.basename(line.n.value))


        #print(image_files)

        cps = ps.c            # get a list of c-lines
        num_control_points = len(cps)
        print("There are a total of " + str(num_control_points) + " matches")

        #Create the graph


        # labels
        num_nodes = len(image_files)
        labels={}
        count = 0

        G = nx.Graph()
        for i in image_files:

            filename, _ = os.path.splitext(os.path.basename(i))
            labels[count] = filename            
            G.add_node(filename)
            count = count + 1

        #add edges

        src_idx = cps[0].n.value
        tgt_idx = cps[0].N.value
        matches = []

        count = 0
        for cp in cps: #TODO Count also edges which are the other way around
            
            #check if this new line is a new image pair 
            if (not src_idx == cp.n.value) or  (not tgt_idx == cp.N.value):

                #create temporaty txt file from matches
                src_path = image_files[src_idx]
                tgt_path = image_files[tgt_idx]
                num_matches = len(matches)

                w = 1 - (float(num_matches) / float(num_control_points)) 
                print("There are " + str(len(matches)) + " matches (w=" + str(w) + ") between images\n" + src_path + "\n" + tgt_path)


                if (num_matches > 0):
                    src_node, _ = os.path.splitext(os.path.basename(src_path))
                    tgt_node, _ = os.path.splitext(os.path.basename(tgt_path))

                    G.add_edge(src_node, tgt_node, weight=w)
                
                #clear the matches
                matches = []

                #Set the new src and tgt idxs
                src_idx = cp.n.value
                tgt_idx = cp.N.value

            matches.append([cp.x.value, cp.y.value, cp.X.value, cp.Y.value])

        
        # 
        #pos = nx.random_layout(G)
        #colors=range(num_nodes)

        #edges,weights = zip(*nx.get_edge_attributes(G,'weight').items())

        #nx.draw(G,pos,node_color='#A0CBE2',edgelist=edges, edge_color=weights, width=1, edge_cmap=plt.cm.Greys_r, with_labels=True, alpha=1, node_size=1500, font_color='r')
        #plt.show() # display
                    




        seminal_image = 'A00_img020'
        for image_file in image_files:
            if image_files == seminal_image:
                continue #nothing to do

            print("\n")

            #Define a path along the graph and color correct image along the path
            start_img = seminal_image;
            end_img, _ = os.path.splitext(os.path.basename(image_file))
            try:
                path_start_to_end = nx.shortest_path(G, source=start_img, target=end_img, weight='weight')
                print("The shortest path from " + start_img + " tp " + end_img + " is: " + str(path_start_to_end))
            except nx.NetworkXNoPath:
                print("There is not path from " + start_img + " tp " + end_img + ". Image " + end_img + " will not be color corrected.")

                #tgt_idx = int(end_img[-3:])
                #tgt_path = image_files[tgt_idx]
                #if not os.path.isfile(tgt_path):

                    #orig_file = apath + "/images/" + os.path.basename(tgt_path)
                    
                    #cmd = "cp " + orig_file + " " + tgt_path
                    #print("Executing command: " + cmd)
                    #p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    #for line in p.stdout.readlines():
                        #print line,
                        #retval = p.wait()

                continue

            for i in range(len(path_start_to_end)-1): #iterate all local steps along the path
                src_node = path_start_to_end[i] #lstart_img stands for local start image
                tgt_node = path_start_to_end[i+1]
                print("Going from " + src_node + " to " + tgt_node)

                src_idx = int(src_node[-3:])
                tgt_idx = int(tgt_node[-3:])
                print(src_idx)
                print(tgt_idx)

                src_path = image_files[src_idx]
                tgt_path = image_files[tgt_idx]
                print(src_path)
                print(tgt_path)

                #Check if tgt image already exists in /color_balanced_images folder. 
                #if the image exists we assume that it has already been corrected and skip this local step correction
                if os.path.isfile(tgt_path):
                    print "File " + tgt_path + " already exists in /color_balanced_images. Skiping this color correction step."
                    continue

                #If the tgt image does not exist we need to conduct color correction. First, we get all the matches between src and tgt and write those to a file
                matches = findControlPoints(cps, src_idx, tgt_idx)
                print("Found " + str(len(matches)) + " matches")
                cp_file = apath + "/color_balanced_images/control_points/" + str(src_idx) + "_" + str(tgt_idx) + ".txt"
                with open(cp_file, 'w') as f:
                    for m in matches:
                        f.write('%s\n' % " ".join(map(str, m)))

                #copy the original images to the color_balanced_images folder 
                #if images not there yet
                if not os.path.isfile(src_path):

                    orig_file = apath + "/images/" + os.path.basename(src_path)
                    
                    cmd = "cp " + orig_file + " " + src_path
                    print("Executing command: " + cmd)
                    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    for line in p.stdout.readlines():
                        print line,
                        retval = p.wait()

                if not os.path.isfile(tgt_path):

                    orig_file = apath + "/images/" + os.path.basename(tgt_path)
                    
                    cmd = "cp " + orig_file + " " + tgt_path
                    print("Executing command: " + cmd)
                    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
                    for line in p.stdout.readlines():
                        print line,
                        retval = p.wait()


                #call the matlab script
                callCorrectImagePair(src_path, tgt_path, cp_file, tgt_path)




        #plot the graph
        #pos=nx.spring_layout(G)
        #pos=nx.circular_layout(G)
        pos = nx.random_layout(G)
        colors=range(num_nodes)

        edges,weights = zip(*nx.get_edge_attributes(G,'weight').items())

        nx.draw(G,pos,node_color='#A0CBE2',edgelist=edges, edge_color=weights, width=1, edge_cmap=plt.cm.Greys_r, with_labels=True, alpha=1, node_size=1500, font_color='r')
        #plt.show() # display
        break #for now we only do acquisition 0
                    
    sys.exit()
    
