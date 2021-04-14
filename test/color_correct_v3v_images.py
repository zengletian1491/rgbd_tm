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
    
    print("Executing command: " + cmd)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for line in p.stdout.readlines():
        print line,
        retval = p.wait()

    print "Preprocessing completed."



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
        fold = apath + "/color_balanced_images/"
        d = os.path.dirname(fold)
        if not os.path.exists(d):
            print "Creating folder " + fold
            os.makedirs(d)

        
        #Create a list of images filenames
        image_files = []
        image_lines = ps.i            # get a list of all i-lines
        for line in image_lines:

            #copy the original images to the color_balanced_images folder
            cmd = "cp " + line.n.value + " " + fold + os.path.basename(line.n.value)
            
            #print("Executing command: " + cmd)
            p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            for line in p.stdout.readlines():
                print line,
                retval = p.wait()

            image_files.append(fold + os.path.basename(line.n.value))


        #print(image_files)

        cps = ps.c            # get a list of c-lines

        src_idx = cps[0].n.value
        tgt_idx = cps[0].N.value
        matches = []

        print("src_idx=" + str(src_idx) + " tgt_idx=" + str(tgt_idx))
        count = 0
        for cp in cps:
            
            #check if this new line is a new image pair 
            if (not src_idx == cp.n.value) or  (not tgt_idx == cp.N.value):
                print("cp[" + str(count) + "] new image pair")

                #create temporaty txt file from matches
                #print(matches)
                src = image_files[src_idx]
                tgt = image_files[tgt_idx]
                print("There are " + str(len(matches)) + " matches between images\n" + src + "\n" + tgt)

                cp_file = '/tmp/v3v_control_points.txt'
                with open(cp_file, 'w') as f:
                    for m in matches:
                        f.write('%s\n' % " ".join(map(str, m)))

                #call the matlab script
                callCorrectImagePair(src, tgt, cp_file, tgt)



                #clear the matches
                matches = []

                #Set the new src and tgt idxs
                src_idx = cp.n.value
                tgt_idx = cp.N.value
                print("new src_idx=" + str(src_idx) + " tgt_idx=" + str(tgt_idx))


                #exit(0)

            #Append a new match to the list
            #print("cp[" + str(count) + "] x" + str(cp.x.value))
            matches.append([cp.x.value, cp.y.value, cp.X.value, cp.Y.value])









            count = count + 1

            
    sys.exit()
    
