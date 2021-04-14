# RGBD-TM

Work submitted to Sensors (ISSN 1424-8220), entitled:

**Robust Texture Mapping using RGB-D Cameras**

Miguel Oliveira, Gi Hyun Lim, Tiago Madeira, Paulo Dias and Vítor Santos

Currently under review. Submitted February 2021.

# Table of Contents

- [RGBD-TM](#rgbd-tm)
- [Table of Contents](#table-of-contents)
- [Introduction](#introduction)
- [Usage](#usage)
- [Creating a camera_info yaml](#creating-a-camera-info-yaml)
- [Color correct](#color-correct)

<small><i><a href='http://ecotrust-canada.github.io/markdown-toc/'>Table of contents generated with markdown-toc</a></i></small>

# Introduction



# Usage

Run with the following command from the root of the repository:

    clear && ./test/gui.py -p datasets/red_book2/dataset/ -m datasets/red_book2/dataset/1528188687058_simplified.obj -i calibrations/camera.yaml -z 0.06 -zbuf -zfilt -sm 0 -si 5 -v
    
Here is the full set of input parameters:

```bash
usage: gui.py [-h] -p PATH_TO_IMAGES -m MESH_FILENAME -i PATH_TO_INTRINSICS
              [-z Z_INCONSISTENCY_THRESHOLD] [-zfilt] [-vzb] [-zbuf]
              [-ext IMAGE_EXTENSION] [-vri] [-vsa] [-sm SELECTION_METHOD]
              [-bfs] [-ucci] [-si SKIP_IMAGES]

optional arguments:
  -h, --help            show this help message and exit
  -p PATH_TO_IMAGES, --path_to_images PATH_TO_IMAGES
                        path to the folder that contains the images
  -m MESH_FILENAME, --mesh_filename MESH_FILENAME
                        full filename to input obj file, i.e. the 3D model
  -i PATH_TO_INTRINSICS, --path_to_intrinsics PATH_TO_INTRINSICS
                        path to intrinsics yaml file
  -z Z_INCONSISTENCY_THRESHOLD, --z_inconsistency_threshold Z_INCONSISTENCY_THRESHOLD
                        threshold for max z inconsistency value
  -zfilt, --use_z_filtering
                        use z-filtering technique
  -vzb, --view_z_buffering
                        visualize z-buffering
  -zbuf, --use_z_buffering
                        use z-buffering
  -ext IMAGE_EXTENSION, --image_extension IMAGE_EXTENSION
                        extension of the image files, e.g., jpg or png
  -vri, --view_range_image
                        visualize sparse and dense range images
  -vsa, --view_selection_animation
                        visualize camera selection animation
  -sm SELECTION_METHOD, --selection_method SELECTION_METHOD
                        method for selecting the cameras for each triangle
  -bfs, --border_face_smoothing
                        Use border face smoothing
  -ucci, --use_color_corrected_images
                        Use previously color corrected images
  -si SKIP_IMAGES, --skip_images SKIP_IMAGES
                        skip images. Useful for fast testing

```


# Creating a camera_info yaml

To create a camera_info.yaml file, follow these steps:

1. Setup TangoRosStreamer
2. Rostopic echo a camera info message
3. Mannualy copy the values to a template camera_info file

Here's an example of the yaml created (Asus ZenPhone):

```yaml
image_width: 1920
image_height: 1080
camera_name: camera_color
camera_matrix:
  rows: 3
  cols: 3
  data: [1484.120151053405, 0.0, 982.1869163422028, 0.0, 1483.296808359715, 543.0569322723705, 0.0, 0.0, 1.0]
distortion_model: rational_polynomial
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.09370951394834764, -0.1640721737187069, 0.0, 0.0, 0.1224192868301668]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
projection_matrix:
  rows: 3
  cols: 4
  data: [1484.120151053405, 0.0, 982.1869163422028, 0.0, 0.0, 1483.296808359715, 543.0569322723705, 0.0, 0.0, 0.0, 1.0, 0.0]
```

More info at:

https://answers.ros.org/question/12193/how-to-get-and-use-camera-calibration-file/

https://answers.ros.org/question/248563/publishing-camerainfo-messages-from-yaml-file/


# Color correct

```bash
clear && ./test/gui_colorcorrect.py -p /home/mike/datasets/red_book_aruco/dataset/  -i calibrations/camera.yaml -z 0.06 -zbuf -zfilt  -si 15 -sv 100
```
