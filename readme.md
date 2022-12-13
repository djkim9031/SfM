# SfM C++ #

Structure from Motion (SfM): Using the sequence of images (or video frames), this calculates the fundamental & essential matrices of epipolar geometry obtained from two base line images. These matrices are then used to construct 3D positions of the extracted landmarks (features) in other images. Currently, this does not support bundle adjustment, so the reprojected error may be high, but in the future bundle adjustment using Colmap is the identified scope of further work.

This work is based on & inspired from the python implementation of simple SfM from the git repo (https://github.com/harish-vnkt/structure-from-motion). This c++ version improves upon this code.

## Dependencies ##

OpenCV 4.6 (>=Version 4.0.0) as it denpends on opencv-contrib-4.x
PCL 1.12
VTK (for point cloud visualization)


