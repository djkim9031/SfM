# SfM C++ #

Structure from Motion (SfM): Using the sequence of images (or video frames), this calculates the fundamental & essential matrices of epipolar geometry obtained from two base line images. These matrices are then used to construct 3D positions of the extracted landmarks (features) in other images. Currently, this does not support bundle adjustment, so the reprojected error may be high, but in the future bundle adjustment using Colmap is the identified scope of further work.<br />

This work is based on & inspired from the python implementation of simple SfM from the git repo (https://github.com/harish-vnkt/structure-from-motion). This c++ version improves upon this code.

<Foundtain> 
![0008](https://user-images.githubusercontent.com/58758359/207216689-1449b1f2-878f-409b-b9ad-a5bc2bb4570c.jpg)
![fountain](https://user-images.githubusercontent.com/58758359/207216728-491b24c7-99c0-4c4c-94a0-0a6c16dfdcb4.png)
![fountain2](https://user-images.githubusercontent.com/58758359/207216738-9eab5bb4-b29a-4749-bc30-a958fdf48fec.png)

## Dependencies ##

OpenCV 4.6 (>=Version 4.0.0) as it denpends on opencv-contrib-4.x <br />
PCL 1.12 <br />
VTK (for point cloud visualization) <br />

## Directory Structure ##
```
Root
|__ [build]                 <- Project make files
|__ [includes]              <- Project header files
|   |__ xxx.h               <- header files
|__ [src]                   <- Project source files
|__ |__ xxx.cpp             <- sfm codes
|__ CMakeLists.txt          <- Project CMakeLists
|__ main.cpp                <- Main function
|__ readme.md               <- Project readme file        
```

[TODO]<br />
1. Intrinsic matrix calculation with multiple image inputs using Colmap <br />
2. Bundle adjustment for better reprojection errors <br />

