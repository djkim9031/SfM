# SfM C++ #

Structure from Motion (SfM): Using the sequence of images (or video frames), this calculates the fundamental & essential matrices of epipolar geometry obtained from two base line images. These matrices are then used to construct 3D positions of the extracted landmarks (features) in other images. <br/>

Currently, this does not support bundle adjustment, so the reprojected error may be high, but in the future bundle adjustment using Colmap is the identified scope of further work.<br />

This work is based on & inspired from the python implementation of simple SfM from the Harish's git [repo](https://github.com/harish-vnkt/structure-from-motion) <br/>
This c++ version improves upon this code.

The input dataset and camera intrinsic values can be found [here](https://github.com/openMVG/SfM_quality_evaluation)

### Fountain ###
With 11 input images, and using SIFT detector & 50000 max landmarks

![0008](https://user-images.githubusercontent.com/58758359/207217614-ad4357ce-15a8-414c-9e80-54bc640bfcd1.jpg)
![fountain](https://user-images.githubusercontent.com/58758359/207217709-ad4893e2-40a1-424e-86fe-a4e5ddefa588.png)
![fountain2](https://user-images.githubusercontent.com/58758359/207217721-561b5a97-5af0-4411-ac84-f070ddb043c1.png)

### Entrance ###
With 10 input images, and using SIFT detector & 50000 max landmarks

![0006](https://user-images.githubusercontent.com/58758359/207217786-48e6170e-4871-4752-9a39-8bd51e3d4061.jpg)
![entrance](https://user-images.githubusercontent.com/58758359/207217802-2b810135-c9f8-40d3-aadb-f069420c86e2.png)
![entrance2](https://user-images.githubusercontent.com/58758359/207217804-d25ef65d-12a8-45f5-87b2-ea03fabead7f.png)


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
2. Bundle adjustment for lower reprojection errors <br />

