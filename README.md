# 3dpose_ba

This repo uses cvsba for 3d pose recovery.  

# cvsba
[cvsba](https://www.uco.es/investiga/grupos/ava/node/39)  
There are sample codes in the Code Examples section.

## Install cvsba
Get and install cvsba from the following repo.  
<https://github.com/willdzeng/cvsba>


# Testing
There is a great example provided here.  
<https://github.com/sunglok/3dv_tutorial>  
[LICENSE](https://github.com/sunglok/3dv_tutorial/blob/master/LICENSE)  
The codes in **Multi-View Geometory** uses cvsba.  
You can test them with *bundle_adjustment_global.cpp*.  
(In order to use the example, you are required to run *image_formation.cpp* first.)  
For both codes, make sure you adjust the paths of the files, including *opencv_all.cpp*.  


# Using BA for your own data
My example *bundle_tester.cpp* is also based from the above code.  


```bash
git clone https://github.com/mkono505/3dpose_ba.git
cd 3dpose_ba
mkdir build
cd build
cmake ..
make
./bundle_tester
```

### Parameters
```bash
camera_focal: focal distance of camera
camera_center: centeral point of camera
n_views: number of cameras
fin: files of input data
Param type: Select the mode your using, MOTIONSTRUCTURE=3dpoints+camera camera parameters MOTION=camera parameters STRUCTURE=3d points
Param fixedIntrinsics: number of the intrinsic parameters to fix
Param fixedDistrotion: number of distrotion parameters to fix
fpts: the 3dpoint output file
fcam: the camera pose output file
```

Sample datas are in the data folder (image_formation0.xyz, image_formation1.xyz; data of two cameras), which is retrieved from OpenPose *BODY_25* model, with additional feature points based on human body constraints.


### Author
[Michinari Kono](https://github.com/mkono505)
