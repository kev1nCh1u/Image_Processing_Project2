# Image_Processing_Project2

![Alt text](img/Screenshot_from_2021-06-25_11-38-50.png?raw=true "Title")

## start cpp

    mkdir build
    cd build

    cmake ..
    make

    cmake -GNinja ..
    ninja

    ./txt_to_pcd "../data/STN6xyzi.txt"
    ./pcd_rotate_pan ../data/STN7xyzi.txt.pcd
    ./icp
    ./interactive_icp 20


## start pyhton
    pip install open3d
    pip install numpy
    pip install open3d-python

## git ignore defore
    git rm -rf --cached .


## view
http://lidarview.com/

## ref
http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html

## tools
pointcloud view
http://lidarview.com/

CloudCompare
https://www.danielgm.net/cc/