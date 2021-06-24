# Image_Processing_Project2

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

## git ignore defore
    git rm -rf --cached .


## view
http://lidarview.com/

## ref
http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html