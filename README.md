# Image_Processing_Project2

## start

    mkdir build
    cd build

    cmake ..
    make

    cmake -GNinja ..
    ninja

    ./txt_to_pcd "../data/STN6xyzi.txt"
    ./pcd_rotate_pan ../data/STN6xyzi.txt.pcd
    ./icp


## git ignore defore
    git rm -rf --cached .


## view
http://lidarview.com/