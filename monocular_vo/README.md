# Monocular Visual Odometry Demo (C++)
This demo builds visual odometry in C++ from scratch and runs on KITTI Dataset


Camera coordinate system: https://de.mathworks.com/help/vision/gs/coordinate-systems.html
## Steps

- Download KITTI grayscale dataset and put it inside `kitti_dataset` directory (https://www.cvlibs.net/datasets/kitti/eval_odometry.php)
  - Image dataset - KITTI grayscale dataset (used in `main.cpp`)
  - Ground truth - odometry ground truth poses (used in `vo.cpp`)
- Compile and run
  - `mkdir build && cd build`
  - `cmake ..`
  - `make`
  - run the executable

It was made in service of [this](https://www.youtube.com/watch?v=H_1OtbMD-sE) visual odometry series on Youtube.
Download odometry ground truth poses (4 MB)