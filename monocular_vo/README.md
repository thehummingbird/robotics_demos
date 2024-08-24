# Monocular Visual Odometry Demo (C++)
This demo builds visual odometry in C++ from scratch and runs on KITTI Dataset


Camera coordinate system: https://de.mathworks.com/help/vision/gs/coordinate-systems.html
## Steps

- Download KITTI grayscale dataset and put it inside `kitti_dataset` directory (https://www.cvlibs.net/datasets/kitti/eval_odometry.php). Please look at `main.cpp` to confirm the path or set your own path.
- Compile and run
  - `mkdir build && cd build`
  - `cmake ..`
  - `make`
  - run the executable

It was made in service of [this](https://www.youtube.com/watch?v=H_1OtbMD-sE) visual odometry series on Youtube.
