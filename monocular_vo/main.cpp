#include "vo.h"

int main(int argc, char **argv)
{
  VisualOdometry vo;

  string path = "/home/sharad/visual_odometry/monocular/data_odometry_gray/dataset/sequences/00/";
  int ret = vo.run(path);
  return ret;
}