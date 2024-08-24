#include "vo.h"

#include <filesystem>
int main(int argc, char **argv)
{
  VisualOdometry vo;
  std::filesystem::path dataset_path = std::filesystem::current_path() / "../kitti_dataset" / "data_odometry_gray";
  std::filesystem::path imageset_path = dataset_path / "dataset/sequences/00/";

  int ret = vo.run(imageset_path.string());
  return ret;
}