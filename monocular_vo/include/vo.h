#include "vo_features.h"

#include <filesystem>

using namespace cv;
using namespace std;

class VisualOdometry
{
public:
  VisualOdometry() = default;
  int run(string dataset_path);
};
