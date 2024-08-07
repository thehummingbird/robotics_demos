#include "vo_features.h"

using namespace cv;
using namespace std;

class VisualOdometry
{
public:
  VisualOdometry();
  int run(string dataset_path);
};
