#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <iterator> // for ostream_iterator
#include <vector>
#include <fstream>

using namespace cv;
using namespace std;

void getCalibrationData(string dataset_path, double &focal, cv::Point2d &pp)
{

  // Load these values from KITTI's calibration files
  string calibration_file_path = dataset_path + "calib.txt";

  ifstream myfile(calibration_file_path);

  if (myfile.is_open())
  {
    string line;
    while (getline(myfile, line))
    {
      istringstream iss(line);
      vector<string> results((istream_iterator<string>(iss)), istream_iterator<string>());
      focal = stod(results[1]);
      pp.x = stod(results[3]);
      pp.y = stod(results[7]);

      break;
    }
    myfile.close();
  }
}
void featureTracking(Mat img_1, Mat img_2, vector<Point2f> &points1, vector<Point2f> &points2, vector<uchar> &status)
{

  // this function automatically gets rid of points for which tracking fails

  vector<float> err;
  Size window_size = Size(21, 21);
  TermCriteria term_criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(img_1, img_2, points1, points2, status, err, window_size, 3, term_criteria, 0, 0.001);

  // getting rid of points for which the KLT tracking failed or those who have gone outside the frame
  int index_correction = 0;
  for (int i = 0; i < status.size(); i++)
  {
    Point2f pt = points2.at(i - index_correction);
    if ((status.at(i) == 0) || (pt.x < 0) || (pt.y < 0))
    {
      if ((pt.x < 0) || (pt.y < 0))
      {
        status.at(i) = 0;
      }
      points1.erase(points1.begin() + (i - index_correction));
      points2.erase(points2.begin() + (i - index_correction));
      index_correction++;
    }
  }
}

void featureDetection(Mat img_1, vector<Point2f> &points1)
{
  // uses FAST for feature detections
  vector<KeyPoint> keypoints_1;
  int fast_threshold = 20;
  bool non_max_suppression = true;
  FAST(img_1, keypoints_1, fast_threshold, non_max_suppression);
  KeyPoint::convert(keypoints_1, points1, vector<int>());
}
