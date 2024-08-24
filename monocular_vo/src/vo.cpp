#include "vo.h"

#define MAX_FRAME 1000
#define MIN_NUM_FEAT 2000

double getAbsoluteScale(int frame_id, int sequence_id, double z_cal)
{

  string line;
  int i = 0;
  ifstream myfile("/home/sharad/visual_odometry/monocular/data_odometry_gray/data_odometry_poses/dataset/poses/00.txt");
  double x = 0, y = 0, z = 0;
  double x_prev, y_prev, z_prev;
  if (myfile.is_open())
  {
    while ((getline(myfile, line)) && (i <= frame_id))
    {
      z_prev = z;
      x_prev = x;
      y_prev = y;
      std::istringstream in(line);
      for (int j = 0; j < 12; j++)
      {
        in >> z;
        if (j == 7)
          y = z;
        if (j == 3)
          x = z;
      }

      i++;
    }
    myfile.close();
  }

  else
  {
    cout << "Unable to open file";
    return 0;
  }

  return sqrt((x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev));
}

VisualOdometry::VisualOdometry()
{
}

int VisualOdometry::run(string dataset_path)
{
  Mat img_1, img_2;
  Mat R_f, t_f; // the final rotation and tranlation vectors containing the

  double scale = 1.00;
  char filename1[200];
  char filename2[200];
  sprintf(filename1, (dataset_path + "image_0/%06d.png").c_str(), 0);
  sprintf(filename2, (dataset_path + "image_0/%06d.png").c_str(), 1);

  char text[100];
  int font_face = FONT_HERSHEY_PLAIN;
  double font_scale = 1;
  int thickness = 1;
  cv::Point text_org(10, 50);

  // read the first two frames from the dataset
  Mat img_1_c = imread(filename1);
  Mat img_2_c = imread(filename2);

  if (!img_1_c.data || !img_2_c.data)
  {
    std::cout << "Error reading images! " << std::endl;
    return -1;
  }

  // we work with grayscale images
  cvtColor(img_1_c, img_1, COLOR_BGR2GRAY);
  cvtColor(img_2_c, img_2, COLOR_BGR2GRAY);

  // feature detection, tracking
  vector<Point2f> points1, points2; // vectors to store the coordinates of the feature points
  featureDetection(img_1, points1); // detect features in img_1
  vector<uchar> status;
  featureTracking(img_1, img_2, points1, points2, status); // track those features to img_2

  double focal;
  cv::Point2d pp;
  getCalibrationData(dataset_path, focal, pp);

  // recovering the pose and the essential matrix
  Mat E, R, t, mask;
  E = findEssentialMat(points2, points1, focal, pp, RANSAC, 0.999, 1.0, mask);
  recoverPose(E, points2, points1, R, t, focal, pp, mask);
  Mat prev_image = img_2;
  Mat curr_image;
  vector<Point2f> prev_features = points2;
  vector<Point2f> curr_features;

  char filename[100];

  R_f = R.clone();
  t_f = t.clone();

  clock_t begin = clock();

  namedWindow("Ego Motion Camera", WINDOW_AUTOSIZE); // Camera Display Window
  namedWindow("Trajectory", WINDOW_AUTOSIZE);        // Trajectory Display Window

  Mat traj = Mat::zeros(600, 600, CV_8UC3);

  for (int num_frame = 2; num_frame < MAX_FRAME; num_frame++)
  {
    sprintf(filename, (dataset_path + "image_0/%06d.png").c_str(), num_frame);
    Mat curr_image_c = imread(filename);
    cvtColor(curr_image_c, curr_image, COLOR_BGR2GRAY);
    vector<uchar> status;
    featureTracking(prev_image, curr_image, prev_features, curr_features, status);

    E = findEssentialMat(curr_features, prev_features, focal, pp, RANSAC, 0.999, 1.0, mask);
    recoverPose(E, curr_features, prev_features, R, t, focal, pp, mask);
    Mat prevPts(2, prev_features.size(), CV_64F), currPts(2, curr_features.size(), CV_64F);

    for (int i = 0; i < prev_features.size(); i++)
    { // this (x,y) combination makes sense as observed from the source code of triangulatePoints on GitHub
      prevPts.at<double>(0, i) = prev_features.at(i).x;
      prevPts.at<double>(1, i) = prev_features.at(i).y;

      currPts.at<double>(0, i) = curr_features.at(i).x;
      currPts.at<double>(1, i) = curr_features.at(i).y;
    }

    scale = getAbsoluteScale(num_frame, 0, t.at<double>(2));

    if ((scale > 0.1) && (t.at<double>(2) > t.at<double>(0)) && (t.at<double>(2) > t.at<double>(1)))
    {

      t_f = t_f + scale * (R_f * t); // t_final = t_previous + scale * (R_previous * t_current)
      R_f = R * R_f;                 // R_final = R_current * R_previous
    }

    // a redetection is triggered in case the number of features being trakced go below a particular threshold
    if (prev_features.size() < MIN_NUM_FEAT)
    {
      featureDetection(prev_image, prev_features);
      featureTracking(prev_image, curr_image, prev_features, curr_features, status);
    }

    prev_image = curr_image.clone();
    prev_features = curr_features;

    int x = int(t_f.at<double>(0)) + 300;
    int y = int(-1 * t_f.at<double>(2)) + 500;
    circle(traj, Point(x, y), 1, CV_RGB(255, 0, 0), 2);

    rectangle(traj, Point(10, 30), Point(550, 50), CV_RGB(0, 0, 0), cv::FILLED);
    sprintf(text, "Coordinates: x = %02fm y = %02fm z = %02fm", t_f.at<double>(0), t_f.at<double>(1), t_f.at<double>(2));
    putText(traj, text, text_org, font_face, font_scale, Scalar::all(255), thickness, 8);

    imshow("Road facing camera", curr_image_c);
    imshow("Trajectory", traj);

    waitKey(1);
  }

  return 0;
}
