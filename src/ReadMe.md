
# Documentation for student_inteface

```c++
void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
```

```c++
bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
```

```c++
void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
```

```c++
void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec, const std::vector<cv::Point3f> &object_points_plane, const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf, const std::string &config_folder)
```

```c++
void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder)
```

```c++
bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
```