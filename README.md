# Laboratory of Applied Robotics Student Interface
Group: IncludeMe
Members: Michele Tessari, David Karbon



# Documentation of student_inteface

### genericImageListener
```c++
void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
```
function to save the images from the camera to be used in a second moment to calculate the distortion parameters of the camera or for other purpose.

The function creates the folder (from the configuration default path) where to save the images and, if it already exists, it continues to try with another name until it will find one. Then it shows the image of the current visual of the camera and (if 's' is pressed) it saves the image in the folder. Otherwise, if 'esc' is pressed, the program will close.

### extrinsicCalib
```c++
bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, 
                    const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
```


### imageUndistort
```c++
void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &cam_matrix, 
                    const cv::Mat &dist_coeffs, const std::string &config_folder)
```

### findPlaneTransform
```c++
void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec, 
                        const std::vector<cv::Point3f> &object_points_plane, 
                        const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf, 
                        const std::string &config_folder)
```

### unwarp
```c++
void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder)
```
using "warpPerspective()" function it applies the transformation computed by "findPlaneTransform()" to unwrap the image and get a top-view visualization


### processMap
```c++
bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, 
                std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
```