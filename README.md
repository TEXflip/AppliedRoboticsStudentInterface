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
estimate the rotation and translation vectors of the camera starting from the undistorted image of the camera and the points of the arena.

First the function check if it already exists the file with the measurement of the points of the arena precedently setted. If the file doesn't exists, it will appear a image of the camera where will be asked to the user to click in the four corner of the arena in counterclockwise order. After, using the function "solvePnP" are computed the rotation and translation vectors of the camera.

### imageUndistort
```c++
void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &cam_matrix, 
                    const cv::Mat &dist_coeffs, const std::string &config_folder)
```
It removes the distortion of the lens of the camera from the image in input from the parameters computed during the camera calibration phase.

Since it's sufficient to calculate the calibration matrix to transform the image only one time, The function, when it's called the first time, it uses initUndistortRectifyMap() to compute the two maps of the two axis X and Y of the calibration matrix.
Finally, everyyime the function is called, it computes the undistorted image with the function "remap()" using the 2 maps precedently calculated.

### findPlaneTransform
```c++
void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec, const cv::Mat &tvec, 
                        const std::vector<cv::Point3f> &object_points_plane, 
                        const std::vector<cv::Point2f> &dest_image_points_plane, cv::Mat &plane_transf, 
                        const std::string &config_folder)
```
It computes the transformation matrix to unwrap the image from the points taken before.

using "projectPoints()" function, findPlaneTransform() projects the 3D points to a 2D image plane and then with "getPerspectiveTransform()" it computes the 3x3 perspective transformation of the corrisponding points.

### unwarp
```c++
void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf, const std::string &config_folder)
```
it apply the transformation to convert the 3D points in a 2D plane.

using "warpPerspective()" function it applies the transformation computed by "findPlaneTransform()" to unwrap the image and get a top-view visualization

### processMap
```c++
bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, 
                std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
```
Process the image to detect victims, obstacles and the gate.

code flow: obstacle list
0. convert input image in hsv colorspace 
1. use a colorfilter to sort the objects in different masks (red for obstacles and green for victims and the gate)
2. apply in the red mask the dilate and erode morphological transformations
3. extract contour of the obstacles with findContours(), approximate them with approxPolyDP() and finally scale them
4. Assign the found obstacles in the output list
5. apply in the green mask the dilate and erode morphological transformations
6. extract contour of the victims and the gate with findContours(), approximate them with approxPolyDP(), scale them and finally extract the gate by controlling its contour size(=4)
7. victims elaboration process:
    - detect round contours (with size > 4)
    - eliminate the green surrounding using as a mask the not operation of the green mask 
    - load the template numbers and flip them to match the camera perspective transformation applied in unwarp() 
    - run the number detection for every boundingBox:
        * extract the region of interest from the image with the boundingBox
        * resize it to template size
        * compare the detected numbers with the templates trying 4 different rotation (90 degrees) and compute the mathing score
        * select the tamplate according too the heighest matching score
        * save the pair of matched template number and scaled victim in a map in order to sort them
        * copy the ordered map into the output vector



### findRobot
```c++
bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta, const std::string &config_folder)
```

0. convert input image in hsv colorspace 
1. filter the blue areas out of the hsv image
2. find the contour of the robot triangle using findContours()
4. approximate the contours
5. look for the triangle contour by taking off all the areas which are too small ot too big and the contours with too edges
6. scale the found triangle contour 
7. compute the position and rotation vectors of the robot (center of gravity and rotation relative to the x axis)