#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>

namespace student
{
  const std::string RAW_IMAGE("/image/raw");

  void loadImage(cv::Mat &img_out, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED");
  }

  void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
  {
    static bool folderCreated = false;
    static std::string save_path;
    static int id = 0;

    if (!folderCreated)
    {
      bool exist = true;
      int i;
      for (i = 0; exist && i < 1000; i++)
      {
        save_path = config_folder + "/saved_images" + std::to_string(i) + "/";
        exist = std::experimental::filesystem::exists(save_path);
      }
      
      if (i > 999 || !std::experimental::filesystem::create_directories(save_path))
        throw std::logic_error("NO EMPTY FOLDER AVAILABLE");
      
      folderCreated = true;
    }

    cv::imshow(topic, img_in);
    char c = cv::waitKey(30);

    switch (c)
    {
    case 's':
    {
      std::string img_file(save_path + "img" + std::to_string(id++) + ".jpg");
      cv::imwrite(img_file, img_in);

      std::cout << "Saved image: " << img_file << std::endl;
    }
    break;
    default:
      break;
    }
  }

  bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED");
  }

  void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                      const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
  {

    throw std::logic_error("STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED");
  }

  void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                          const cv::Mat &tvec, const std::vector<cv::Point3f> &object_points_plane,
                          const std::vector<cv::Point2f> &dest_image_points_plane,
                          cv::Mat &plane_transf, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED");
  }

  void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
              const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED");
  }

  bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED");
  }

  bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta, const std::string &config_folder)
  {
    throw std::logic_error("STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED");
  }

  bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list, const std::vector<std::pair<int, Polygon>> &victim_list, const Polygon &gate, const float x, const float y, const float theta, Path &path)
  {
    throw std::logic_error("STUDENT FUNCTION - PLAN PATH - NOT IMPLEMENTED");
  }

} // namespace student
