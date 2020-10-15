#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>

#include <vector>
#include <atomic>
#include <unistd.h>
// COMMENT DA DAVID
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
      break;
    }
    case 27: // Esc key
    {
      exit(0);
      break;
    }
    default:
      break;
    }
  }

#pragma region extrinsicCalib functions

  // funcions for extrinsic calibration
  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;

  void mouseCallback(int event, int x, int y, int, void *p) // function from professor interface
  {
    if (event != cv::EVENT_LBUTTONDOWN || done.load())
      return;

    result.emplace_back(x * show_scale, y * show_scale);
    cv::circle(bg_img, cv::Point(x, y), 20 / show_scale, cv::Scalar(0, 0, 255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n)
    {
      usleep(500 * 1000);
      done.store(true);
    }
  }

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img) // function from professor interface
  {
    result.clear();
    cv::Size small_size(img.cols / show_scale, img.rows / show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load())
    {
      cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
  }

  bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points, const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
  {
    std::string file_path(config_folder + "/extrinsicCalib.csv");

    std::vector<cv::Point2f> image_points;

    if (!std::experimental::filesystem::exists(file_path))
    {

      std::experimental::filesystem::create_directories(config_folder);
      image_points = pickNPoints(4, img_in);

      std::ofstream output(file_path);
      if (!output.is_open())
        throw std::runtime_error("Cannot write file: " + file_path);
      for (const auto pt : image_points)
        output << pt.x << " " << pt.y << std::endl;
      output.close();
    }
    else
    {
      std::ifstream input(file_path);
      if (!input.is_open())
        throw std::runtime_error("Cannot read file: " + file_path);
      while (!input.eof())
      {
        double x, y;
        if (!(input >> x >> y))
          if (input.eof())
            break;
          else
            throw std::runtime_error("Bad file: " + file_path);
        image_points.emplace_back(x, y);
      }
      input.close();
    }

    cv::Mat dist_coeffs(1, 4, CV_32F);
    bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs, rvec, tvec);

    if (!ok)
      std::cerr << "FAILED SOLVE_PNP" << std::endl;

    return ok;
  }

#pragma endregion

  void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                      const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
  {
    // TODO: capire come e dove (nel codice) va a prendersi i parametri per la calibrazione + implementazione del salvataggio dei parametri (da fare in camera_calibration.cpp)
    //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs); // TODO: fast undistort

    static bool maps_initialized = false;
    static cv::Mat full_map1, full_map2;

    if(!maps_initialized){
      cv::Mat R;
      cv::initUndistortRectifyMap(cam_matrix, dist_coeffs, R, cam_matrix, img_in.size(), CV_16SC2, full_map1, full_map2);

      maps_initialized = true;
    }
 
    cv::remap(img_in, img_out, full_map1, full_map2, cv::INTER_LINEAR);    
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
