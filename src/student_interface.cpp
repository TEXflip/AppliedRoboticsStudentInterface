#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
// #include "collision_detection.hpp"

#include "DubinsCurves.hpp"
// #include "voronoiHelper.hpp"
#include "graph.hpp"
#include "voronoiHandler.hpp"
#include "gridBasedPlanning.hpp"
#include "Astar_pathplanning.hpp"
// #include "graph.hpp"

#include "collision_detection.hpp"

#include <stdexcept>
#include <sstream>
#include <experimental/filesystem>
#include <cstdlib>

#include <opencv2/imgproc.hpp>

#include <vector>
#include <atomic>
#include <unistd.h>
#include <fstream>
#include "debug.hpp"

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
      //std::system("./camera_calibration");
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

  void mouseCallback(int event, int x, int y, int, void *p) // function from professor interface to get the mouseklick points
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

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat &img) // function from professor interface to schow the picture and get the 4 points
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

    cv::Mat dist_coeffs(1, 4, CV_32F); // coeficients from the cameracalibration;?
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
    //cv::undistort(img_in, img_out, cam_matrix, dist_coeffs);

    static bool maps_initialized = false;
    static cv::Mat full_map1, full_map2;

    if (!maps_initialized)
    {
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
    cv::Mat img_points;

    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), img_points);

    plane_transf = cv::getPerspectiveTransform(img_points, dest_image_points_plane);
  }

  void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
              const std::string &config_folder)
  {
    cv::warpPerspective(img_in, img_out, transf, img_in.size());
  }

  bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacle_list, std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
  {
    static const int W_0 = 300;
    static const int H_0 = 0;
    static const int OFFSET_W = 10;
    static const int OFFSET_H = 100;

    cv::imwrite("graph.jpg", img_in);
    std::ofstream outfile;
    outfile.open(config_folder + "/scale.txt", std::ios_base::out);
    outfile << scale;

    //colorspace convertion
    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    // Matrix for the colormasks
    cv::Mat red_mask_high, red_mask_low, red_obstacle_mask, green_victim_mask, black_border_mask;

    /*!
      values for masks (H_low,S_low,V_low) (H_hight,S_hight,V_hight)
      RED     (0,10,150) (10,255,255) && (142,029,199) (180,255,255)
      Green   (52,12,151) (82,255,255)
      Black   (0,0,0)   (0-180..doesnt mater,1-180,225) !! the V value is important to be at 224
      The values change if applied to real images
    */

    //selecting the red_obstacles
    cv::inRange(hsv_img, cv::Scalar(0, 9, 69), cv::Scalar(20, 255, 255), red_mask_high);

    //for real images use hue values left and right from 0 in order to get the best result

    cv::inRange(hsv_img, cv::Scalar(158, 93, 33), cv::Scalar(180, 255, 253), red_mask_low);

    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_obstacle_mask);

    //selecting the green_victims AND the gate
    cv::inRange(hsv_img, cv::Scalar(40, 20, 50), cv::Scalar(79, 255, 255), green_victim_mask);

    //selecting the black border if needed. Atention(numbers get also included)
    //cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(10, 10, 225), red_obstacle_mask);

    //process RED_OBSTACLES

    std::vector<std::vector<cv::Point>> contours, contours_approx; //define point vectros for the curves and contoures
    std::vector<cv::Point> approx_curve;

    //Apply enventally some filtering
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));
    cv::dilate(red_obstacle_mask, red_obstacle_mask, kernel);
    cv::erode(red_obstacle_mask, red_obstacle_mask, kernel);

    //find contours
    cv::findContours(red_obstacle_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //process the found contours (aprox and scale)
    for (int i = 0; i < contours.size(); ++i)
    {

      approxPolyDP(contours[i], approx_curve, 8, true); // approxPolyDP( InputArray curve,OutputArray approxCurve,double epsilon, bool closed )
                                                        //function that closes eventual opend contoures ???

      //scaling loop
      Polygon scaled_contour; //typedev vector

      for (const auto &pt : approx_curve)
      {
        scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
      }

      obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list
    }

    //process GREEN_VICTIMS AND GATE///////////////////////////////////////
    // eventual filtering on green_victim_mask
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));
    cv::dilate(green_victim_mask, green_victim_mask, kernel);
    cv::erode(green_victim_mask, green_victim_mask, kernel);

    //find contours
    cv::findContours(green_victim_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //elaborating the found contours
    const double MIN_AREA_SIZE = 100;
    cv::Mat contours_img_boundingbox = cv::Mat(img_in.size(), CV_8UC3, cv::Scalar::all(0));
    std::vector<cv::Rect> boundRect(contours.size());
    Polygon scaled_contour_green;
    std::vector<cv::Point> contours_approx_array[12];

    for (int i = 0; i < contours.size(); ++i)
    {
      double area = cv::contourArea(contours[i]); // check the contour area to remove false positives
      if (area < MIN_AREA_SIZE)
        continue;

      approxPolyDP(contours[i], approx_curve, 11, true); //aproxximate the contoure in less vertices

      if (approx_curve.size() == 4) //if i have a gate(a quadratic figure in green)
      {
        for (const auto &pt : approx_curve)
          scaled_contour_green.emplace_back(pt.x / scale, pt.y / scale);

        gate = scaled_contour_green;
      }
      else if (approx_curve.size() > 4) //  //if i have a number circle and not the gate
      {
        contours_approx_array[i] = {approx_curve};          //assosciate the found aprox contour to the same number as the boundingbox (sync the arrays)
        boundRect[i] = boundingRect(cv::Mat(approx_curve)); // finds bounding box for each green blob
      }
    }

    ////////////////TEMPLATEMATCHING//////////////////
    cv::Mat green_mask_inv, filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::bitwise_not(green_victim_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask

    // Load Templatenumbers into a vector
    std::vector<cv::Mat> templROIs;
    for (int i = 0; i <= 9; ++i)
    {
      cv::Mat templImg = cv::imread("/home/ubuntu/Desktop/workspace/project/src/01_template_matching/template/" + std::to_string(i) + ".png");
      cv::flip(templImg, templImg, 1);
      templROIs.emplace_back(templImg);
    }

    img_in.copyTo(filtered, green_mask_inv); //creates a copy of image without green surounding

    double score;
    std::map<int, Polygon> victim_Map; //create a ma to sort the detected number with the position of the

    //loop for every numberBlob detected
    std::cout << "boundingbox size: " << boundRect.size() << std::endl;

    for (int i = 0; i < boundRect.size(); ++i)
    {
      cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit()

      if (processROI.empty())
        continue;

      cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI to match with the template size!!!!!
      cv::threshold(processROI, processROI, 100, 255, 0);     // threshold and binarize the image, to suppress some noise

      // Apply some additional smoothing and filtering
      cv::erode(processROI, processROI, kernel);
      cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
      cv::erode(processROI, processROI, kernel);

      // Find the template digit with the best matching
      cv::Mat templROIturned;
      cv::Mat result;
      double maxScore = 0;
      int maxIdx = -1;
      // std::cout << templROIs.size();

      for (int j = 0; j < templROIs.size(); ++j)
      {
        for (int r = 0; r < 360; r += 90)
        {

          cv::Point2f pc((templROIs[j].cols - 1) / 2., (templROIs[j].rows - 1) / 2.);
          cv::Mat rot = cv::getRotationMatrix2D(pc, r, 1.0);
          cv::warpAffine(templROIs[j], templROIturned, rot, templROIs[j].size());

          cv::matchTemplate(processROI, templROIturned, result, cv::TM_CCOEFF);

          cv::minMaxLoc(result, nullptr, &score);
          if (score > maxScore)
          {
            maxScore = score;
            maxIdx = j;
          }
        }
      }

      Polygon scaled_contour; //typedev vector

      //scale the found contour
      for (const auto &pt : contours_approx_array[i])
        scaled_contour.emplace_back(pt.x / scale, pt.y / scale);

      victim_Map.insert(std::pair<int, Polygon>(maxIdx, scaled_contour)); // insert the found blob in the MAP in order o assign the contour to the value

      std::cout << "Best fitting template: " << maxIdx << std::endl;
    }

    //copy the sorted map into the vector
    victim_list.assign(victim_Map.begin(), victim_Map.end());

    std::cout << "victims: " << victim_list.size() << "\tobstacles: " << obstacle_list.size() << std::endl;
    return victim_list.size() > 0 && obstacle_list.size() > 0;
  }

  bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle, double &x, double &y, double &theta, const std::string &config_folder)
  {
    /*!
      1. filter the blue areas out of the hsv image
      2. apply some filtering
      3. analyse the robot position(barricentre and rotation relative to x axis)
    */

    cv::Mat blue_mask, hsv_img;

    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_img, cv::Scalar(85, 103, 19), cv::Scalar(123, 255, 255), blue_mask);

    // Blur Fiilter????

    // Process blue mask
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;
    // cv::imshow("blue mask" ,blue_mask);
    // cv::waitKey(10);
    cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool found = false;
    for (int i = 0; i < contours.size() && found == false; ++i)
    {

      cv::approxPolyDP(contours[i], approx_curve, 15, true);
      contours_approx = {approx_curve};

      double area = cv::contourArea(approx_curve);
      // for (int j = 0; j < approx_curve.size(); j++)
      // {
      //   std::cout<< "\tx: " << approx_curve[j].x<< "y: " << approx_curve[j].y <<std::endl;
      // }
      // std::cout<< "area: " << area << std::endl;

      if (approx_curve.size() != 3)
        continue;

      if (area < 1000 || area > 3000)
        continue;

      found = true;
    }

    // cv::drawContours(img_in, contours_approx, -1, cv::Scalar(0, 170, 220), 3, cv::LINE_AA);
    // cv::imshow("robot contours", img_in);
    // cv::waitKey(0);

    //find the barrycentre and rotation
    if (found)
    {
      //scale the contoure
      for (const auto &pt : approx_curve)
        triangle.emplace_back(pt.x / scale, pt.y / scale);

      //calculate the barrycentre
      double cx = 0, cy = 0;
      for (auto item : triangle)
      {
        cx += item.x;
        cy += item.y;
      }
      cx /= triangle.size();
      cy /= triangle.size();

      double dst = 0;
      Point vertex;
      for (auto &item : triangle)
      {
        double dx = item.x - cx;
        double dy = item.y - cy;
        double curr_d = dx * dx + dy * dy;
        if (curr_d > dst)
        {
          dst = curr_d;
          vertex = item;
        }
      }
      double dx = cx - vertex.x;
      double dy = cy - vertex.y;

      x = cx;
      y = cy;
      theta = std::atan2(dy, dx);

      // std::cout << "x: " << x << "\ty: " << y << "\ttheta: " << theta * 180 / M_PI << "°" << std::endl;
    }
    return found;
  }

  bool planPath(const Polygon &borders, const std::vector<Polygon> &obstacle_list,
                const std::vector<std::pair<int, Polygon>> &victim_list,
                const Polygon &gate, const float x, const float y, const float theta, Path &path, const std::string &config_folder)
  {
    std::vector<Point> printPoints;
    printPoints.emplace_back(x, y);

    auto avgPoint = [](const Polygon &polygon) {
      float avgX = 0, avgY = 0;
      for (int i = 0; i < polygon.size(); i++)
      {
        avgX += polygon[i].x;
        avgY += polygon[i].y;
      }
      avgX /= polygon.size();
      avgY /= polygon.size();
      return Point(avgX, avgY);
    };

    // DubinsCurvesHandler dcHandler(5);
    // Point gateCenter

    Point avgGate = avgPoint(gate);

    // DubinsCurve c = dcHandler.findShortestPath(x, y, theta, gateCenter.x, gateCenter.y, 0);
    // std::cout << "x: " << x << "\ty: " << y << "\tth: " << theta << "\tgateX: " << gateCenter.x << "\tgateY: " << gateCenter.y << std::endl;

    // std::vector<DubinsLine> lines = dcHandler.discretizeDubinsCurve(c, 0.02);
    // float L = 0;

    // for (int i = 0; i < lines.size(); i++)
    //   path.points.emplace_back(lines[i].s, lines[i].x, lines[i].y, lines[i].th, lines[i].k);

    Graph::Graph graph;
    // VoronoiHandler::buildVoronoi(borders, obstacle_list, graph, 100, 1e6);
    float sideLength = 0.02;
    int nOriz = max(max(borders[0].x, borders[1].x), max(borders[2].x, borders[3].x)) / sideLength;
    int nVert = max(max(borders[0].y, borders[1].y), max(borders[2].y, borders[3].y)) / sideLength;

    auto toGraphCoord = [sideLength](float coord) {
      return (int)((coord + sideLength * 0.5) / sideLength);
    };

    buildGridGraph(graph, obstacle_list, nVert, nOriz, sideLength);

    /////////////////////////////////////////////////////////////////////////
    //calculate center of victims in order to calculate the path
    vector<Point> victim_centers;
    Point center;

    std::vector<std::pair<int, Polygon>>::const_iterator itv = victim_list.begin();

    for (std::vector<std::pair<int, Polygon>>::const_iterator itv = victim_list.begin(); itv != victim_list.end(); itv++)
    {
      center = avgPoint(itv->second);
      victim_centers.push_back(center);
    }
    std::cout << "center size " << victim_centers.size() << std::endl;
    std::cout << "center size victim 0 " << victim_centers[0].x << "       " << victim_centers[0].y << std::endl;

    ///////////////////////////////////////////////////////////////////////////////
    //convert the position of a point in the grid reference system
    int victim1x = toGraphCoord(victim_centers[0].x);
    int victim1y = toGraphCoord(victim_centers[0].y);
    int victim2x;
    int victim2y;
    int robotX = toGraphCoord(x);
    int robotY = toGraphCoord(y);

    printPoints.emplace_back(avgGate);
    int gateX = toGraphCoord(avgGate.x);
    int gateY = toGraphCoord(avgGate.y);
    vector<int> segmentsize;

    vector<int> opti_path;
    vector<int> smoothed_path;
    vector<int> path_segment = Astar::Solve_AStar(graph, (robotY * nOriz + robotX), (victim1y * nOriz + victim1x));
    Astar::smoothPath(graph, path_segment, smoothed_path, obstacle_list);

    opti_path.insert(opti_path.end(), smoothed_path.begin(), smoothed_path.end());
    path_segment.clear();
    smoothed_path.clear();

    for (int i = 0; i < victim_centers.size() - 1; i++)
    {
      victim1x = toGraphCoord(victim_centers[i].x);
      victim1y = toGraphCoord(victim_centers[i].y);
      victim2x = toGraphCoord(victim_centers[i + 1].x);
      victim2y = toGraphCoord(victim_centers[i + 1].y);

      path_segment = Astar::Solve_AStar(graph, (victim1y * nOriz + victim1x), (victim2y * nOriz + victim2x));
      Astar::smoothPath(graph, path_segment, smoothed_path, obstacle_list);

      if (smoothed_path.size() == 2)
        opti_path.push_back(smoothed_path[1]);
      else
        opti_path.insert(opti_path.end(), smoothed_path.begin() + 1, smoothed_path.end());

      path_segment.clear();
      smoothed_path.clear();
    }

    path_segment = Astar::Solve_AStar(graph, (victim2y * nOriz + victim2x), (gateY * nOriz + gateX));

    Astar::smoothPath(graph, path_segment, smoothed_path, obstacle_list);

    opti_path.insert(opti_path.end(), smoothed_path.begin() + 1, smoothed_path.end());
    path_segment.clear();
    smoothed_path.clear();

    for (int p : opti_path)
    {
      std::cout << "x: " << graph[p].x << "\ty: " << graph[p].y << std::endl;
    }
    /*
x: 0.16	y: 0.22
x: 0.8	y: 0.2
x: 1.3	y: 0.2
x: 1.32	y: 0.46
x: 1.32	y: 0.76
x: 1.32	y: 1.02

     */

    ///////////////////////////////////////////////////////////////

    //create pose vector for dubins curve
    vector<Pose> pose;
    Pose p;
    float x1;
    float x2;
    float y1;
    float y2;
    float b;       //cross product
    int sign;      // sign del cross product
    float av;      // angle between exit direction and x - axis
    float a_seg;   //angle between the 2 segments (entry and exit direction)
    float theta_f; // angle for the robot to assume in the certain location (theta f for the dubins)
    float mag;
    float dot;

    //initial position
    p.x = x;
    p.y = y;
    p.theta = theta;
    pose.push_back(p);

    for (int i = 0; i < opti_path.size() - 2; i++)
    {
      x1 = graph[opti_path[i + 1]].x - graph[opti_path[i]].x;
      x2 = graph[opti_path[i + 2]].x - graph[opti_path[i + 1]].x;
      y1 = graph[opti_path[i + 1]].y - graph[opti_path[i]].y;
      y2 = graph[opti_path[i + 2]].y - graph[opti_path[i + 1]].y;

      dot = x1 * x2 + y1 * y2; // dot product between [x1, y1] and [x2, y2]
      mag = (sqrt((x1 * x1) + (y1 * y1))) * (sqrt((x2 * x2) + (y2 * y2)));

      b = ((x1 * y2) - (x2 * y1));                //cros product
      sign = b > 0 ? 1 : -1;                      // sign of cross product
      av = atan2(y2, x2);                         //angle between exit direction and x -axis
      a_seg = M_PI - acos((dot) / (mag));         // angle between entrz and exit direction
      theta_f = sign * a_seg / 2 + av + M_PI / 2; // theta f
      if (sign > 0)
      {
        theta_f = theta_f - M_PI;
      }
      p.x = graph[opti_path[i + 1]].x;
      p.y = graph[opti_path[i + 1]].y;
      p.theta = theta_f;
      pose.push_back(p);
      //cout << "angle " << theta_f * 180 / M_PI << " x " << graph[opti_path[i + 1]].x << " y " << graph[opti_path[i + 1]].y << endl;
    }
    //final position
    p.x = avgGate.x;
    p.y = avgGate.y;
    pose.push_back(p);
    for(int i = 0; i < pose.size();i++)
    {
      std::cout << "angle " << pose[i].theta * 180 / M_PI << " x " << pose[i].x << " y " << pose[i].y << endl;
    }
    showPath(graph, opti_path, printPoints);

    return true;
  }
} // namespace student
