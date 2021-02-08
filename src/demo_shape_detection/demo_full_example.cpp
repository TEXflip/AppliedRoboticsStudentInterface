// full_example.cpp:
// Detect shapes on a camera-captured image

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <iostream>
#include "utils.hpp"
#include <vector>
//#include <tesseract/baseapi.h> // Tesseractheaders
///#include <leptonica/allheaders.h>

static const int W_0 = 300;
static const int H_0 = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;

const double MIN_AREA_SIZE = 30;

float *boundingBoxcaracteristics(Polygon &x)
{
  //create bounding  Box around Polygon x
  float *bound_x = new float[4];
  int max = 0;
  int min = 0;

  //select min / max  from Polygon

  //max
  for (int i = 0; i < x.size(); i++)
  {
    if (x[i].x > max)
    {
      max = x[i].x;
    }
  }

  //min
  min = max;
  for (int i = 0; i < x.size(); i++)
  {
    if (x[i].x < min)
    {
      min = x[i].x;
    }
  }
  bound_x[0] = min; //min_x

  //determine width
  bound_x[2] = max - min;

  //select min / max  y from Polygon
  //max
  max = 0;
  for (int i = 0; i < x.size(); i++)
  {
    if (x[i].y > max)
    {
      max = x[i].y;
    }
  }

  //min
  min = max;
  for (int i = 0; i < x.size(); i++)
  {
    if (x[i].y < min)
    {
      min = x[i].y;
    }
  }

  bound_x[1] = min;

  //determine height
  bound_x[3] = max - min;

  return bound_x;
}

bool collide(Polygon a, Polygon b)
{

  /*
    broad fase implementaion
    1.Load the two desired Polygons in here
    2.Create a bounding box around them and extract their caracteristics (boundingBoxcaracteristics() )
        a.boundingBoxcaracteristics()
            1.select the highest and lowest x value of the Point vector of the Polygon
            2.calculate the width of the bounding box
            3.select the highest and lowest y value of the Point vector of the Polygon
            4.calculate the height of the bounding box
    3.Run both bounding boxcahracteristics throug the controll if() 
    
    */
  bool collided = false;

  float *bound_a = boundingBoxcaracteristics(a);
  float *bound_b = boundingBoxcaracteristics(b);

  //std::cout << "minx a: " << bound_a[0] << std::endl;
  //std::cout << "miny a:" << bound_a[1] << std::endl;
  //std::cout << " width a: " << bound_a[2] << std::endl;
  //std::cout << " height a: " << bound_a[3] << std::endl;

  //std::cout << "minx b: " << bound_b[0] << std::endl;
  //std::cout << "miny b:" << bound_b[1] << std::endl;
  //std::cout << " width b: " << bound_b[2] << std::endl;
  //std::cout << " height b: " << bound_b[3] << std::endl;

  // Colition detection of the two bounding boxes
  if (bound_a[0] < bound_b[0] + bound_b[2] &&
      bound_a[0] + bound_a[2] > bound_b[0] &&
      bound_a[1] < bound_b[1] + bound_b[3] &&
      bound_a[1] + bound_a[3] > bound_b[1])
  {
    collided = true;
  }

  //////////////////////// implement narrow phase////////////////////////////////////7
  if (collided == true)
  {
    Polygon *poly1 = &a;
    Polygon *poly2 = &b;

    float overlap = INFINITY;

    for (int shape = 0; shape < 2; shape++)
    {
      if (shape == 1)
      {
        poly1 = &a;
        poly2 = &b;
      }

      for (int a = 0; a < poly1->size(); a++)
      {
        int b = (a + 1) % poly1->size();
        Point axisProj = {-((*poly1)[b].y - (*poly1)[a].y), (*poly1)[b].x - (*poly1)[a].x};
        // Optional normalisation of projection axis enhances stability slightly
        //float d = sqrtf(axisProj.x * axisProj.x + axisProj.y * axisProj.y);
        //axisProj = { axisProj.x / d, axisProj.y / d };

        // Work out min and max 1D points for poly1
        float min_poly1 = INFINITY, max_poly1 = -INFINITY;
        for (int p = 0; p < poly1->size(); p++)
        {
          float q = ((*poly1)[p].x * axisProj.x + (*poly1)[p].y * axisProj.y);
          min_poly1 = std::min(min_poly1, q);
          max_poly1 = std::max(max_poly1, q);
        }

        // Work out min and max 1D points for b
        float min_poly2 = INFINITY, max_poly2 = -INFINITY;
        for (int p = 0; p < poly2->size(); p++)
        {
          float q = ((*poly2)[p].x * axisProj.x + (*poly2)[p].y * axisProj.y);
          min_poly2 = std::min(min_poly2, q);
          max_poly2 = std::max(max_poly2, q);
        }

        // Calculate actual overlap along projected axis, and store the minimum
        overlap = std::min(std::min(max_poly1, max_poly2) - std::max(min_poly1, min_poly2), overlap);

        if (!(max_poly2 >= min_poly1 && max_poly1 >= min_poly2))
          return false;
      }
    }
  }

  return collided;
}

//////////////////////////////////////////////////////////////////////////////////////////////////

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
bool onSegment(Point p, Point q, Point r)
{
  if (q.x <= std ::max(p.x, r.x) && q.x >= std ::min(p.x, r.x) &&
      q.y <= std ::max(p.y, r.y) && q.y >= std ::min(p.y, r.y))
    return true;
  return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
int orientation(Point p, Point q, Point r)
{
  int val = (q.y - p.y) * (r.x - q.x) -
            (q.x - p.x) * (r.y - q.y);

  if (val == 0)
    return 0;               // colinear
  return (val > 0) ? 1 : 2; // clock or counterclock wise
}

// The function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
bool doIntersect(Point p1, Point q1, Point p2, Point q2)
{
  // Find the four orientations needed for general and
  // special cases
  int o1 = orientation(p1, q1, p2);
  int o2 = orientation(p1, q1, q2);
  int o3 = orientation(p2, q2, p1);
  int o4 = orientation(p2, q2, q1);

  // General case
  if (o1 != o2 && o3 != o4)
    return true;

  // Special Cases
  // p1, q1 and p2 are colinear and p2 lies on segment p1q1
  if (o1 == 0 && onSegment(p1, p2, q1))
    return true;

  // p1, q1 and p2 are colinear and q2 lies on segment p1q1
  if (o2 == 0 && onSegment(p1, q2, q1))
    return true;

  // p2, q2 and p1 are colinear and p1 lies on segment p2q2
  if (o3 == 0 && onSegment(p2, p1, q2))
    return true;

  // p2, q2 and q1 are colinear and q1 lies on segment p2q2
  if (o4 == 0 && onSegment(p2, q1, q2))
    return true;

  return false; // Doesn't fall in any of the above cases
}

// Returns true if the point p lies inside the polygon[] with n vertices
bool isInside(Polygon &pol, int n, Point p)
{
  float inf = 100000;
  // There must be at least 3 vertices in polygon[]
  if (n < 3)
    return false;

  // Create a point for line segment from p to infinite
  Point extreme = {inf, p.y};

  // Count intersections of the above line with sides of polygon
  int count = 0, i = 0;
  do
  {
    int next = (i + 1) % n;

    // Check if the line segment from 'p' to 'extreme' intersects
    // with the line segment from 'polygon[i]' to 'polygon[next]'
    if (doIntersect(pol[i], pol[next], p, extreme))
    {
      // If the point 'p' is colinear with line segment 'i-next',
      // then check if it lies on segment. If it lies, return true,
      // otherwise false
      if (orientation(pol[i], p, pol[next]) == 0)
        return onSegment(pol[i], p, pol[next]);

      count++;
    }
    i = next;
  } while (i != 0);

  // Return true if count is odd, false otherwise
  return count % 2 == 1; // Same as (count%2 == 1)
}

bool isInside_Global(Point p, std::vector<Polygon> &obstacle_list)
{
  bool inside = false;
  for (int i = 0; i < obstacle_list.size(); i++)
  {
    Polygon obstacle = obstacle_list[i];
    int n = sizeof(obstacle) / sizeof(obstacle[1]);
    inside = isInside(obstacle, n, p);
    if (inside == true)
    {
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////////******************************************

void processImage()
{
  // Load image from file
  for (int j = 10; j <= 20; j++)
  {
    //char filename[50];
    //sprintf(filename, "camera_image_raw/0%d.jpg", j);
    std::string filename;
    filename = "imgs/000.jpg";
    // filename = "camera_image_raw/008.jpg";
    cv::Mat img = cv::imread(filename);

    if (img.empty())
    {
      throw std::runtime_error(filename);
    }

    // Display original image
    cv::imshow("Original", img);
    cv::moveWindow("Original", W_0, H_0);

    // Convert color space from BGR to HSV
    cv::Mat hsv_img;
    cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

    // Display HSV image
    ////cv::imshow("HSV", hsv_img);
    //::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

    // Preparing the kernel matrix
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));

    // Definining contour containers
    cv::Mat contours_img;
    std::vector<std::vector<cv::Point>> contours, contours_approx;
    std::vector<cv::Point> approx_curve;

    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
    cv::Mat red_mask_low, red_mask_high, red_mask;
    cv::inRange(hsv_img, cv::Scalar(0, 9, 69), cv::Scalar(20, 255, 255), red_mask_high);
    cv::inRange(hsv_img, cv::Scalar(158, 93, 33), cv::Scalar(180, 255, 253), red_mask_low);
    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask);

    // Filter (applying an erosion and dilation) the image
    cv::erode(red_mask, red_mask, kernel);
    cv::dilate(red_mask, red_mask, kernel);

    cv::imshow("RED_filter", red_mask);
    cv::moveWindow("RED_filter", W_0, H_0 + img.rows + OFFSET_H);

    // Process red mask
    std::vector<Polygon> obstacle_list;
    int scale = 1;
    contours_img = img.clone();
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    drawContours(contours_img, contours, -1, cv::Scalar(40, 190, 40), 1, cv::LINE_AA);
    ////std::cout << "N. contours: " << contours.size() << std::endl;
    for (int i = 0; i < contours.size(); ++i)
    {
      // //std::cout << (i + 1) << ") Contour size: " << contours[i].size() << std::endl;
      approxPolyDP(contours[i], approx_curve, 7, true);
      contours_approx = {approx_curve};
      drawContours(contours_img, contours_approx, -1, cv::Scalar(0, 170, 220), 3, cv::LINE_AA);
      ////std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
    }

    for (int i = 0; i < contours.size(); ++i)
    {
      approxPolyDP(contours[i], approx_curve, 3, true); // approxPolyDP( InputArray curve,OutputArray approxCurve,double epsilon, bool closed )

      //function that closes eventual opend contoures ???

      ////std::cout << "approxcurve size  " << approx_curve.size() << std::endl;
      //scaling loop
      Polygon scaled_contour; //typedev vector

      for (const auto &pt : approx_curve)
      {
        scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
      }

      obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list
      ////std::cout << "obsacel " << scaled_contour.size() << std::endl;
    }

    Polygon scaled_contour; //typedev vector
    scaled_contour.emplace_back(30, 40);
    scaled_contour.emplace_back(10, 60);
    scaled_contour.emplace_back(50, 60);

    obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list

    cv::imshow("Original", contours_img);
    cv::waitKey(0);

    for (int i = 0; i < obstacle_list.size(); i++)
    {
      ////std::cout << "obsacel " << i << " size: " << obstacle_list[i].size() << std::endl;
    }

    //////////////////////////////////////////////////////////////////////
    //process GREEN_VICTIMS AND GATE///////////////////////////////////////
    cv::Mat green_victim_mask;

    cv::inRange(hsv_img, cv::Scalar(40, 20, 50), cv::Scalar(79, 255, 255), green_victim_mask);

    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1 * 2) + 1, (1 * 2) + 1));
    cv::dilate(green_victim_mask, green_victim_mask, kernel);
    cv::erode(green_victim_mask, green_victim_mask, kernel);
    cv::imshow("Green", green_victim_mask);
    cv::moveWindow("Green", W_0, H_0 + img.rows + OFFSET_H);

    //find contours
    cv::findContours(green_victim_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //elaborating the found contours
    const double MIN_AREA_SIZE = 1000;
    cv::Mat contours_img_boundingbox = cv::Mat(img.size(), CV_8UC3, cv::Scalar::all(0));
    std::vector<cv::Rect> boundRect(contours.size());
    Polygon scaled_contour_green;
    std::vector<cv::Point> contours_approx_array[12];
    for (int i = 0; i < contours.size(); ++i)
    {
      double area = cv::contourArea(contours[i]); // check the contour area to remove false positives
      if (area < MIN_AREA_SIZE)
        continue;

      approxPolyDP(contours[i], approx_curve, 5, true);         //aproxximate the contoure in less vertices
      if (approx_curve.size() >= 4 && approx_curve.size() <= 6) //if i have a gate(a quadratic figure in green)
      {
        for (const auto &pt : approx_curve)
          scaled_contour_green.emplace_back(pt.x / scale, pt.y / scale);

        Polygon gate = scaled_contour_green;
      }
      else if (approx_curve.size() > 6) //  //if i have a number circle and not the gate
      {

        contours_approx = {approx_curve}; //assosciate the found aprox contour to the same number as the boundingbox (sync the arrays)
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0, 170, 220), 3, cv::LINE_AA);

        boundRect[i] = boundingRect(cv::Mat(approx_curve)); // finds bounding box for each green blob
      }
    }

    cv::imshow("Original", contours_img);

    ////////////////TEMPLATEMATCHING//////////////////
    cv::Mat green_mask_inv, filtered(img.rows, img.cols, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::bitwise_not(green_victim_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask

    // Load Templatenumbers into a vector
    std::vector<cv::Mat> templROIs;
    for (int i = 0; i <= 9; ++i)
    {
      cv::Mat templImg = cv::imread("/home/ubuntu/Desktop/workspace/project/src/01_template_matching/template/" + std::to_string(i) + ".png");
      if (templImg.empty())
        continue;

      // cv::flip(templImg, templImg, 1);
      templROIs.emplace_back(templImg);
    }

    img.copyTo(filtered, green_mask_inv); //creates a copy of image without green surounding

    double score;
    std::map<int, Polygon> victim_Map; //create a ma to sort the detected number with the position of the

    //loop for every numberBlob detected
    std::cout << "boundingbox size: " << boundRect.size() << std::endl;

    for (int i = 0; i < boundRect.size(); ++i)
    {
      cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit()
      cv::Mat processROIturned;
      cv::Mat processROIfiltered;
      if (processROI.empty())
        continue;

      cv::resize(processROI, processROI, cv::Size(196, 196)); // resize the ROI to match with the template size!!!!!
      cv::threshold(processROI, processROI, 50, 255, 0);      // threshold and binarize the image, to suppress some noise

      // Apply some additional smoothing and filtering
      kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((2 * 1) + 1, (2 * 1) + 1));
      cv::erode(processROI, processROIfiltered, kernel);
      cv::GaussianBlur(processROIfiltered, processROIfiltered, cv::Size(5, 5), 2, 2);
      cv::erode(processROIfiltered, processROIfiltered, kernel);

      cv::GaussianBlur(processROIfiltered, processROIfiltered, cv::Size(0, 0), 3);
      cv::addWeighted(processROIfiltered, 1.5, processROI, -0.5, 0, processROI);
      /*
*/
      cv::imshow("Orinal", processROIfiltered);
      cv::waitKey(0);
      ////////////////////////////////////TEmplate matching///////////////////////////////////
      // Find the template digit with the best matching
      cv::Mat templROIturned;
      cv::Mat result;
      double maxScore = 0;
      int maxIdx = -1;
      // std::cout << templROIs.size();

      for (int j = 0; j < templROIs.size(); ++j)
      {
        for (int r = 0; r < 360; r += 360)
        {
          //for (int off = -10; off <= 10; off += 2)
          //{

          cv::Point2f pc((templROIs[j].cols - 1) / 2., (templROIs[j].rows - 1) / 2.);
          cv::Mat rot = cv::getRotationMatrix2D(pc, r, 1.0);
          cv::warpAffine(templROIs[j], templROIturned, rot, templROIs[j].size());

          cv::Point minlock, maxloc;
          cv::matchTemplate(processROI, templROIturned, result, cv::TM_CCOEFF_NORMED);

          //std:: cout << "minlock " << minlock << "  maxlock "<< maxloc<<std::endl;
          if (score > maxScore)
          {
            maxScore = score;
            maxIdx = j;
          }
          //}
        }
      }

      Polygon scaled_contour; //typedev vector

      //scale the found contour

      std::cout << "Best fitting template Template matching: " << maxIdx << std::endl;
      //////////////////////////////////////////////Teseract/////////////////////////////////////////

      /*  tesseract::TessBaseAPI *ocr = new tesseract::TessBaseAPI();
    for (int r = 0; r < 90; r += 90)
    {
      cv::Point2f pc((processROI.cols - 1) / 2., (processROI.rows - 1) / 2.);
      cv::Mat rot = cv::getRotationMatrix2D(pc, r, 1.0);
      cv::warpAffine(processROI, processROIturned, rot, processROI.size());
      ocr->Init(NULL, "eng");                                    // Initialize tesseract to use English (eng)
      ocr->SetPageSegMode(tesseract::PSM_SINGLE_CHAR);           // Image containsa single character
      ocr->SetVariable("tessedit_char_whitelist", "0123456789"); // Only digits are valid output
      ocr->SetImage(processROIturned.data, processROIturned.cols, processROIturned.rows, 3, processROIturned.step);
      char *n = ocr->GetUTF8Text();
      ocr->End();

      std::cout << "Best fitting template OCR: " << *n << std::endl;
    }
  }
*/
      /*
    */
      ////////////////////
      /////////////////////////////////////////////////////////////////////////////////////////////////////////
      //Find blue regions
      cv::Mat blue_mask;
      cv::inRange(hsv_img, cv::Scalar(90, 50, 40), cv::Scalar(135, 255, 255), blue_mask);

      cv::imshow("BLUE_filter", blue_mask);
      cv::moveWindow("BLUE_filter", W_0 + img.cols + OFFSET_W, H_0 + img.rows + OFFSET_H);
      cv::waitKey(0);
      // Process blue mask
      contours_img = img.clone();
      cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
      drawContours(contours_img, contours, -1, cv::Scalar(40, 190, 40), 1, cv::LINE_AA);
      //    //std::cout << "N. contours: " << contours.size() << std::endl;
      for (int i = 0; i < contours.size(); ++i)
      {
        //std::cout << (i + 1) << ") Contour size: " << contours[i].size() << std::endl;
        double area = cv::contourArea(contours[i]);
        if (area < MIN_AREA_SIZE)
          continue; // filter too small contours to remove false positives
        approxPolyDP(contours[i], approx_curve, 7, true);
        contours_approx = {approx_curve};
        drawContours(contours_img, contours_approx, -1, cv::Scalar(0, 170, 220), 3, cv::LINE_AA);
        //std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
      }

      cv::imshow("Original", contours_img);
      cv::waitKey(0);
    }

    cv::waitKey(0);
  }
}

  int main()
  {
    processImage();
    return 0;
  }
