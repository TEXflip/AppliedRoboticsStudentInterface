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



static const int W_0      = 300;
static const int H_0      = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;

const double MIN_AREA_SIZE = 30;

float *boundingBoxcaracteristics (Polygon& x)
{
    //create bounding  Box around Polygon x
    float* bound_x = new float [4];
    int max=0;
    int min=0;


    //select min / max  from Polygon 
    
    //max
    for ( int i=0; i<x.size(); i++)
    {
        if(x[i].x > max )
        {
            max=x[i].x;
        }
     }
    
    //min
    min=max;
    for ( int i=0; i<x.size(); i++)
    {
        if(x[i].x < min )
        {
            min=x[i].x;
        }
     }
    bound_x[0] = min; //min_x
   
    //determine width
    bound_x[2] = max-min;

    //select min / max  y from Polygon
    //max
    max=0;
    for ( int i=0; i<x.size(); i++)
    {
        if(x[i].y > max )
        {
            max=x[i].y;
        }
     }
    
    //min
    min=max;
    for ( int i=0; i<x.size(); i++)
    {
        if(x[i].y < min )
        {
            min=x[i].y;
        }
     }
    
    bound_x[1] = min;
    
    //determine height
    bound_x[3]=max-min;
  



    return bound_x;     
}
  
bool collide( Polygon a,  Polygon b){
    
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
    
    
   float* bound_a =  boundingBoxcaracteristics(a);
   float* bound_b = boundingBoxcaracteristics(b);


    std::cout << "minx a: " << bound_a[0] << std::endl;
    std::cout << "miny a:" << bound_a [1]<< std::endl;
    std::cout << " width a: " << bound_a[2] << std::endl;
    std::cout << " height a: " << bound_a[3] << std::endl;
    
    std::cout << "minx b: " << bound_b[0] << std::endl;
    std::cout << "miny b:" << bound_b [1]<< std::endl;
    std::cout << " width b: " << bound_b[2] << std::endl;
    std::cout << " height b: " << bound_b[3] << std::endl;

 
    // Colition detection of the two bounding boxes        
    if(bound_a[0] < bound_b[0] + bound_b[2] &&
   bound_a[0] + bound_a[2] > bound_b[0] &&
   bound_a[1] < bound_b[1] + bound_b[3] &&
   bound_a[1] + bound_a[3] > bound_b[1])
   {
       collided = true; 
   }
    

    //////////////////////// implement narrow phase////////////////////////////////////7

  
   
   return collided;
}
void processImage()
{
  // Load image from file
  std::string filename = "imgs/img00.jpg";
  cv::Mat img = cv::imread(filename.c_str());

  if(img.empty()) {
    throw std::runtime_error("Failed to open the file " + filename);
  }

  // Display original image
  cv::imshow("Original", img);
  cv::moveWindow("Original", W_0, H_0);

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);

  // Display HSV image
  cv::imshow("HSV", hsv_img);
  cv::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

  // Preparing the kernel matrix
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));

  // Definining contour containers
  cv::Mat contours_img;
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;

  // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])
  cv::Mat red_mask_low, red_mask_high, red_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), red_mask_low);
  cv::inRange(hsv_img, cv::Scalar(160, 50, 50), cv::Scalar(179, 255, 255), red_mask_high);
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks

  // Filter (applying an erosion and dilation) the image
  cv::erode(red_mask, red_mask, kernel);
  cv::dilate(red_mask, red_mask, kernel);
 
  cv::imshow("RED_filter", red_mask);
  cv::moveWindow("RED_filter", W_0, H_0+img.rows+OFFSET_H);

  // Process red mask
  std::vector<Polygon> obstacle_list;
  int scale=1;
  contours_img = img.clone();
  cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 7, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
 
 for (int i = 0; i < contours.size(); ++i)
    {

      approxPolyDP(contours[i], approx_curve, 3, true); // approxPolyDP( InputArray curve,OutputArray approxCurve,double epsilon, bool closed )
                                                        //function that closes eventual opend contoures ???

      //scaling loop
      Polygon scaled_contour; //typedev vector

      for (const auto &pt : approx_curve)
      {
        scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
      }

      obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list
    }
 
Polygon a = obstacle_list[0];

cv::waitKey(0);


 
  cv::imshow("Original", contours_img);


  // Find blue regions
  cv::Mat blue_mask;
  cv::inRange(hsv_img, cv::Scalar(90, 50, 40), cv::Scalar(135, 255, 255), blue_mask);

 




  cv::imshow("BLUE_filter", blue_mask);
  cv::moveWindow("BLUE_filter", W_0+img.cols+OFFSET_W, H_0+img.rows+OFFSET_H);
  cv::waitKey(0);
  // Process blue mask
  contours_img = img.clone();
  cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    double area = cv::contourArea(contours[i]);
    if (area < MIN_AREA_SIZE) continue; // filter too small contours to remove false positives
    approxPolyDP(contours[i], approx_curve, 7, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
      for (int i = 0; i < contours.size(); ++i)
    {

      approxPolyDP(contours[i], approx_curve, 3, true); // approxPolyDP( InputArray curve,OutputArray approxCurve,double epsilon, bool closed )
                                                        //function that closes eventual opend contoures ???

      //scaling loop
      Polygon scaled_contour; //typedev vector

      for (const auto &pt : approx_curve)
      {
        scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
      }

      obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list
    }
 std:: cout << "obstacle_list " << obstacle_list.size()<<std::endl;

 for(int i= 0; i<obstacle_list.size();i++){
   std:: cout << "obstacle_listing " << obstacle_list[i].size()<<std::endl;

 }

Polygon b = obstacle_list[3];

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
std::cout <<"Collition detection "<<collide(a,b)<< std::endl;
 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  cv::imshow("Original", contours_img);
  cv::waitKey(0);
/*

  // Find black regions
  cv::Mat black_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 40, 40), black_mask);
  
  // Filter (applying dilation, blurring, dilation and erosion) the image
  kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((4*2) + 1, (4*2)+1));
  cv::dilate(black_mask, black_mask, kernel);
  cv::GaussianBlur(black_mask, black_mask, cv::Size(5, 5), 2, 2);
  cv::dilate(black_mask, black_mask, kernel);
  cv::erode(black_mask, black_mask, kernel);
  
  cv::imshow("BLACK_filter", black_mask);
  cv::moveWindow("BLACK_filter", W_0+2*(img.cols+OFFSET_W), H_0);

  // Process black mask
  contours_img = img.clone();
  cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 7, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  cv::imshow("Original", contours_img);
  
   int scale =1;
   std::vector<Polygon> obstacle_list;
  Polygon scaled_contour;


  for (const auto &pt : approx_curve)
      {
        scaled_contour.emplace_back(pt.x / scale, pt.y / scale);
      }

      obstacle_list.push_back(scaled_contour);
    std :: cout <<"obstacle_list " <<obstacle_list.size()<<" scaled_contour.size "<< scaled_contour.size()<< std::endl;
      
      std::cout << scaled_contour[1].x << std::endl;


  
  cv::waitKey(0);*/

}

int main()
{
  processImage();
  return 0;
}
