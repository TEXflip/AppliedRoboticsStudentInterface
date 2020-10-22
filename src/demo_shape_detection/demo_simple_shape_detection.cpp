// shape_detection.cpp:
// Detect shapes on a computer-generated image
//optimized for square_arena_full_example

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>


#include <stdexcept>
#include <sstream>

#include <vector>
#include <atomic>
#include <unistd.h>

#include <experimental/filesystem>
#include <sstream>

static const int W_0      = 300;
static const int H_0      = 0;
static const int OFFSET_W = 10;
static const int OFFSET_H = 100;

 bool findRobot(const cv::Mat& hsv_img){

 double x,  y, theta;
 
//////////////////////////////////////////////////
struct Pose 
{
  float s, x, y, theta, kappa;

  Pose(float s, float x, float y, float theta, float kappa):
    s(s), x(x), y(y), theta(theta), kappa(kappa)
  {}

  Pose(): 
    Pose(0, 0, 0, 0, 0)
  {}

  float distance(float _x, float _y)
  {
    return std::hypot(x-_x, y-_y);
  }
};

// A sequence of sampled robot configurations composing a (discretization of the) path
int scale = 1;
struct Path 
{
  std::vector<Pose> points;
  
  Path(std::vector<Pose> const & points):
    points(points)
  {}

  Path()
  {}
  
  bool empty() { return points.empty(); }
  size_t size() { return points.size(); }
  void setPoints(const std::vector<Pose>& points) { this->points = points; }
};

struct Point 
{
  float x, y;

  Point(float x, float y):
    x(x), y(y)
  {}

  Point(): 
    Point(0, 0)
  {}

};
typedef std::vector<Point> Polygon;
Polygon triangle;
////////////////////////////////////////////

/*
    1.filter the blue areas out of the hsv image
    2.apply some filtering
    3. analyse the 
*/

cv::Mat blue_mask;    
     
  cv::inRange(hsv_img, cv::Scalar(90, 50, 50), cv::Scalar(140, 255, 255), blue_mask);
  // Process blue mask
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  bool found = false;
 for (int i=0; i<contours.size() && found == false; ++i)
    {
      
      cv::approxPolyDP(contours[i], approx_curve, 10, true);
      contours_approx = {approx_curve};


      double area = cv::contourArea(approx_curve);
      // use onlz triangles
      if (approx_curve.size() != 3) continue;
      
      if (area < 300 || area>3000) continue;
      
      found = true;
    }
  //find the barrycentre and rotation
    if (found) 
    {
      //scale the contoure
      for (const auto& pt: approx_curve) {
        triangle.emplace_back(pt.x/scale, pt.y/scale);
      }
      //calculate the barrycentre
      double cx = 0, cy = 0;
      for (auto item: triangle) 
      {
        cx += item.x;
        cy += item.y;
      }
      cx /= triangle.size();
      cy /= triangle.size();

      double dst = 0;
      Point vertex;
      for (auto& item: triangle)
      {
        double dx = item.x-cx;      
        double dy = item.y-cy;
        double curr_d = dx*dx + dy*dy;
        if (curr_d > dst)
        { 
          dst = curr_d;
          vertex = item;
        }
      }











}}







void processMap(const cv:: Mat& img_in)
{
  std::cout << "Loading the templates Failed"<< std::endl;
   /* we need a obstacle list
        0. convert imput image in hsv colorspace ::  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
        1. use a colorfilter to sort the objects in a mask (g,R,B,Purple); ::quite a bit of code
        2. filter the objects if necessary (molto probabile)
        3.  extract contour and the centre of the obstacles (put them in the assigned lists)
        4. order the victims and put them in the list
        5. detect the gate by countuing the edges on the green contour mask
        
    */

//§§§§§§§§§§§§§§§include not necessarz for final file§§§§§§§§§§§§§§§§§§§§§§
#include <vector>
#include <cmath>
#include <cstddef>


// A configuration of the robot along the path, represented by x, y, orientation and curvature
struct Pose 
{
  float s, x, y, theta, kappa;

  Pose(float s, float x, float y, float theta, float kappa):
    s(s), x(x), y(y), theta(theta), kappa(kappa)
  {}

  Pose(): 
    Pose(0, 0, 0, 0, 0)
  {}

  float distance(float _x, float _y)
  {
    return std::hypot(x-_x, y-_y);
  }
};

// A sequence of sampled robot configurations composing a (discretization of the) path
int scale = 1;
struct Path 
{
  std::vector<Pose> points;
  
  Path(std::vector<Pose> const & points):
    points(points)
  {}

  Path()
  {}
  
  bool empty() { return points.empty(); }
  size_t size() { return points.size(); }
  void setPoints(const std::vector<Pose>& points) { this->points = points; }
};

struct Point 
{
  float x, y;

  Point(float x, float y):
    x(x), y(y)
  {}

  Point(): 
    Point(0, 0)
  {}

};


typedef std::vector<Point> Polygon;
//§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§ù
   
   
   //colorspace convertion BGR->HSV

    cv::Mat hsv_img;
    cv::cvtColor(img_in, hsv_img, cv::COLOR_BGR2HSV);

    //Mats for the colormasks
    cv::Mat red_mask_high,red_mask_low, red_obstacle_mask, green_victim_mask, black_border_mask, purple_gate_mask;
    /*
        values for masks (H_low,S_low,V_low) (H_hight,S_hight,V_hight)
        RED     (0,10,0) (10,255,255)
        Green   (10,0,0) (110,255,255)
        Black   (0,0,0)   (0-180..doesnt mater,1-180,225) !! the V value is important to be at 224

    */
    //filtering the red_obstacles
    cv::inRange(hsv_img, cv::Scalar(0, 30, 113), cv::Scalar(10, 255, 218), red_mask_high);
    // cv::imwrite("/home/ubuntu/Desktop/Redmask.jpg", red_mask_high);
    //for real images use hue values left and right from 0 in order to get the best result

    cv::inRange(hsv_img, cv::Scalar(142, 29, 199), cv::Scalar(180, 255, 255), red_mask_low);

    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_obstacle_mask);

    
    //selecting the green_victims AND the gate
        
    cv::inRange(hsv_img, cv::Scalar(52,12,151), cv::Scalar(82,255,255), green_victim_mask); 
    cv::imwrite("1aaaaa.jpg",red_obstacle_mask);
    //filter the black border
    //cv::inRange(hsv_img, cv::Scalar(0,0,0), cv::Scalar(10,10,225), black_border_mask); 
    

    /*TESTING ONLY)*/

      cv::imshow("red_obstacle_mask", red_obstacle_mask);
      
      cv::moveWindow("red_obstacle_mask", W_0+2*(img_in.cols+OFFSET_W), H_0);
      cv::imshow("green_victim_mask", green_victim_mask);
      cv::moveWindow("green_victim_mask", W_0-2*(img_in.cols+OFFSET_W), H_0);
     // cv::imshow("black_border_mask", black_border_mask);
     // cv::moveWindow("black_border_mask", W_0+2*(img_in.cols+OFFSET_W), H_0);
      std::vector<Polygon> obstacle_list;

    //process obstacles RE///////////////////////////////////////////////////
    //find contours
    
    std::vector<std::vector<cv::Point>> contours, contours_approx; //define point vectros for the curves and contoures
    std::vector<cv::Point> approx_curve;
    
    //apply filter

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
  cv::dilate(red_obstacle_mask, red_obstacle_mask, kernel);
  cv::erode(red_obstacle_mask, red_obstacle_mask, kernel);
 

    cv::Mat contours_img_aproxy = cv::Mat (img_in.size(), CV_8UC3, cv::Scalar::all(0));
    cv::Mat contours_img_raw_red = cv::Mat (img_in.size(), CV_8UC3, cv::Scalar::all(0));
    
    //Red Obstacle contours
    cv::findContours(red_obstacle_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
     
     
   for (int i=0; i<contours.size(); ++i)
  {
    //std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;


    approxPolyDP(contours[i], approx_curve, 3, true);   // approxPolyDP( InputArray curve,OutputArray approxCurve,double epsilon, bool closed )
                                                        //function that closes eventual opend contoures ???
   
    //scaling and inserting in list loop

    Polygon scaled_contour;         //typedev vector
    for (const auto& pt: approx_curve) 
      {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }


    obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list


    /*Testing*/
    contours_approx = {approx_curve};                   //adding the single contour to the whole image ???
    drawContours(contours_img_aproxy, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
   
   
  }
    //delete this
    drawContours(contours_img_raw_red, contours, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);


    cv::imshow("contours_img_aproxy_red", contours_img_aproxy);
    cv::moveWindow("contours_img_aproxy_red", W_0+2*(img_in.cols+OFFSET_W), H_0+(img_in.rows+OFFSET_H));

    //cv::imshow("contours_img_raw_red", contours_img_raw_red);

  std::cout<<"ostcoli rossi: " << obstacle_list.size() << std::endl;

//GREEN VICTiMS §§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§§ù

    // eventual filtering on green_victim_mask
    kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((1*2) + 1, (1*2)+1));
    cv::dilate(green_victim_mask, green_victim_mask, kernel);
    cv::erode(green_victim_mask, green_victim_mask, kernel);

  // TESTING cv::imshow("afterFilter_green", green_victim_mask);
  
  cv::Mat contours_img_raw_green = cv::Mat (img_in.size(), CV_8UC3, cv::Scalar::all(0));
  
  // Find contours
  cv::findContours(green_victim_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  
  drawContours(contours_img_raw_green, contours, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
  cv::imshow("green_contours", contours_img_raw_green);//*************************
  cv::moveWindow("green_contours", W_0,H_0+img_in.rows);

  //elaborating the found contours
  const double MIN_AREA_SIZE = 100;
  cv::Mat contours_img_boundingbox = cv::Mat (img_in.size(), CV_8UC3, cv::Scalar::all(0));
  cv::Mat gateimage = cv::Mat (img_in.size(), CV_8UC3, cv::Scalar::all(0));
  std::vector<cv::Rect> boundRect(contours.size());
  Polygon scaled_contour_green; 
   std::vector<std::vector<cv::Point>>contours_approx_green,contours_approx_green_test_only;
   std::vector<std::vector<cv::Point>>contours_approx_gate;
   std::vector<cv::Point> contours_approx_array[12];





  for (int i=0; i<contours.size(); ++i)
  {
    double area = cv::contourArea(contours[i]); // check the contour area to remove false positives
    if (area < MIN_AREA_SIZE) continue; 
    
    approxPolyDP(contours[i], approx_curve, 11, true);
    
   
       
    if(approx_curve.size()==4) //if i have a gate
    {
      Polygon gate;
       //scale
    for (const auto& pt: approx_curve) 
     {
       scaled_contour_green.emplace_back(pt.x/scale, pt.y/scale);
     }
      gate = scaled_contour_green;
      contours_approx_gate={approx_curve};
      drawContours(gateimage, contours_approx_gate, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    }
    else if(approx_curve.size()>4) // && approx_curve.size()<  as in example?
    {
      contours_approx_green = {approx_curve}; //NOT SCALED!!
      contours_approx_array [i] = {approx_curve};
      boundRect[i] = boundingRect(cv::Mat(approx_curve)); // finds bounding box for each green blob
    }

   drawContours(contours_img_boundingbox, contours_approx_green, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
  
  }
  
  ////Testkit
  /*
  for(int i = 0; i < boundRect.size(); i++)
  {
    contours_approx_green_test_only=contours_approx_array[i];
    drawContours(contours_img_boundingbox, contours_approx_green_test_only, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);

  }
*/
  cv::imshow("Boundingboxes", contours_img_boundingbox);
  cv::moveWindow("Boundingboxes", W_0,H_0+100);

  cv::imshow("gate", gateimage);
  cv::moveWindow("gate", W_0+img_in.cols+OFFSET_W,H_0);
  ///////

  /////////////////TEMLATEMATCHING////////////  
  cv::Mat green_mask_inv, filtered(img_in.rows, img_in.cols, CV_8UC3, cv::Scalar(255,255,255));
  cv::bitwise_not(green_victim_mask, green_mask_inv); // generate binary mask with inverted pixels w.r.t. green mask -> black numbers are part of this mask
  
  //cv::imshow("green_mask_inv", green_mask_inv);//**********************
  //
  // Load Templatenumbers into a vector
    std::vector<cv::Mat> templROIs;
    for (int i=0; i<=9; ++i) 
  {
    templROIs.emplace_back(cv::imread("/home/ubuntu/Desktop/workspace/project/src/01_template_matching/template/" + std::to_string(i) + ".png"));
  }

  //////TestKit
   if(templROIs[1].empty()) std::cout << "Loading the templates Failed"<< std::endl;
  //cv::imshow("Template_image",templROIs[1]);
  //////  
  
  // cv::imshow("green_mask_inv", green_mask_inv);
   //ma come??
  img_in.copyTo(filtered, green_mask_inv); //creates a copy of image without green surounding 
  cv::imshow("filtered", filtered);
  cv::moveWindow("filtered", W_0+img_in.cols+OFFSET_W, H_0+img_in.rows+OFFSET_H);


  // cv::imshow("green_mask_in2v", green_mask_inv);


   //number detection
   /*
        1. Mask green objects
        2. Filter them
        3. get contours
        4. aprox them -> generate boundingbox
        5. create and invert the obtained bitmap 
        6. load templates from file
        7. create copy of the immage without green surounding(filtered is a mat with all black and therfore the numbers remain hen copzing the bitmap
        8. for everz boundingbox in boundRect 
            - resize region of interst with the template size
        
   */
    std::vector<std::pair<int, Polygon>> victim_list;
  
      double score;
  //loop for every numberBlob detected

  std::map < int , Polygon> victim_Map;

  for (int i=0; i<boundRect.size(); ++i)
  {
    cv::Mat processROI(filtered, boundRect[i]); // extract the ROI containing the digit()
        
    //cv::imshow("processROI(filtered, boundRect[i])", processROI);  
    //cv::moveWindow("ROI", W_0+img_in.cols+OFFSET_W, H_0+img_in.rows+OFFSET_H-200);
    if (processROI.empty()) continue;

    cv::resize(processROI, processROI, cv::Size(200, 200)); // resize the ROI to match with the template size!!!!!
    cv::threshold( processROI, processROI, 100, 255, 0 ); // threshold and binarize the image, to suppress some noise
    
    // Apply some additional smoothing and filtering
    cv::erode(processROI, processROI, kernel);
    cv::GaussianBlur(processROI, processROI, cv::Size(5, 5), 2, 2);
    cv::erode(processROI, processROI, kernel);
    
    // Show the actual image used for the template matching
    cv::imshow("ROI", processROI);  
    cv::moveWindow("ROI", W_0+img_in.cols+OFFSET_W, H_0+img_in.rows+OFFSET_H);

    
    
    // Find the template digit with the best matching
    double maxScore = 0;
    int maxIdx = -1;
    std::cout << templROIs.size();

    for (int j=0; j<templROIs.size(); ++j) 
    {
      cv::Mat result;
      cv::matchTemplate(processROI, templROIs[j], result, cv::TM_CCOEFF);
     
      cv::minMaxLoc(result, nullptr, &score); 
      if (score > maxScore) 
      {
        maxScore = score;
        maxIdx = j;
      }
    
    }

    /////NON MI VA IL SCALING !!!!!
    Polygon scaled_contour;         //typedev vector
    

    for (const auto& pt: contours_approx_array[i]) 
      {
        scaled_contour.emplace_back(pt.x/scale, pt.y/scale);
      }

    
    victim_Map.insert( std::pair<int,Polygon>(maxIdx,scaled_contour));

   // obstacle_list.push_back(scaled_contour); //add the aprox and scaled object to the list

        //save the boundRect in the associate pos of the   vector <pair <int, Polygon> victim_list
    
    //victim_list[maxIdx]=std::pair<const maxIdx,boundRect[i]>;
        std::cout << "Best fitting template: " << maxIdx << std::endl;
    
    cv::waitKey(0);
  }

        cv::Mat templROIturned;
        cv::Mat result;
        cv::Point2f pc(templROIs[1].cols/2., templROIs[1].rows/2.);
        cv::Mat rot = cv::getRotationMatrix2D(pc, 45, 1.0);
        cv::warpAffine(templROIs[1], templROIturned, rot, cv::Size(200, 200));


  victim_list.assign(victim_Map.begin(), victim_Map.end() );

  ///TESTKIT

  std:: cout << "Vectorsize(" << victim_list.size() << ")" << std:: endl;


    cv::waitKey(0);


}































void processImage(cv::Mat& img)
{
  // Display original image
  cv::imshow("Original", img);
  cv::moveWindow("Original", W_0, H_0);

  // Convert color space from BGR to HSV
  cv::Mat hsv_img;
  cv::cvtColor(img, hsv_img, cv::COLOR_BGR2HSV);
  
  // Display HSV image
  cv::imshow("HSV", hsv_img);
  cv::moveWindow("HSV", W_0+img.cols+OFFSET_W, H_0);

/*
  // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])      171,180;	96,255;		113,218;
  cv::Mat red_mask_low, red_mask_high, red_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 10, 10), cv::Scalar(15, 255, 255), red_mask_low);
  cv::inRange(hsv_img, cv::Scalar(160, 10, 10), cv::Scalar(179, 255, 255), red_mask_high);
  cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_mask); // combine together the two binary masks
  cv::imshow("RED_filter", red_mask);
  cv::moveWindow("RED_filter", W_0, H_0+img.rows+OFFSET_H);
*/

      cv::Mat red_mask_high, red_mask_low, red_obstacle_mask, green_victim_mask, black_border_mask;


    // Find red regions: h values around 0 (positive and negative angle: [0,15] U [160,179])        171,180;	96,255;		113,218;
  // cv::imwrite("/home/ubuntu/Desktop/Redmask.jpg", red_mask_high);
    //for real images use hue values left and right from 0 in order to get the best result
    //selecting the red_obstacles
    //selecting the red_obstacles
   
   
    cv::inRange(hsv_img, cv::Scalar(0, 30, 188), cv::Scalar(10, 255, 255), red_mask_high);
    // cv::imwrite("/home/ubuntu/Desktop/Redmask.jpg", red_mask_high);
    //for real images use hue values left and right from 0 in order to get the best result

    cv::inRange(hsv_img, cv::Scalar(142, 29, 199), cv::Scalar(180, 255, 255), red_mask_low);

    cv::addWeighted(red_mask_low, 1.0, red_mask_high, 1.0, 0.0, red_obstacle_mask);

    
    //selecting the green_victims AND the gate
        
    cv::inRange(hsv_img, cv::Scalar(52,12,151), cv::Scalar(82,255,255), green_victim_mask);
    // cv::imshow("input",green_victim_mask);
    
  cv::Mat blue_mask;
  cv::inRange(hsv_img, cv::Scalar(106, 75, 38), cv::Scalar(125,202,122), blue_mask); //106,125;	75,202;		38,122;
  cv::imshow("BLUE_filter", blue_mask);
  cv::moveWindow("BLUE_filter", W_0+img.cols+OFFSET_W, H_0+img.rows+OFFSET_H);
  
  // Find green regions
  cv::Mat green_mask;
  cv::inRange(hsv_img, cv::Scalar(30,66,41), cv::Scalar(100,213, 100), green_mask); //HSV values got by try and error from demo_hvs_filter
  cv::imshow("GREEN_filter", green_mask);
  cv::moveWindow("GREEN_filter", W_0+2*(img.cols+OFFSET_W), H_0);                    

  // Find black regions (filter on saturation and value)
  cv::Mat black_mask;
  cv::inRange(hsv_img, cv::Scalar(0, 0, 0), cv::Scalar(180, 95, 65), black_mask);  0,180;		0,95;		0,65; 
  cv::imshow("BLACK_filter", black_mask);
  cv::moveWindow("BLACK_filter", W_0+2*(img.cols+OFFSET_W), H_0+img.rows+OFFSET_H);

  // Wait keypress
  cv::waitKey(0);

  // Find contours
  std::vector<std::vector<cv::Point>> contours, contours_approx;
  std::vector<cv::Point> approx_curve;
  cv::Mat contours_img;

  // Process black mask
  cv::findContours(black_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); // find external contours of each blob
  contours_img = img.clone();
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true); // fit a closed polygon (with less vertices) to the given contour,
                                                      // with an approximation accuracy (i.e. maximum distance between 
                                                      // the original and the approximated curve) of 3
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 5, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

  // Process red mask
  contours_img = img.clone();
  cv::findContours(red_obstacle_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

  // Process blue mask
  contours_img = img.clone();
  cv::findContours(blue_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);


//process GREEN mask (DK) written to enter in th emain program(repeat for each color) is there a contour funktion which approximates circles(green victim is alwazs a circle)
contours_img = img.clone(); //for tuhis example only
  cv::findContours(green_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  drawContours(contours_img, contours, -1, cv::Scalar(40,190,40), 1, cv::LINE_AA);
  std::cout << "N. contours: " << contours.size() << std::endl;
  for (int i=0; i<contours.size(); ++i)
  {
    std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
    approxPolyDP(contours[i], approx_curve, 3, true);
    contours_approx = {approx_curve};
    drawContours(contours_img, contours_approx, -1, cv::Scalar(0,170,220), 3, cv::LINE_AA);
    std::cout << "   Approximated contour size: " << approx_curve.size() << std::endl;
  }
  std::cout << std::endl;
  cv::imshow("Original", contours_img);
  cv::waitKey(0);

}
/*
*/


int main( int, char** argv )
{
  /// Load an image
  cv::Mat img = cv::imread( argv[1] );

  if( !img.data ){
    printf(" Error opening image\n");
    printf(" Usage: ./demo_simple_shape_detection [image_name] \n"); 
    return -1; 
  }
    
  //processImage(img);
  processMap(img);
 // findRobot(img);
  return 0;
}



