
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

void processImage(cv::Mat& img_in)
{
    //cv:: Rect rect ()

    
}
int main(int argc, char* argv[])
{
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " <image>" << std::endl;
    return 0;
  }
  //processImage(argv[1]);


  return 0;
}
