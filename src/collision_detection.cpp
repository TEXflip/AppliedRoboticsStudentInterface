#include "collision_detection.hpp"


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
/*

    std::cout << "minx a: " << bound_a[0] << std::endl;
    std::cout << "miny a:" << bound_a [1]<< std::endl;
    std::cout << " width a: " << bound_a[2] << std::endl;
    std::cout << " height a: " << bound_a[3] << std::endl;
    
    std::cout << "minx b: " << bound_b[0] << std::endl;
    std::cout << "miny b:" << bound_b [1]<< std::endl;
    std::cout << " width b: " << bound_b[2] << std::endl;
    std::cout << " height b: " << bound_b[3] << std::endl;
*/
 
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