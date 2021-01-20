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
    if(collided==true)
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
                Point axisProj = { -((*poly1)[b].y - (*poly1)[a].y), (*poly1)[b].x - (*poly1)[a].x };				
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



