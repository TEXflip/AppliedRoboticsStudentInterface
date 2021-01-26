#include "graph.hpp"
#include "collision_detection.hpp"
#include <map>



void connectRobot(Graph::Graph &graph,const float x, const float y, const float xgate,float ygate, const std::vector<Polygon> &obstacle_list)
{
    int myConnection;
    int closest;
    int secondclosest;
    float distance;
    float shortesuntilnow=1000;
    std::multimap<float,int> dist;
    float dx;
    float dy;

    for (int i =0; i< graph.size(); i++)
    {   
        dx=graph[i].x-x;
        dy=graph[i].y-y;
        distance = (dx*dx)+(dy*dy);
        dist.insert(std::pair<float,int>(distance,i));
    }
   

/*
   //check the closest point to gate in a epsilon to the robot
   float epsilon = 0.1*0.1; //radius squared, becasue the distance isalso radius squared
    std::vector<std::multimap<float,int> > togate;
    
    std::multimap<float, int>::const_iterator it = dist.begin();
    while(it != dist.end() && it->first < epsilon)
    {
        //togate.insert(it);

    }
        
    //check distance to gate

  */
 
//check if the connection doesnt collide with any obstacle

std::multimap<float, int>::const_iterator it = dist.begin();
bool collition=true;
while(it != dist.end() && collition==true)
{
    //create segment from robot to connection node
    Point p1,p2;
    p1.x=graph[it->second].x;
    p1.y=graph[it->second].y;
    p2.x=x;
    p2.y=y;
    //check if new connection intersects a polygon
    collition = intersect_Global(p1,p2,obstacle_list);
    if(collition==false)
    {

        myConnection = it->second; 
    }
    it++;
}



    // insert the ropot pos in the graph
    Graph::node robot;
    robot.x=x;
    robot.y=y;
    //indiriyyo del graph?
    robot.neighbours.emplace_back(myConnection);
    graph.emplace_back(robot);
    graph[myConnection].neighbours.emplace_back(graph.size()-1);
}


void connectVictim(Graph::Graph &graph,const float x, const float y, 
const float xgate,float ygate, const std::vector<Polygon> &obstacle_list,const std::vector<std::pair<int, Polygon>> &victim_list)
{
    int myConnectionIn;
    int myConnectionOUT;
    float distance;
    std::multimap<float,int> dist;
    float dx;
    float dy;
    int nr_nodes_check=10;
    
    Polygon victim;
    std::vector<std::pair<int, Polygon> >::const_iterator itv = victim_list.begin();
    Point pv;
    float xv = 0, yv = 0, xvictim, yvictim;

    
    ///calculate center of victim for everz victim and put it in a vector eventually
    std::vector<std::pair<float, float>> victims_center;
    
    itv=victim_list.begin();
    
        victim = itv->second;
        for (int i = 0; i < victim.size(); i++)
    {
         xv += victim[i].x;
      yv += victim[i].y;
    }
        xvictim = xv / victim.size();
        yvictim = yv / victim.size();
        victims_center.push_back(std::pair<float, float>(xvictim, yvictim));
    
 
//check distance from victim to node
   for (int i =0; i< graph.size(); i++)
    {   
        dx=graph[i].x-xvictim;
        dy=graph[i].y-yvictim;
        distance = (dx*dx)+(dy*dy);
        dist.insert(std::pair<float,int>(distance,i));
    }


    std::multimap<float, int>::const_iterator it = dist.begin();
    int node_count=0;
   
 

//check if the connection of the 10 nodes collides with an obstacle. no?-> connect ot graph
    bool collision = true;
    Point p1,p2;
    Graph::node n_victim;
    n_victim.x=xvictim;
    n_victim.y=yvictim;
    graph.emplace_back(n_victim);



    while(node_count<nr_nodes_check && it!=dist.end())
    {   
     p1.x=graph[it->second].x;
     p1.y=graph[it->second].y;
     p2.x=xvictim;
     p2.y=yvictim;
    //check if new connection intersects a polygon
    collision = intersect_Global(p1,p2,obstacle_list);
    if(collision==false)
    {
        //connect victim to graph
        myConnectionIn = it->second; 
        n_victim.neighbours.emplace_back(myConnectionIn);
        graph[myConnectionIn].neighbours.emplace_back(graph.size()-1);
    }
   
    }

}