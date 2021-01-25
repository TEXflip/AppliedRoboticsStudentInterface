#include "graph.hpp"
#include <map>



void closestPoint(Graph::Graph &graph,const float x, const float y, const float xgate,float ygate, const std::vector<Polygon> &obstacle_list)
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
    
    //colitionFree = doIntersectPolzgons()
    if(collition==true)
    {
        myConnection = it->second; 
    }
    it++;
}



    // insert the ropot pos in the graph
    Graph::node robot;
    robot.x=x;
    robot.y=y;
    
    robot.neighbours.emplace_back(myConnection);
    graph.emplace_back(robot);
    graph[myConnection].neighbours.emplace_back(graph.size()-1);
}
// void Graph::setNodes(std::vector<Graph::node> *nodes){
//     this->nodes_ = nodes;
// }

// void Graph::setCells(std::vector<Graph::cell> *cells){
//     this->cells_ = cells;
// }

// std::vector<Graph::node>* Graph::nodes(){
//     return this->nodes_;
// }

// std::vector<Graph::cell>* Graph::cells(){
//     std::cout<< "Tot Cells_: " << this->cells_->size() << std::endl;
//     return this->cells_;
// }