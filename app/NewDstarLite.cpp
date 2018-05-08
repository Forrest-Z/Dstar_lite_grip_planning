/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-04-20T22:06:58-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: NewDstarLite.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-04-20T22:07:04-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 * @Discription: This is the grid based impletation for Sven Koenig's D*lite, which is
 * a dynamic path planning algorithm for unknown dynamic encironment.
 */

#include "NewDstarLite.h"
#include "BuildingMap.hpp"
#include <limits>
#include <iostream>
#include <memory>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <unordered_map>


/**
 * [DstarLite::initialize the class and the parameters of the grids, see Sven Koenig'spaper: D*lite
 * for procedure code Initialize()]
 */
void DstarLite::initialize() {

        // empty set U; 1. clear open_list; 2. clear openHash
        // clear open_list by reset open_list
        while(!open_list.empty()) open_list.pop();
        open_hash.clear();
        node_map.clear();
        // set up km = 0
        km = 0;

        // initial g value
        double g_infinity = infinity;

        // initial rhs value
        double rhs_infinity = infinity;

        // for all s in S, rhs(s) = inf and g(s) = inf
        for (auto x = 0; x < COL; x++) {
                for(auto y = 0; y < ROW; y++) {

                        std::shared_ptr<Node> temp_node = std::make_shared<Node>(x, y, g_infinity, rhs_infinity);   // initial temp_node

                        // insert to node_map line(04)
                        addNodeToMap(temp_node);
                }
        }
        // initial rhs of goal as 0


        node_map[goal_node_index]->rhs = 0;
        // update key of goal
        node_map[goal_node_index]->k = calculateKey(node_map[goal_node_index]);
        // insert goal_node to open_list
        open_list.push(node_map[goal_node_index]);
        open_hash.insert(std::make_pair(goal_node_index,node_map[goal_node_index]));
        if(map_info_old[start_node_index] == 0 || map_info_new[start_node_index] == 0)
                throw std::invalid_argument( "Start node is invalid" );
        if(map_info_old[goal_node_index] == 0 || map_info_new[goal_node_index] == 0)
                throw std::invalid_argument( "Goal node is invalid" );
}
/**
 * [DstarLite::DstarLite description]
 * @param gridmap_old [initial map]
 * @param gridmap_new [changed map]
 * @param start_x     [start node x]
 * @param start_y     [start node y]
 * @param goal_x      [goal node x]
 * @param goal_y      [goal node y]
 */
DstarLite::DstarLite(Buildingmap::gridMatrix gridmap_old,Buildingmap::gridMatrix gridmap_new,unsigned int start_x,unsigned int start_y,unsigned int goal_x,unsigned int goal_y) {
        // Iterate through the 2D map and turns it into 1D
        for (int i = 0; i< COL; i++) {
                for(int j = 0; j< ROW; j++) {
                        if(gridmap_old[i][j] == 1)
                                map_info_old.push_back(1);
                        else
                                map_info_old.push_back(0);
                }
        }
        // Iterate through the 2D map and turns it into 1D
        for (int i = 0; i< COL; i++) {
                for(int j = 0; j< ROW; j++) {
                        if(gridmap_new[i][j] == 1)
                                map_info_new.push_back(1);
                        else
                                map_info_new.push_back(0);
                }
        }
        // Assign the inde for start and goal node
        start_node_index = start_y *COL + start_x;
        last_node_index = start_node_index;
        goal_node_index = goal_y *COL + goal_x;
        initialize();
        Map = build_background(gridmap_old,gridmap_new,start_x,start_y,goal_x,goal_y);

}

// Draw the grip map with start and goal node using OpenCV, default as 10 by 10 gripds
cv::Mat DstarLite:: build_background(Buildingmap::gridMatrix gridmap_old,Buildingmap::gridMatrix gridmap_new,unsigned int start_x,unsigned int start_y,unsigned int goal_x,unsigned int goal_y){
        int image_x = COL * 10;
        int image_y = ROW * 10;
        cv::Mat Background = cv::Mat::zeros(image_x, image_y, CV_8UC3);
        for (int i = 0; i < COL; i++) {
                for (int j = 0; j < ROW; j++) {
                        if (gridmap_old[i][j] == 0 && gridmap_new[i][j] == 0)
                                continue;
                        else if(gridmap_old[i][j] == 1 && gridmap_new[i][j] == 0)
                                cv::circle(Background, cv::Point((j*10+5),(i*10+5)),5, cv::Scalar(255,0,0),CV_FILLED, 8,0);
                        else if (gridmap_old[i][j] == 0 && gridmap_new[i][j] == 1)
                                cv::rectangle(Background, cv::Point(j * 10, (i + 1) * 10),
                                              cv::Point((j + 1) * 10, i * 10),
                                              cv::Scalar(0, 255, 255), -1, 8);
                        else if (gridmap_old[i][j] == 1 && gridmap_new[i][j] == 1)
                                cv::rectangle(Background, cv::Point(j * 10, (i + 1) * 10),
                                              cv::Point((j + 1) * 10,  i* 10),
                                              cv::Scalar(255, 255, 0), -1, 8);
                }
        }

        cv::rectangle(Background, cv::Point(start_x * 10, start_y * 10),
                      cv::Point((start_x + 1) * 10, (start_y + 1) * 10),
                      cv::Scalar(0, 255, 0), -1, 8);
        cv::rectangle(Background, cv::Point(goal_x * 10, goal_y * 10),
                      cv::Point((goal_x + 1) * 10, (goal_y + 1) * 10),
                      cv::Scalar(0, 255, 0), -1, 8);
        return Background;
}
/**
 * [DstarLite::drawPath draw the path grid by grip]
 * @param path [vector<int>]
 */
void DstarLite::drawPath(std::vector<int> path) {
        int start_path_x = 10* (path.front() % COL) +5; int goal_path_x = 0;
        int start_path_y = 10* (path.front() / COL) + 5; int goal_path_y = 0;
        //cv::Point start; cv::Point goal;
        path.erase(path.begin());
        while(!path.empty()) {
                std::cout<< "Draw node " << path.back()<<std::endl;
                goal_path_x = 10* (path.front() % COL) +5;
                goal_path_y = 10* (path.front() / COL) +5;
                cv::Point start(start_path_x,start_path_y); cv::Point goal(goal_path_x,goal_path_y);
                cv::line(Map,start,goal,cv::Scalar(0,0,0),1,8);
                start_path_x = goal_path_x;
                start_path_y = goal_path_y;
                path.erase(path.begin());
                if(!video.isOpened()) {
                        throw std::invalid_argument( "Video writer is not opened !" );
                }
                video.write(Map);
        }

}
/**
 * @brief calcalute the key value k
 * @param currentnode; the current node
 * @param km; variable to compensate the change of start_node caused by vehicle movement
 * @return pair k = [k1 k2]
 */
std::pair<double,double> DstarLite::calculateKey(std::shared_ptr<Node> currentnode){

        // Initialize key inside function for return value
        std::pair<double,double> key;

        // get heuristic between currentnode and start_node (start node may vary with time)
        double h = heuristic(currentnode, node_map[start_node_index]);

        // get the smaller value between g and rhs in currentnode
        double min_value = min(currentnode->g, currentnode->rhs);

        // assign value for key first element
        key.first = min_value + h;

        // assign value for key second element
        key.second = min_value;

        // return value
        return key;

}

/**
 * @brief return smaller value between g and rhs
 * @param g; the g cost
 * @param rhs; c(x, x') + g(x' +  goal)
 * @return the smaller value
 */
double DstarLite::min(const double& g, const double& rhs) {
        if (g <= rhs) {
                return g;
        } else {
                return rhs;
        }
}


/**
 * @brief Initialize(): initialize variables in class DstarLite
 */



/**
 * @brief calculate index using coordinate x and y, and the size of column;
 * the world coordinate frame with x - direction (right) and y - direction (down)
 * @param x; the cooridnate x
 * @param y; the cooridnate y;
 * @param column_size; the size of column in y - direction
 */
int DstarLite::computeIndex(const int& x, const int& y, const int& column_size) {

        return (y*column_size + x);

}


/**
 * @brief add node to node_map
 * @param node;
 * @param node_map; the map store all node information
 * @warning: this function will change the input node_map
 */
void DstarLite::addNodeToMap(std::shared_ptr<Node> node) {

        // get the index of node
        unsigned int map_index = node->index;

        // make pair <index, node>
        auto index_node_pair = std::make_pair(map_index, node);

        // insert the node to node_map
        node_map.insert(index_node_pair);


}
/**
 * [DstarLite::updateVertex update the node's rhs]
 * @param currentnode [Node]
 */
void DstarLite::updateVertex(std::shared_ptr<Node> currentnode) {
        // Line 07 assign rhs(u) to its smallerst successors' c(s,s') + g(s')
        if ( currentnode->index != goal_node_index) {
                double tem = infinity;
                std::vector<std::shared_ptr<Node> > successor_list;
                successor_list = successor_node_list(currentnode);
                for (auto node : successor_list) {
                        // calculate c(u,s') + g(s')
                        double new_tem = node->g +cost_between_nodes(node,currentnode);
                        if(new_tem < tem)
                                tem = new_tem;
                }
                // std::cout<<"current node "<<currentnode->index<<" 's rhs is "<< tem<<std::endl;
                node_map[currentnode->index]->rhs = tem;
        }
        // lazy remove the open_hash if currentnode belongs to open_list. Line 8
        std::priority_queue<std::shared_ptr<Node>,std::vector<std::shared_ptr<Node> >,Greater<std::shared_ptr<Node> > > tem_list;
        if(open_hash.find(currentnode->index) != open_hash.end()) {
                open_hash.erase(open_hash.find(currentnode->index));
                while(!open_list.empty()) {
                        std::shared_ptr<Node> topNode = open_list.top();
                        open_list.pop();
                        if(open_hash.find(topNode->index)!= open_hash.end())
                                tem_list.push(topNode);
                }
                open_list.swap(tem_list);
        }
        // Line 09 if g(u) not euqal to rhs(u), recalclate the key value and inset the node to open_list
        if(!AreSame(currentnode->g, currentnode->rhs)) {
                currentnode->k = calculateKey(currentnode);
                open_list.push(currentnode);   // insert to open_list
                open_hash.insert(std::make_pair(currentnode->index,currentnode)); // insert to open_hash
        }



}

/**
 * [DstarLite::successor_node_list Find all the successors of the current node]
 * @param currentnode [Node]
 */
std::vector<std::shared_ptr<Node> > DstarLite::successor_node_list(std::shared_ptr<Node> currentnode)  {
        std::vector<std::shared_ptr<Node> > successor_list;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode->x + i;     // Find its neighbours
                        int new_y = currentnode->y + j;
                        if (new_x < 0 || new_x > (COL-1) || new_y < 0 || new_y > (ROW -1) ||  (i == 0 && j == 0)) // if its x and y are invalid, ignore the node
                                continue;
                        else      // if not, recore the index of the neighbours to a vector
                                successor_list.push_back(node_map[computeIndex(new_x, new_y, COL)]);
                }
        }
        return successor_list;
}

/**
 * [DstarLite::cost_between_nodes determine the cost between two nodes, infinity if any of them is blocked]
 * @param  a [Node]
 * @param  b [Node]
 * @return   [double]
 */
double DstarLite::cost_between_nodes(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        // Build a pair for two nodes
        unsigned int node_pair  = hash_two_nodes(a->index,b->index);
        if(cost_between_nodes_map.find(node_pair) != cost_between_nodes_map.end()) // if the nodes are already in the cost map
                return cost_between_nodes_map[node_pair];                          // return this value
        else if(map_info_old[a->index] == 0 || map_info_old[b->index] == 0) {        // if not inside the map, verfity if one of them are walls
                cost_between_nodes_map.insert(std::make_pair(node_pair, infinity));  // if is, cost between wall and nodes are INFINITY
                return infinity;
        }

        else {                                                                     // if no walls, the distance is the euclidean distance
                unsigned int axx = a->x;
                unsigned int bxx = b->x;
                unsigned int ayy = a->y;
                unsigned int byy = b->y;
                double cost = 10*std::sqrt((axx-bxx)*(axx-bxx)+(ayy-byy)*(ayy-byy));
                cost_between_nodes_map.insert(std::make_pair(node_pair, cost));
                return cost;
        }

}

/**
 * [DstarLite::heuristic euclidean distance for two nodes]
 * @param  a [Node]
 * @param  b [Node]
 * @return   [double]
 */
double DstarLite::heuristic(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        return 10*std::sqrt((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));  // euclidean distance
}

/**
 * [DstarLite::AreSame determine two numbers are approximately same]
 * @param  a [double]
 * @param  b [double]
 * @return   [bool]
 */
bool DstarLite::AreSame(double a, double b)
{
        return fabs(a - b) < 0.01;    // verfity two double are same by defining a boundary
}

/**
 * [DstarLite::predecessor_node_list basiclly same as successor_list for eight connected neighbours]
 * @param currentnode [Node]
 */
std::vector<std::shared_ptr<Node> > DstarLite::predecessor_node_list(std::shared_ptr<Node>  currentnode)  {
        std::vector<std::shared_ptr<Node> > predecessor_list;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode->x + i; // Find its neighbours
                        int new_y = currentnode->y + j;
                        if (new_x < 0 || new_x > (COL -1) || new_y < 0 || new_y > (ROW -1) || (i == 0 && j == 0)) // if its x and y are invalid, ignore the node
                                continue;
                        else // if not, recore the index of the neighbours to a vector
                                predecessor_list.push_back(node_map[computeIndex(new_x, new_y, COL)]);
                }
        }
        return predecessor_list;
}

/**
 * [DstarLite::computeShortestPath find the optimal path based on the current map]
 * @return [vector<int> which is the indexes of the path]
 */
std::vector<int> DstarLite::computeShortestPath() {
        while(isKeySmaller(open_list.top()->k,calculateKey(node_map[start_node_index])) || !AreSame(node_map[start_node_index]->rhs,node_map[start_node_index]->g)) {
                std::pair<double,double> key_old;
                std::shared_ptr<Node> topNode = open_list.top();
                std::cout<<"Top node is "<< topNode->index<<std::endl;
                key_old = topNode->k;
                open_list.pop();
                std::pair<double,double> key_new = calculateKey(topNode);
                // line 13
                if(isKeySmaller(key_old,key_new)) {
                        // line 14
                        topNode->k = calculateKey(topNode);
                        open_list.push(topNode);
                }
                // line 15
                else if(topNode->g > topNode->rhs) {
                        topNode->g = topNode->rhs;
                        std::vector<std::shared_ptr<Node> > predecessor_list = predecessor_node_list(topNode);
                        for(auto node : predecessor_list) {
                                updateVertex(node);
                        }
                }
                else {
                        topNode->g = infinity;
                        std::vector<std::shared_ptr<Node> > predecessor_list = predecessor_node_list(topNode);
                        for(auto node : predecessor_list) {
                                updateVertex(node);
                        }
                        updateVertex(topNode);
                }
        }

        // return the path to draw
        unsigned int path_index = start_node_index;
        std::vector<int> path;
        path.push_back(path_index);
        while(path_index != goal_node_index) {
                std::vector<std::shared_ptr<Node> > successor_list = successor_node_list(node_map[path_index]);
                // line 26 find the node has smallest c(current,s') + g(s')
                path_index = Find_path_Node(successor_list,path_index);
                path.push_back(path_index);
                //  std::cout<<"path node is "<< path_index<<std::endl;
        }
        drawPath(path);
        return path;

}

/**
 * [DstarLite::hash_two_nodes hash two indexes of two nodes as the key for cost map between nodes]
 * @param  index1 [int]
 * @param  index2 [int]
 * @return        [int]
 */
unsigned int DstarLite::hash_two_nodes(int index1, int index2) {
        // return index1*ROW*COL*2 + index2;
        if (index1 > 9 && index2 > 9)
                return index1 * index2 + 10*(index1+index2);
        else
                return index1*ROW*ROW*COL*COL+ index2;
}

/**
 * [DstarLite::isKeySmaller compare the key of two nodes]
 * @param  p1 [pair]
 * @param  p2 [pair]
 * @return    [bool]
 */
bool DstarLite::isKeySmaller(std::pair<double,double> p1,std::pair<double,double> p2) {
        if (p1.first < p2.first) return true;
        else if (p1.first > p2.first) return false;
        return p1.second < p2.second;

}

/**
 * [DstarLite::Find_min_Node find the min nodes among the successors]
 * @param  successor_nodes [vector<node>]
 * @return                 [int: index]
 */
unsigned int DstarLite::Find_min_Node( std::vector<std::shared_ptr<Node> > successor_nodes) {
        double tem = infinity;
        unsigned int index = successor_nodes.front()->index;
        for(auto node : successor_nodes) {
                if((cost_between_nodes(node,node_map[start_node_index]) + node->g) < tem) {
                        tem = cost_between_nodes(node,node_map[start_node_index]) + node->g;
                        index = node->index;
                }
        }
        return index;
}

/**
 * [DstarLite::Find_path_Node find the node that has min cost+g]
 * @param  successor_nodes [vector <Node>]
 * @param  path_index      [int]
 * @return                 [int]
 */
unsigned int DstarLite::Find_path_Node( std::vector<std::shared_ptr<Node> > successor_nodes,unsigned int path_index) {
        double tem = infinity;
        unsigned int index = successor_nodes.front()->index;
        for(auto node : successor_nodes) {
                if((cost_between_nodes(node,node_map[path_index]) + node->g) < tem) {
                        tem = cost_between_nodes(node,node_map[path_index]) + node->g;
                        index = node->index;
                }
        }
        return index;
}

/**
 * [DstarLite::Scan_for_cost_change scan for the eight connected neighbours for map change by comparing the inital and changed map]
 * @param currentnode [Node]
 */
void DstarLite::Scan_for_cost_change(std::shared_ptr<Node> currentnode) {
        bool update_km =  false;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode->x + i; // Find its neighbours
                        int new_y = currentnode->y + j;
                        unsigned int index_new = computeIndex(new_x,new_y,COL);
                        if (new_x < 0 || new_x > (COL -1) || new_y < 0 || new_y > (ROW -1) || (i == 0 && j == 0)) // if its x and y are invalid, ignore the node
                                continue;
                        else if(map_info_old[index_new] != map_info_new[index_new] ) {

                                if(!update_km) {
                                        km = heuristic(node_map[last_node_index],node_map[start_node_index]);
                                        last_node_index = start_node_index;
                                        update_km = true;
                                }
                                // update nodes cost
                                update_nodes_cost(node_map[index_new]);
                                updateVertex(node_map[index_new]);
                                computeShortestPath();
                        }

                }
        }
}

/**
 * [DstarLite::update_nodes_cost update the cost between two nodes ]
 * @param currentnode [Node]
 */
void DstarLite::update_nodes_cost(std::shared_ptr<Node> currentnode) {
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode->x + i; // Find its neighbours
                        int new_y = currentnode->y + j;
                        if (new_x < 0 || new_x > (COL-1) || new_y < 0 || new_y > (ROW -1) || (i == 0 && j == 0)) // if its x and y are invalid, ignore the node
                                continue;
                        else  {
                                int index_new = computeIndex(new_x,new_y,COL);
                                unsigned int node_pair  = hash_two_nodes(currentnode->index,index_new);
                                if(map_info_new[currentnode->index] == 0 || map_info_new[index_new ] == 0) {  // if not inside the map, verfity if one of them are walls
                                        //cost_between_nodes_map.insert(std::make_pair<node_pair, INFINITY>); // if is, cost between wall and nodes are INFINITY
                                        if(cost_between_nodes_map.find(node_pair) != cost_between_nodes_map.end()) // if the nodes are already in the cost map
                                                cost_between_nodes_map[node_pair] = infinity;
                                        else
                                                cost_between_nodes_map.insert(std::make_pair(node_pair, infinity));
                                }
                                else {
                                        if(cost_between_nodes_map.find(node_pair) != cost_between_nodes_map.end()) // if the nodes are already in the cost map
                                                cost_between_nodes_map[node_pair] = 10*std::sqrt((new_x - currentnode->x)*(new_x - currentnode->x) + (new_y - currentnode->y)*(new_y - currentnode->y));
                                        else
                                                // if no walls, the distance is the euclidean distance
                                                cost_between_nodes_map.insert(std::make_pair(node_pair, 10*std::sqrt((new_x - currentnode->x)*(new_x - currentnode->x) + (new_y - currentnode->y)*(new_y - currentnode->y))));
                                }
                        }
                }
        }
}

/**
 * [DstarLite::draw_move_to_next draw the node moving to next node]
 * @param last [int]
 * @param next [int]
 */
void DstarLite::draw_move_to_next(unsigned int last, unsigned int next) {
        unsigned int last_x =  (last % COL);
        unsigned int last_y =  (last / COL);
        unsigned int next_x =  (next % COL);
        unsigned int next_y =  (next / COL);
        cv::rectangle(Map, cv::Point(last_x * 10, (last_y + 1) * 10),
                      cv::Point((last_x + 1) * 10,  last_y* 10),
                      cv::Scalar(255, 255, 0), -1, 8);
        cv::rectangle(Map, cv::Point(next_x * 10, (next_y + 1) * 10),
                      cv::Point((next_x + 1) * 10,  next_y* 10),
                      cv::Scalar(0, 255, 0), -1, 8);
        video.write(Map);
}
/**
 * [DstarLite::replan move along the path and repair the path when environment is changed]
 */
void DstarLite::replan() {
        std::cout<< "Start planning path" << std::endl;
        computeShortestPath();
        std::cout<< "Find init path" << std::endl;
        while(start_node_index != goal_node_index) {
                std::vector<std::shared_ptr<Node> > successor_list = successor_node_list(node_map[start_node_index]);
                // line 26 find the node has smallest c(current,s') + g(s')
                unsigned int tem = start_node_index;
                start_node_index = Find_min_Node(successor_list);
                // Move to the current node, draw the node movement
                draw_move_to_next(tem,start_node_index);
                std::cout<< "move to node "<< start_node_index << std::endl;
                //   Scan graph for cost change
                Scan_for_cost_change(node_map[start_node_index]);
        }
        video.release();
}
