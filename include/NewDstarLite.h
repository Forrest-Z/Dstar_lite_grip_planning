/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-04-27T20:10:14-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: NewDstarLite.h
 * @Last modified by:   yzy
 * @Last modified time: 2018-05-07T23:07:11-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */



#include <vector>
#include <opencv2/opencv.hpp>
#include <queue>
#include <tr1/unordered_map>
#include <memory>
#include "BuildingMap.hpp"

const double infinity = std::numeric_limits<double>::infinity();


/**
 * [operator: an operator for class Node to compare the with each other, used in priority_queue]
 * @return [bool]
 */
template<class T>
class Greater {
public:
bool operator()(const T& p1,const T& p2){
								if (p1->k.first < p2->k.first) return false;
								else if (p1->k.first > p2->k.first) return true;
								return p1->k.second > p2->k.second;
}
};



/**
 * [Node  a grid with its specific parameters]
 * @param x   [x position]
 * @param y   [y position]
 * @param g   [cost of the path from the start node]
 * @param rhs [min of the predecessor's g(s') + c(s,s')]
 * @param index [index of the node in 1D map]
 * @param k    [key of the node]
 */
class Node {
public:
unsigned int x;
unsigned int y;
unsigned int index;
double g;
double rhs;
//unsigned double min_cost_succ;
std::pair< double, double> k;
//unsigned int parent_index;
Node (const unsigned int& x, const unsigned int& y, const double& g, const double& rhs) : x(x), y(y), g(g), rhs(rhs) {
								// initialize min_cost_succ
								index = y *COL + x;
								k.first = infinity; k.second = infinity;

}

bool operator == (std::shared_ptr<Node>s2) const {
								return ((x == s2->x) && (y == s2->y));
}

bool operator != (std::shared_ptr<Node>s2) const {
								return ((x != s2->x) || (y != s2->y));
}

bool operator > (std::shared_ptr<Node>s2) const {
								if (k.first-0.00001 > s2->k.first) return true;
								else if (k.first < s2->k.first-0.00001) return false;
								return k.second > s2->k.second;
}

bool operator <= (std::shared_ptr<Node>s2) const {
								if (k.first < s2->k.first) return true;
								else if (k.first > s2->k.first) return false;
								return k.second < s2->k.second + 0.00001;
}


bool operator < (std::shared_ptr<Node>s2) const {
								if (k.first + 0.000001 < s2->k.first) return true;
								else if (k.first - 0.000001 > s2->k.first) return false;
								return k.second < s2->k.second;
}
};
/**
 * [DstarLite the class that contains the functions for the dynamic path planning algorithm ]
 * @param gridmap_old [initial map]
 * @param gridmap_new [changed map]
 * @param start_x     [start node x]
 * @param start_y     [start node y]
 * @param goal_x      [goal node x]
 * @param goal_y      [goal node y]
 */
class DstarLite {
// Constructor
public:
DstarLite(Buildingmap::gridMatrix gridmap_old,Buildingmap::gridMatrix gridmap_new,unsigned int start_x,unsigned int start_y,unsigned int goal_x,unsigned int goal_y);
std::vector<int> map_info_old;
std::vector<int> map_info_new;
cv::Mat Map;
void replan();
cv::VideoWriter video;
//private:
// Initialize the node index
unsigned int start_node_index;
unsigned int goal_node_index;
unsigned int last_node_index;
double km = 0;
std::priority_queue<std::shared_ptr<Node>,std::vector<std::shared_ptr<Node> >,Greater<std::shared_ptr<Node> > > open_list;
std::tr1::unordered_map<unsigned int, std::shared_ptr<Node> > open_hash;
std::tr1::unordered_map<unsigned int, std::shared_ptr<Node> > node_map;
std::tr1::unordered_map<unsigned int, double > cost_between_nodes_map;
std::vector<std::shared_ptr<Node> > successor_node_list(std::shared_ptr<Node> currentnode);
std::vector<std::shared_ptr<Node> > predecessor_node_list(std::shared_ptr<Node> currentnode);

unsigned int hash_two_nodes(int index1, int index2);
void initialize();
double cost_between_nodes(std::shared_ptr<Node> a, std::shared_ptr<Node> b);
double heuristic(std::shared_ptr<Node> a, std::shared_ptr<Node> b);
std::vector<int> computeShortestPath();
double min(const double& g, const double& rhs);
unsigned int Find_min_Node( std::vector<std::shared_ptr<Node> > successor_nodes);
int computeIndex(const int& x, const int& y, const int& column_size);
void nodeIndexing(Node& node);
void addNodeToMap(std::shared_ptr<Node> node);
std::pair<double,double> calculateKey(std::shared_ptr<Node> currentnode);
bool isKeySmaller(std::pair<double,double> p1,std::pair<double,double> p2);
void updateVertex(std::shared_ptr<Node> currentnode);
void Scan_for_cost_change(std::shared_ptr<Node> currentnode);
void update_nodes_cost(std::shared_ptr<Node> currentnode);
bool AreSame(double a, double b);
unsigned int Find_path_Node( std::vector<std::shared_ptr<Node> > successor_nodes,unsigned int path_index);
cv::Mat build_background(Buildingmap::gridMatrix gridmap_old,Buildingmap::gridMatrix gridmap_new,unsigned int start_x,unsigned int start_y,unsigned int goal_x,unsigned int goal_y);
void drawPath(std::vector<int> path);
void draw_move_to_next(unsigned int last, unsigned int next);
};
