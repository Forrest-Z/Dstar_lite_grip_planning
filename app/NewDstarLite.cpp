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
 */

#include "NewDstarLite.h"
#include "BuildingMap.hpp"
#include <limits>

DstarLite::DstarLite(Buildingmap::gridMatrix gridmap_old,Buildingmap::gridMatrix gridmap_new,int start_x,int start_y,int goal_x,int goal_y) {
        for (int i = 0; i< ROW; i++) {
                for(int j = 0; j< COL; j++) {
                        if(gridmap_old[i][j] == 1)
                                map_info_old.push_back(1);
                        else
                                map_info_old.push_back(0);
                }
        }
        for (int i = 0; i< ROW; i++) {
                for(int j = 0; j< COL; j++) {
                        if(gridmap_new[i][j] == 1)
                                map_info_new.push_back(1);
                        else
                                map_info_new.push_back(0);
                }
        }
        start_node.x = start_x; start_node.y = start_y; start_node.index = start_node.y *COL + start_node.x;
        last_node = start_node;
        goal_node.x = goal_x; goal_node.y = goal_y; goal_node.index = goal_node.y *COL + goal_node.x;

}

/**
 * @brief calcalute the key value k
 * @param currentnode; the current node
 * @param km; variable to compensate the change of start_node caused by vehicle movement
 * @return pair k = [k1 k2]
 */
std::pair<double,double> DstarLite::calculateKey(const Node& currentnode,
                                                 const Node& start_node){

        // Initialize key inside function for return value
        std::pair<double,double> key;

        // get heuristic between currentnode and start_node
        double h = heuristic(currentnode, start_node);

        // get the smaller value between g and rhs in currentnode
        double min_value = min(currentnode.g, currentnode.rhs);

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
void DstarLite::initialize() {

        // empty set U; 1. clear open_list; 2. clear openHash
        // clear open_list by reset open_list
        while(!open_list.empty()) open_list.pop();
        open_hash.clear();
        node_map.clear();
        // set up km = 0
        km = 0;

        // initial g value
        double g_infinity = INFINITY;

        // initial rhs value
        double rhs_infinity = INFINITY;

        // for all s in S, rhs(s) = inf and g(s) = inf
        for (auto x = 1; x < ROW; x++) {
                for(auto y = 1; y < COL; y++) {

                        Node temp_node(x, y, g_infinity, rhs_infinity);         // initial temp_node

                        // update index of temp_node
                        nodeIndexing(temp_node);

                        // insert to node_map line(04)
                        addNodeToMap(temp_node, node_map);
                }
        }
        // initial rhs of goal as 0
        goal_node.rhs = 0;

        // update key of goal
        goal_node.k = calculateKey(goal_node,start_node);
        // insert goal_node to open_list
        open_list.push(goal_node);
        open_hash.insert(std::make_pair(goal_node.index,goal_node));
}


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
 * @brief update the index in node
 * @param node; the node that we want to update index
 * @warning: this function will alter the index value of input node !!
 */
void DstarLite::nodeIndexing(Node& node) {


        // assign variable x for x coordinate
        int x = node.x;

        // assign variable y for y coordinate
        int y = node.y;

        // compute node index; COL is the static variable in BuildingMap.hpp
        int index = computeIndex(x, y, COL);

        // assign value for element index in node class
        node.index = index;
}

/**
 * @brief add node to node_map
 * @param node;
 * @param node_map; the map store all node information
 * @warning: this function will change the input node_map
 */
void DstarLite::addNodeToMap(const Node& node, std::tr1::unordered_map<int, Node >& node_map) {

        // get the index of node
        int map_index = node.index;

        // make pair <index, node>
        auto index_node_pair = std::make_pair(map_index, node);

        // insert the node to node_map
        node_map.insert(index_node_pair);


}
void DstarLite::updateVertex(Node currentnode) {
        // Line 07 assign rhs(u) to its smallerst successors' c(s,s') + g(s')
        if ( currentnode != goal_node) {
                double tem = INFINITY;
                std::vector<Node> successor_list;
                successor_list = successor_node_list(currentnode);
                for (auto node : successor_list) {
                        // calculate c(u,s') + g(s')
                        double new_tem = node.g +cost_between_nodes(node,currentnode);
                        if(new_tem < tem)
                                tem = new_tem;
                }
                node_map[currentnode.index].rhs = tem;
        }
        // lazy remove the open_hash if currentnode belongs to open_list. Line 8
        std::priority_queue<Node,std::vector<Node> > tem_list;
        if(open_hash.find(currentnode.index) != open_hash.end()) {
                open_hash.erase(open_hash.find(currentnode.index));
                while(!open_list.empty()) {
                        Node topNode = open_list.top();
                        open_list.pop();
                        if(open_hash.find(topNode.index)!= open_hash.end())
                                tem_list.push(topNode);
                }
                open_list = tem_list;
        }
        // Line 09 if g(u) not euqal to rhs(u), recalclate the key value and inset the node to open_list
        if(AreSame(currentnode.g, currentnode.rhs)) {
                currentnode.k = calculateKey(currentnode,last_node);
                open_list.push(currentnode);   // insert to open_list
                open_hash.insert(std::make_pair(currentnode.index,currentnode)); // insert to open_hash
        }



}

std::vector<Node> DstarLite::successor_node_list(Node currentnode)  {
        std::vector<Node> successor_list;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode.x + i;     // Find its neighbours
                        int new_y = currentnode.y + j;
                        if (new_x < 0 || new_x > COL || new_y < 0 || new_y > ROW) // if its x and y are invalid, ignore the node
                                continue;
                        else      // if not, recore the index of the neighbours to a vector
                                successor_list.push_back(node_map[computeIndex(new_x, new_y, COL)]);
                }
        }
        return successor_list;
}

double DstarLite::cost_between_nodes(Node a, Node b) {
        // Build a pair for two nodes
        std::pair<int,int> node_pair  = std::make_pair(a.index,b.index);
        if(cost_between_nodes_map.find(node_pair) != cost_between_nodes_map.end()) // if the nodes are already in the cost map
                return cost_between_nodes_map[node_pair];                          // return this value
        else if(map_info_old[a.index -1] == 0 || map_info_old[b.index -1] == 0) {        // if not inside the map, verfity if one of them are walls
                cost_between_nodes_map.insert(std::make_pair(node_pair, INFINITY));  // if is, cost between wall and nodes are INFINITY
                return INFINITY;
        }

        else {                                                                     // if no walls, the distance is the euclidean distance
                cost_between_nodes_map.insert(std::make_pair(node_pair, std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y))));
                return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
        }

}

double DstarLite::heuristic(Node a, Node b) {
        return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));  // euclidean distance
}

bool DstarLite::AreSame(double a, double b)
{
        return fabs(a - b) < 0.01;    // verfity two double are same by defining a boundary
}


// basiclly same as successor_list for eight connected neighbours
std::vector<Node> DstarLite::predecessor_node_list(Node currentnode)  {
        std::vector<Node> predecessor_list;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode.x + i; // Find its neighbours
                        int new_y = currentnode.y + j;
                        if (new_x < 0 || new_x > COL || new_y < 0 || new_y > ROW) // if its x and y are invalid, ignore the node
                                continue;
                        else // if not, recore the index of the neighbours to a vector
                                predecessor_list.push_back(node_map[computeIndex(new_x, new_y, COL)]);
                }
        }
        return predecessor_list;
}

std::vector<int> DstarLite::computeShortestPath() {

        while(open_list.top() < start_node || !AreSame(start_node.rhs,start_node.g)) {
                std::pair<double,double> key_old;
                Node topNode = open_list.top();
                key_old = topNode.k;
                open_list.pop();
                std::pair<double,double> key_new;
                key_new = calculateKey(topNode,last_node);
                // line 13
                if(isKeySmaller(key_old,key_new)) {
                        // line 14
                        topNode.k = calculateKey(topNode,last_node);
                        open_list.push(topNode);
                }
                // line 15
                else if(topNode.g > topNode.rhs) {
                        topNode.g = topNode.rhs;
                        std::vector<Node> predecessor_list = predecessor_node_list(topNode);
                        for(auto node : predecessor_list) {
                                updateVertex(node);
                        }
                }
                else {
                        topNode.rhs = INFINITY;
                        std::vector<Node> predecessor_list = predecessor_node_list(topNode);
                        for(auto node : predecessor_list) {
                                updateVertex(node);
                        }
                        updateVertex(topNode);
                }
        }
}


bool DstarLite::isKeySmaller(std::pair<double,double> p1,std::pair<double,double> p2) {
        if (p1.first < p2.first) return true;
        else if (p1.first > p2.first) return false;
        return p1.second < p2.second + 0.01;
}

Node DstarLite::Find_min_Node(const std::vector<Node>& successor_nodes) {
        Node tem = successor_nodes.front();
        for(auto node : successor_nodes) {
                if(tem > node)
                        tem =  node;
        }
        return tem;
}

bool DstarLite::isNodeValid(Node node){

}

void DstarLite::Scan_for_cost_change(Node currentnode) {
        bool update_km =  false;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode.x + i; // Find its neighbours
                        int new_y = currentnode.y + j;
                        int index_new = computeIndex(new_x,new_y,COL);
                        if (new_x < 0 || new_x > COL || new_y < 0 || new_y > ROW) // if its x and y are invalid, ignore the node
                                continue;
                        else if(map_info_old[index_new -1] != map_info_new[currentnode.index -1] ) {

                                if(!update_km) {
                                        km = heuristic(last_node,start_node);
                                        last_node = start_node;
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

void DstarLite::update_nodes_cost(Node currentnode) {
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode.x + i; // Find its neighbours
                        int new_y = currentnode.y + j;
                        if (new_x < 0 || new_x > COL || new_y < 0 || new_y > ROW) // if its x and y are invalid, ignore the node
                                continue;
                        else  {
                                int index_new = computeIndex(new_x,new_y,COL);
                                std::pair<int,int> node_pair  = std::make_pair(currentnode.index,index_new);
                                if(map_info_new[currentnode.index -1 ] == 0 || map_info_new[index_new -1 ] == 0) {  // if not inside the map, verfity if one of them are walls
                                        //cost_between_nodes_map.insert(std::make_pair<node_pair, INFINITY>); // if is, cost between wall and nodes are INFINITY
                                        if(cost_between_nodes_map.find(node_pair) != cost_between_nodes_map.end()) // if the nodes are already in the cost map
                                                cost_between_nodes_map[node_pair] = INFINITY;
                                        else
                                                cost_between_nodes_map.insert(std::make_pair(node_pair, INFINITY));
                                }
                                else {
                                        if(cost_between_nodes_map.find(node_pair) != cost_between_nodes_map.end()) // if the nodes are already in the cost map
                                                cost_between_nodes_map[node_pair] = std::sqrt((new_x - currentnode.x)*(new_x - currentnode.x) + (new_y - currentnode.y)*(new_y - currentnode.y));
                                        else
                                                // if no walls, the distance is the euclidean distance
                                                cost_between_nodes_map.insert(std::make_pair(node_pair, std::sqrt((new_x - currentnode.x)*(new_x - currentnode.x) + (new_y - currentnode.y)*(new_y - currentnode.y))));
                                }
                        }
                }
        }
}


void DstarLite::replan() {
        computeShortestPath();
        while(start_node.index != goal_node.index) {
                std::vector<Node> successor_list = successor_node_list(start_node);
                // line 26 find the node has smallest c(current,s') + g(s')
                start_node = Find_min_Node(successor_list);
                // Move to the current node, draw the node movement

                //   Scan graph for cost change
                Scan_for_cost_change(start_node);
        }
}
