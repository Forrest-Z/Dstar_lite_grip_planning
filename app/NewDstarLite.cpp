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
#include <limits>
static const int INFINITY = std::numeric_limits<double>::infinity();

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
                                                 const Node& start_node, const double& km){

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
        while(!open_list.empty()) openList.pop();
        open_hash.clear();
        node_map.clear();
        // set up km = 0
        km = 0;

        // initial g value
        double g_infinity = INFINITY;

        // initial rhs value
        double rhs_infinity = INFINITY;

        // for all s in S, rhs(s) = inf and g(s) = inf
        for (auto x : ROW) {
                for(auto y : COL) {

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
        goal.k = calculateKey(goal_node,start_node,km);
        // insert goal_node to open_list
        open_list.insert(goal_node);
        open_hash.insert(std::make_pair<goal_node.index,goal_node>);
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
        if ( currentnode != goal_node) {
                double tem = INFINITY;
                td::vector<Node> successor_list;
                successor_list = successor_node_list(currentnode);
                for (auto node : successor_list) {
                        // calculate c(u,s') + g(s')
                        double new_tem = node.g +cost_between_nodes(node,currentnode);
                        if(tem_new < tem)
                                tem = tem_new;
                }
                node_map[currentnode.index].rhs = tem_new;
        }
        // lazy remove the open_hash if currentnode belongs to open_list. Line 8
        std::priority_queue<Node,std::vector<Node> > tem_list;
        if(open_hash.find(currentnode.index) != open_hash.end()) {
                open_hash.erase(open_hash.find(currentnode.index));
                while(!open_list.empty()) {
                        Node topNode = open_list.top();
                        open_list.pop();
                        if(open_hash.find(topNode.index)!= open_hash.end())
                                tem_list.insert(topNode);
                }
                open_list = tem_list;
        }

        if(AreSame(currentnode.g, currentnode.rhs)) {
                currentnode.k = calculateKey(currentnode,last_node,km);
                open_list.insert(currentnode);
                open_hash.insert(std::make_pair<currentnode.index,currentnode>);
        }



}

std::vector<Node> DstarLite::successor_node_list(Node currentnode)  {
        std::vector<Node> successor_list;
        for(int i = -1; i < 2; i++) {
                for (int j = -1; j < 2; j++) {
                        int new_x = currentnode.x + i;
                        int new_y = currentnode.y + j;
                        successor_list.push_back(node_map[computeIndex(new_x, new_y, COL)]);
                }
        }
        return successor_list;
}

double DstarLite::cost_between_nodes(Node a, Node b) {
        if ()
                return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

double DstarLite::heuristic(Node a, Node b) {
        return std::sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
}

bool DstarLite::AreSame(double a, double b)
{
        return fabs(a - b) < 0.01;
}

std::vector<Node> DstarLite::predecessor_node_list(Node currentnode,std::vector<int> map)  {

}

std::vector<int> DstarLite::computeShortestPath() {

        while(open_list.top() < start_node || !AreSame(start_node.rhs,start_node.g)) {
                std::pair<double,double> key_old;
                node topNode = open_list.top();
                key_old = topNode.k;
                open_list.pop();
                // line 13
                if(isKeySmaller(key_old,calculateKey(topNode,last_node,km))) {
                        // line 14
                        topNode.k = calculateKey(topNode,last_node,km);
                        open_list.insert(topNode);
                }
                // line 15
                else if(topNode.h > topNode.rhs) {
                        topNode.g = topNode.rhs;

                }
        }

}
bool isKeySmaller(std::pair<double,double> p1,std::pair<double,double> p2) {
        if (p1.first < p2.first) return true;
        else if (p1.first > p2.first) return false;
        return p1.second < p2.second + 0.01;
}

Node DstarLite::Find_min_Node(const std::vector<Node>& successor_nodes) {

}

bool DstarLite::isNodeValid(Node start_node,std::vector<int> map){

}
