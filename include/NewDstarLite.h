
#include <vector>
#include <queue>
#include <tr1/unordered_map>
#include "BuildingMap.hpp"

class Node {
public:
int x;
int y;
int index;
double g;
double rhs;
double min_cost_succ;
std::pair<double,double> k;
int parent_index;
Node (const int& x, const int& y, const double& g, const double& rhs) : x(x), y(y), g(g), rhs(rhs) {
        // initialize min_cost_succ
        min_cost_succ = -1;
}
Node() {
}
bool operator == (const Node &s2) const {
        return ((x == s2.x) && (y == s2.y));
}

bool operator != (const Node &s2) const {
        return ((x != s2.x) || (y != s2.y));
}

bool operator > (const Node &s2) const {
        if (k.first-0.00001 > s2.k.first) return true;
        else if (k.first < s2.k.first-0.00001) return false;
        return k.second > s2.k.second;
}

bool operator <= (const Node &s2) const {
        if (k.first < s2.k.first) return true;
        else if (k.first > s2.k.first) return false;
        return k.second < s2.k.second + 0.00001;
}


bool operator < (const Node &s2) const {
        if (k.first + 0.000001 < s2.k.first) return true;
        else if (k.first - 0.000001 > s2.k.first) return false;
        return k.second < s2.k.second;
}
};

class DstarLite {
// Constructor
public:
DstarLite(Buildingmap::gridMatrix gridmap_old,Buildingmap::gridMatrix gridmap_new,int start_x,int start_y,int goal_x,int goal_y);
std::vector<int> map_info_old;
std::vector<int> map_info_new;


private:
double infinity = std::numeric_limits<double>::infinity();
Node start_node;
Node goal_node;
Node last_node;
Node next_node; // the node has smallest c(current,s') + g(s')
double km = 0;
std::priority_queue<Node,std::vector<Node> > open_list;
std::tr1::unordered_map<int, Node > open_hash;
std::tr1::unordered_map<int, Node > node_map;
std::tr1::unordered_map<std::pair<int,int>, double > cost_between_nodes_map;
std::vector<Node> successor_node_list(Node currentnode);
std::vector<Node> predecessor_node_list(Node currentnode);

void initialize();
double cost_between_nodes(Node a, Node b);
double heuristic(Node a, Node b);
std::vector<int> computeShortestPath();
double min(const double& g, const double& rhs);
Node Find_min_Node(const std::vector<Node>& successor_nodes);
bool isNodeValid(Node node);
int computeIndex(const int& x, const int& y, const int& column_size);
void nodeIndexing(Node& node);
void addNodeToMap(const Node& node, std::tr1::unordered_map<int, Node >& node_map);
void calculateKey(Node currentnode);
std::pair<double,double> calculateKey(const Node& currentnode,
                                      const Node& start_node);
bool isKeySmaller(std::pair<double,double> p1,std::pair<double,double> p2);
void updateVertex(Node currentnode);
void Scan_for_cost_change(Node currentnode);
void update_nodes_cost(Node currentnode);
bool AreSame(double a, double b);
void replan();
};
