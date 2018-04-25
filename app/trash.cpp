#include <tr1/unordered_map>


int main() {
        int a = 1;
        int b = 2;
        std::pair<int,int> node_pair = std::make_pair(a,b);
        std::tr1::unordered_map< double,std::pair<int,int> > cost_between_nodes_map;
        cost_between_nodes_map.insert(std::make_pair(3.23,node_pair));
        return 0;
}
