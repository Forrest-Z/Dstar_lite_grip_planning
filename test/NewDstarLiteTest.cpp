/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-04-21T17:44:01-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: NewDstarLiteTest.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-04-21T17:44:08-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */
#include <gtest/gtest.h>
#include "NewDstarLite.h"
#include "BuildingMap.hpp"
#include <array>
#include <cmath>
#include "Map.hpp"

struct DstarLiteTest : public ::testing::Test {
	const double infinity = std::numeric_limits<double>::infinity();
	    Map setmap;
        Buildingmap::coordinate start = setmap.SetStart(2, 3);
        Buildingmap::coordinate goal = setmap.SetGoal(8, 6);
        Map::gridMatrix map_old = setmap.getGridmap(5);
        Map::gridMatrix map_new = setmap.getGridmap(1);

};

TEST_F(DstarLiteTest, constructorTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
	EXPECT_EQ(32,dstar.start_node_index);
	EXPECT_EQ(68,dstar.goal_node_index);
	EXPECT_EQ(100,dstar.map_info_old.size());
	EXPECT_EQ(100,dstar.map_info_new.size());
	EXPECT_EQ(100,dstar.node_map.size());
	EXPECT_DOUBLE_EQ(infinity,dstar.node_map[dstar.goal_node_index]->g);
}

TEST_F(DstarLiteTest, isKeySmallerTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
     std::pair<double,double> p1 = std::make_pair(3.2,4.3);
     std::pair<double,double> p2 = std::make_pair(3.7,7.3);
     std::pair<double,double> p3 = std::make_pair(3.7,10.3);
     EXPECT_TRUE(dstar.isKeySmaller(p1,p2));
     EXPECT_FALSE(dstar.isKeySmaller(p2,p1));
     EXPECT_TRUE(dstar.isKeySmaller(p2,p3));

}
TEST_F(DstarLiteTest, calculateKeyTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
    auto key =  dstar.calculateKey(dstar.node_map[dstar.goal_node_index]);
    EXPECT_EQ(0,key.second);
    EXPECT_EQ(dstar.heuristic(dstar.node_map[dstar.start_node_index],dstar.node_map[dstar.goal_node_index]),key.first);
    EXPECT_EQ(1,dstar.open_list.size());
    EXPECT_EQ(1,dstar.open_hash.size());
}

TEST_F(DstarLiteTest, predecessorTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
	std::vector<std::shared_ptr<Node> > predecessor_list_start = dstar.predecessor_node_list(dstar.node_map[dstar.start_node_index]);
	std::vector<std::shared_ptr<Node> > predecessor_list_goal = dstar.predecessor_node_list(dstar.node_map[39]);
	EXPECT_EQ(8,predecessor_list_start.size());
	EXPECT_EQ(5,predecessor_list_goal.size());

}

TEST_F(DstarLiteTest, sucessorTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
	std::vector<std::shared_ptr<Node> > successor_list_start = dstar.successor_node_list(dstar.node_map[dstar.start_node_index]);
	std::vector<std::shared_ptr<Node> > successor_list_goal = dstar.successor_node_list(dstar.node_map[dstar.goal_node_index]);
	EXPECT_EQ(8,successor_list_start.size());
	EXPECT_EQ(8,successor_list_goal.size());

}

TEST_F(DstarLiteTest,cost_between_nodesTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
	EXPECT_DOUBLE_EQ(infinity,dstar.cost_between_nodes(dstar.node_map[22],dstar.node_map[12]));
	EXPECT_DOUBLE_EQ(10,dstar.cost_between_nodes(dstar.node_map[dstar.start_node_index],dstar.node_map[22]));
	EXPECT_DOUBLE_EQ(10*std::sqrt(2),dstar.cost_between_nodes(dstar.node_map[dstar.start_node_index],dstar.node_map[21]));
}

TEST_F(DstarLiteTest,computePathTest) {
	DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
	                std::pair<double,double> key_old;
	                std::shared_ptr<Node> topNode = dstar.open_list.top();
	                key_old = topNode->k;
	                dstar.open_list.pop();
	                int choice;
	                std::pair<double,double> key_new = dstar.calculateKey(topNode);
	                // line 13
	                if(dstar.isKeySmaller(key_old,key_new)) {
	                        // line 14
	                	choice = 1;
	                        topNode->k = dstar.calculateKey(topNode);
	                        dstar.open_list.push(topNode);
	                }
	                // line 15
	                else if(topNode->g > topNode->rhs) {
	                	choice = 2;
	                        topNode->g = topNode->rhs;
	                        std::vector<std::shared_ptr<Node> > predecessor_list = dstar.predecessor_node_list(topNode);
	                        for(auto node : predecessor_list) {
	                                dstar.updateVertex(node);
	                        }
	                }
	                else {
	                	 choice = 3;
	                        topNode->g = infinity;
	                        std::vector<std::shared_ptr<Node>> predecessor_list = dstar.predecessor_node_list(topNode);
	                        for(auto node : predecessor_list) {
	                                dstar.updateVertex(node);
	                        }
	                        dstar.updateVertex(topNode);
	                }
	                EXPECT_EQ(2,choice);
	                EXPECT_EQ(68,topNode->index);
	                EXPECT_DOUBLE_EQ(0,dstar.node_map[68]->g);
	                EXPECT_EQ(8,dstar.open_list.size());
	                EXPECT_DOUBLE_EQ(10,dstar.node_map[69]->rhs);
	                EXPECT_DOUBLE_EQ(10+dstar.heuristic(dstar.node_map[69],dstar.node_map[32]),dstar.node_map[69]->k.first);
	                EXPECT_EQ(58,dstar.open_list.top()->index);
	                std::vector<std::shared_ptr<Node>> successor_list = dstar.successor_node_list(dstar.node_map[dstar.goal_node_index]);
	                	 unsigned int start_node_index = dstar.Find_min_Node(successor_list);
	                	 EXPECT_EQ(58,start_node_index);
}

TEST_F(DstarLiteTest,Scan_for_cost_changeTest) {
	EXPECT_TRUE(map_old[37] != map_new[37]);
}
