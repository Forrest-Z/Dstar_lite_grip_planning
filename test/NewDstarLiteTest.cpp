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

struct DstarLiteTest : public ::testing::Test {
        Buildingmap::coordinate start = setmap.SetStart(2, 3);
        Buildingmap::coordinate goal = setmap.SetGoal(7, 7);
        Buildingmap::gridMatrix gridMap_old =  {
                {
                        {1,1,1,1,1,1,1,1,1,1},
                        {1,1,0,0,0,0,0,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,0,0,0,0,0,0,1,1},
                        {1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1}
                }
        };
        Buildingmap::gridMatrix gridMap_old =  {
                {
                        {1,1,1,1,1,1,1,1,1,1},
                        {1,1,0,0,0,0,0,0,1,1},
                        {1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,1,1,1,1,1,0,1,1},
                        {1,1,0,0,0,0,0,0,1,1},
                        {1,1,1,1,1,1,1,1,1,1},
                        {1,1,1,1,1,1,1,1,1,1}
                }
        };
        DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
};

TEST_F(DstarLiteTest, calculateKeyTest) {
        Node
        calculateKey(const Node &currentnode,const Node &start_node, const double& km);
}
