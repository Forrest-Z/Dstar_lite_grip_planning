/**
 * @Author: Zejiang Zeng <yzy>
 * @Date:   2018-03-12T20:29:41-04:00
 * @Email:  zzeng@terpmail.umd.edu
 * @Filename: main.cpp
 * @Last modified by:   yzy
 * @Last modified time: 2018-03-27T09:21:10-04:00
 * @Copyright: (C) 2017 Zejiang Zeng - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD license.
 */

#include <iostream>
#include <array>
#include <opencv2/opencv.hpp>
#include "Map.hpp"
#include "BuildingMap.hpp"
#include "dstar.h"
#include "NewDstarLite.h"

/**
 * [main The main function for the grid based D* lite algorithm impletation]
 * @return [start and goal position are set here, and Map is set Map class, and a video
 * which contains the dynamic path will be generate through the DsratLite class]
 */
int main() {
        Map setmap;
        // Set the inital and changed map
        Map::gridMatrix map_old = setmap.getGridmap(2);
        Map::gridMatrix map_new = setmap.getGridmap(6);
        // Set the goal and end point
        Buildingmap::coordinate start = setmap.SetStart(8, 0);
        Buildingmap::coordinate goal = setmap.SetGoal(5, 9);
        // Create a video writer
        cv::VideoWriter video("outcpp.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10,
                              cv::Size(100, 100));
        // Construct the DstarLite class
        DstarLite dstar(map_old,map_new,start.first,start.second,goal.first,goal.second);
        dstar.video = video;
        std::cout<<"replan"<<std::endl;
        // Dynamic replan the according to the environment (map) change
        dstar.replan();
        // Display the final map
        cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
        cv::imshow( "Display window", dstar.Map);
        cv::waitKey(1000);
        return 0;
}
