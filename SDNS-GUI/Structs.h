/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        Structs.cpp
// Purpose:     A simple library containing required structs for the SDNS-SNL capstone project
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.5.24 7:18 PM MT: Version 1 finalized
//
// Copyright 2024 Andrew Gallagher
// 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
// 
//      http://www.apache.org/licenses/LICENSE-2.0
// 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/////////////////////////////////////////////////////////////////////////////////////////////////////

// Required includes
#include <vector>
#include <QVector>

#ifndef STRUCTS_H
#define STRUCTS_H

// Structs go here

// Coordinate data struct for storaging X and Y coordinates (Server side)
struct CoordinateData {
    // Vector for X coordinates
    std::vector<double> xData;
    // Vector for Y coordinates
    std::vector<double> yData;
    // Total number of points (in pairs, X&Y)
    int numberOfPoints;
};

// Map data struct that is streamed from the robot
struct MapData {
    // Actual map data from the robot (with respect to it's current position)
    std::vector<double> r;    
    std::vector<double> theta;
    // Robot position X
    double x;     
    // Robot position Y
    double y;   
    // Angle given in radians, orientation of the robot
    double phi;   
    // Varience of X coordinate
    double var_x; 
    // Varience of Y coordinate
    double var_y; 
    // Varience of angle given in radians
    double var_phi; 
};

// A simple x, y coordinate point for the graph
struct MapPoint {
    double x, y; //point in base frame
};

// A simple a, b struct containing the calculated error for each scan
struct ErrorSize {
    double a, b;
};

// A simple x,y coordinate struct using QVectors for direct plotting
struct MapPointsQVector {
    QVector<double> xData;
    QVector<double> yData;
};

#endif // STRUCTS_H

