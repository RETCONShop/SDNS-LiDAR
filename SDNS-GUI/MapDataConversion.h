/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        MapDataConversion.h
// Purpose:     A simple library for converting LiDAR data to XY map points
//              for SNDS-SNL Capstone
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.24.24 6:19 PM MT: Comments added
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
#ifndef MAPDATACONVERSION_H
#define MAPDATACONVERSION_H

#include "Structs.h"
#include <cmath>

/*
 * Class name:
 *   - MapDataConversion
 * Description:
 *   - This class provides functions for converting Lidar data to XY map points.
 */
class MapDataConversion {
public:
    /*
     * Function name:
     *   - convertToXYMap
     * Inputs:
     *   - MapData data: the input MapData struct containing Lidar data
     * Outputs:
     *   - std::vector<MapPoint>: a vector of MapPoint structs representing XY map points
     * Description:
     *   - Converts the Lidar data in the input MapData struct to XY map points.
     */
    std::vector<MapPoint> convertToXYMap(MapData data) {
        // Allocate memory for an array of structs
        int numberOfElements = 5000;
        std::vector<MapPoint> points;

        double angle;
        double thetaConv = 0.0;
        MapPoint point = {0, 0};

        for (int id : data.r) {

            //convert Lidar theta from degrees to radians before summing angle for map
            thetaConv = (-1*data.theta[id] - 90) * halfC;

            data.theta[id] = thetaConv;

            //angle of reading in base frame is the sum of the angle from the Kalman filter and LiDAR, respectively
            //angle = data.phi + data.theta[id];
            angle = data.theta[id];

            //note that (to my knowledge) lidar reads in degrees and kalman angle is given in radians; conversion of one is necessary

            //"horizontal" distance in base frame is lidar x (in active frame) + bot position x
            point.x = data.r[id]*cos(angle) + data.x;

            //"vertical" distance in base frame is lidar y (in active frame) + bot position y
            point.y = data.r[id]*sin(angle) + data.y;

            // Append the new point to the vector
            points.push_back({point.x, point.y});
        }

        return points;
    }

    /*
     * Function name:
     *   - convertToXYMapNew
     * Inputs:
     *   - MapData& data: a reference to the input MapData struct containing Lidar data
     * Outputs:
     *   - MapPointsQVector: a struct containing vectors of x and y data representing XY map points
     * Description:
     *   - Updated version of the convertToXYMap function, works directly with an existing MapData struct.
     */
    MapPointsQVector convertToXYMapNew(MapData& data) {
        // Local variables
        double angle = 0.0;
        MapPointsQVector cData;

        // Loop through the struct, based on the size of R.
        // Both R and Theta will be the same length, so this is fine
        for (int id = 0; id < data.r.size(); ++id) {
            data.theta[id] = (-1*data.theta[id] - 90) * halfC;
            angle = data.phi + data.theta[id];

            // Push x and y data to the CoordinateData struct
            cData.xData.push_back(data.r[id]*cos(angle) + data.x);
            cData.yData.push_back(data.r[id]*sin(angle) + data.y);
        }

        // Return the new CoordinateData struct
        return cData;
    }

private:
    const double halfC = M_PI / 180; // Constant for converting degrees to radians

};
#endif // MAPDATACONVERSION_H
