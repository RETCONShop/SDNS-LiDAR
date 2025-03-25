/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        Serial.h
// Purpose:     A first attempt revision of creating a serialization library from scratch for the
//              SDNS-GUI project
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.24.24 5:35 PM MT: Comments added
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


// This library doesn't really serve much of a purpose. There's a few good functions, but it's safe to pretty much ignore.
// I left it here for you to get ideas on where to take the project, and methods to avoid (specifically the spaghetti that is 
// serializeCoordianteData)
#include <iostream>
#include <vector>
#include <cstring>

struct CoordinateData {
    std::vector<double> xData;
    std::vector<double> yData;
    int numberOfPoints;
};

std::string vectorToString(const std::vector<char>& vec) {
    return std::string(vec.begin(), vec.end());
}

std::vector<char> stringToVector(const std::string& str) {
    return std::vector<char>(str.begin(), str.end());
}

void printCoordinateData(CoordinateData data) {
    printf("Coordinate Data:\n");
    printf("X Data: ");
    for (int i = 0; i < data.numberOfPoints; i++) {
        printf("%.1f ", data.xData[i]);
    }
    printf("\nY Data: ");
    for (int i = 0; i < data.numberOfPoints; i++) {
        printf("%.1f ", data.yData[i]);
    }
    printf("\nNumber of Points: %d\n", data.numberOfPoints);
}

std::vector<char> serializeCoordinateData(const CoordinateData& data) {
    std::vector<char> serializedData;

    // Serialize the xData and yData vectors
    int32_t numPoints = static_cast<int32_t>(data.xData.size());
    serializedData.resize(sizeof(int32_t) + numPoints * sizeof(double) * 2);

    char* ptr = serializedData.data();
    memcpy(ptr, &numPoints, sizeof(int32_t));
    ptr += sizeof(int32_t);

    memcpy(ptr, data.xData.data(), numPoints * sizeof(double));
    ptr += numPoints * sizeof(double);

    memcpy(ptr, data.yData.data(), numPoints * sizeof(double));

    // Serialize the numberOfPoints field
    memcpy(serializedData.data() + serializedData.size() - sizeof(int32_t),
           &data.numberOfPoints, sizeof(int32_t));

    return serializedData;
}

CoordinateData deserializeCoordinateData(const std::vector<char>& serializedData) {
    CoordinateData data;

    // Deserialize the numberOfPoints field
    int32_t numPoints;
    memcpy(&numPoints, serializedData.data(), sizeof(int32_t));

    // Deserialize the xData and yData vectors
    data.xData.resize(numPoints);
    data.yData.resize(numPoints);

    const char* ptr = serializedData.data() + sizeof(int32_t);
    memcpy(data.xData.data(), ptr, numPoints * sizeof(double));
    ptr += numPoints * sizeof(double);
    memcpy(data.yData.data(), ptr, numPoints * sizeof(double));

    // Deserialize the numberOfPoints field
    memcpy(&data.numberOfPoints, serializedData.data() + serializedData.size() - sizeof(int32_t),
           sizeof(int32_t));

    return data;
}
