/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        LightSerial.h
// Purpose:     A simple processing library for converting structs for SDNS-SNL
//              Capstone into strings
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
#include <iostream>
#include <vector>
#include <random>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

// Custom classes
#include "Structs.h"

/*
 * For keeping everything isolated to this one header file, you can copy the two structs from "Structs.h"
 * below the "public:" section of the class. For simplicity, I have all the structs in the "Structs.h" file
 * as that is what the server side of this project expects.
 */ 

class LightSerial {
// Public functions and variables
public:

/*
 * Function name: 
 *   - generateRandomArray
 * Inputs: 
 *   - int numPoints: the number of random points to generate
 *   - int numDecimalPlaces: the number of decimal places to round the random numbers to
 *   - double lowerBound: the lower bound of the random number range
 *   - double upperBound: the upper bound of the random number range
 * Outputs: 
 *   - std::vector<double>: a vector of randomly generated numbers
 * Description: 
 *   - This function generates a vector of random numbers within the specified range and rounds them to the specified 
 *     number of decimal places.
 */
std::vector<double> generateRandomArray(int numPoints, int numDecimalPlaces, double lowerBound, double upperBound) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(lowerBound, upperBound);

    std::vector<double> randomArray(numPoints);

    for (int i = 0; i < numPoints; i++) {
        double randomNumber = dis(gen);
        randomArray[i] = round(randomNumber * pow(10, numDecimalPlaces)) / pow(10, numDecimalPlaces);
    }

    return randomArray;
}

/*
 * Function name:
 *   - readFileToString
 * Inputs:
 *   - const char* filename: the name of the file to read
 * Outputs:
 *   - char*: a char* array of the file that's been read
 * Description:
 *   - This function will read the entire contents of a file (any extension type) into an ASCII char* array
 */
char* readFileToString(const char* filename) {
    // Local variables
    FILE* pFile;
    long lSize;
    char* buffer;

    // Open the file in read mode
    pFile = fopen(filename, "rb");
    if (!pFile) {
        std::cout << "Error opening file: " << filename << std::endl;
        return NULL;
    }

    // Determine the size of the file
    fseek(pFile, 0L, SEEK_END);
    lSize = ftell(pFile);
    rewind(pFile);

    // Allocate memory to hold the file contents
    buffer = (char*)malloc(sizeof(char) * (lSize + 1));
    if (!buffer) {
        std::cout << "Memory allocation failed!" << std::endl;
        fclose(pFile);
        return NULL;
    }

    // Read the file contents into the buffer
    if (fread(buffer, 1, lSize, pFile) != lSize) {
        std::cout << "Error reading file!" << std::endl;
        free(buffer);
        fclose(pFile);
        return NULL;
    }

    // Null terminate the string
    buffer[lSize] = '\0';

    // Close the file and return the buffer
    fclose(pFile);
    return buffer;
}

/*
 * Function name:
 *   - writeStringToFile
 * Inputs:
 *   - const char* filename: the name of the file to read
 *   - const std::string& content: a string passed pending writing to file
 * Outputs:
 *   - None
 * Description:
 *   - This function will take a file name as an argument, the write the contents of the string passed as an argument to
 *     the file name provided (same directory as the compiled binary)
 */
void writeStringToFile(const char* filename, const std::string& content) {
    std::ofstream file(filename, std::ios::out);
    if (file.is_open()) {
        file << content;
        file.close();
    } else {
        std::cout << "Error opening file: " << filename << std::endl;
    }
}

/*
 * Function name: 
 *   - printCoordinateData
 * Inputs: 
 *   - const CoordinateData& data: A reference to the CoordinateData struct containing the coordinate data to be printed.
 * Outputs: 
 *   - None
 * Description: 
 *   - This function takes a CoordinateData struct and prints its contents in a user-friendly format to the console. 
 *     It displays the number of data points, the X data, and the Y data, with each value formatted to 2 decimal places.
 */
void printCoordinateData(const CoordinateData& data) {
    std::cout << "Coordinate Data:" << std::endl;
    std::cout << "Number of Points: " << data.numberOfPoints << std::endl;
    std::cout << std::fixed << std::setprecision(8); // Set output precision to 2 decimal places

    std::cout << "Coordinates:" << std::endl;
    for (size_t i = 0; i < data.xData.size(); ++i) {
        std::cout << "  [" << i << "]: " << data.xData[i] << ", " << data.yData[i] << std::endl;
    }
}

/*
 * Function name: 
 *   - printMapData
 * Inputs: 
 *   - const MapData& data: A reference to the MapData struct containing the Lidar data, robot position, and varience associated
 * Outputs: 
 *   - None
 * Description: 
 *   - This function takes a MapData struct and prints its contents in a user-friendly format to the console. 
 *     It displays the R and Theta data from the Lidar, the robot's position in X,Y coordinates, and the varience
 *     for each piece of data.
 */
void printMapData(const MapData& data) {
    std::cout << "Map Data:" << std::endl;
    std::cout << std::fixed << std::setprecision(8); // Set output precision to 8 decimal places

    // Print all the non vector stuff first
    std::cout 
        << "Robot Position: " << data.x << ", " << data.y << std::endl
        << "Robot Phi: " << data.phi << std::endl
        << "Varience of robot position: " << data.var_x << ", " << data.var_y << std::endl
        << "Varience of robot phi: " << data.var_phi << std::endl;
    
    // Print the vector stuff
    std::cout << "Lidar data (r, theta):" << std::endl;
    for (size_t i = 0; i < data.r.size(); ++i) {
        std::cout << "  [" << i << "]: " << data.r[i] << ", " << data.theta[i] << std::endl;
    }
    /*
    std::cout << "Coordinates:" << std::endl;
    for (size_t i = 0; i < data.xData.size(); ++i) {
        std::cout << "  [" << i << "]: " << data.xData[i] << ", " << data.yData[i] << std::endl;
    }*/
}

/*
 * Function name:
 *   - writeCoordinateDataToFile
 * Inputs:
 *   - const CoordinateData& data: A reference to the CoordinateData struct for processing
 *   - const char* filename: The file name that the function will write the contents of the struct to
 * Outputs:
 *   - None
 * Description:
 *   - This function takes a reference to the CoordinateData struct, and the name of the file to save the
 *     struct's contents to. This function does not preserve the numberOfPoints value within the struct, 
 *     opting to reassemble this later when reading the file for import.
 */
void writeCoordinateDataToFile(const CoordinateData& data, const char* filename) {
    std::cout << "Writing CoordinateData struct to file..." << std::endl;

    // Create new file indicator
    FILE * pFile;
    pFile = fopen(filename, "w");
    if (pFile == NULL) {
        std::cout << "Error opening file!" << std::endl;
        return; 
    } else {
        for (int i = 0; i < data.numberOfPoints; i++) {
            fprintf(pFile, "%.8f %.8f\n", data.xData[i], data.yData[i]);
        }
    }
    fclose(pFile);
}

/*
 * Function name:
 *   - readCoordinateDataFromFile
 * Inputs:
 *   - const char* filename: the name of the file to read
 * Outputs:
 *   - CoordinateData: the newly created CoordinateData struct assembled from the file read
 * Description:
 *   - This function will read the contents of a file provided by 'filename', them reassemble a new
 *     CoordinateData struct. The file contents is expected to only have the xData and yData values
 *     in the format of vector doubles. Each line is formatted as: 'xData yData'. The 'numberOfPoints'
 *     value is rebuilt from the number of reads from the file.
 */
CoordinateData readCoordinateDataFromFile(const char* filename) {
    CoordinateData data;
    data.numberOfPoints = 0;
    FILE* pFile;

    // Open the file in read mode
    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        printf("Error opening file.\n");
        return data;
    }

    // Read the coordinates from the file
    char line[100];
    while (fgets(line, sizeof(line), pFile) != NULL) {
        double x, y;
        if (sscanf(line, "%lf %lf", &x, &y) == 2) {
            data.xData.push_back(x);
            data.yData.push_back(y);
            data.numberOfPoints++;
        }
    }

    // Close the file
    fclose(pFile);

    return data;
}

/*
 * Function name:
 *   - writeMapDataToFile
 * Inputs:
 *   - const mapData& data: a reference to an existing MapData struct
 *   - const char* filename: the name of the file to read
 * Outputs:
 *   - None
 * Description:
 *   - This function takes a reference to an exsting MapData struct, as well as a file name, then writes 
 *     the contents of the struct to a file. It writes all the non-vectors in the first line, then begins
 *     to read the two vectors for R and Theta (stored next to eachother, much like writeCoordinateDataToFile)
 */
void writeMapDataToFile(const MapData& data, const char* filename) {
    std::cout << "Writing MapData struct to file..." << std::endl;

    // Create new file indicator
    FILE * pFile;
    pFile = fopen(filename, "w");
    if (pFile == NULL) {
        std::cout << "Error opening file!" << std::endl;
        return;
    } else {
        // Print all doubles first
        fprintf(pFile, "%.8f %.8f %.8f %.8f %.8f %.8f\n", data.x, data.y, data.phi, data.var_x, data.var_y, data.var_phi);

        // Write the two vectors, R and Theta
        for (int i = 0; i < data.r.size(); i++) {
            fprintf(pFile, "%.8f %.8f\n", data.r[i], data.theta[i]);
        }
    }

    // Close the file
    fclose(pFile);
}

/*
 * Function name:
 *   - readMapDataFromFile
 * Inputs:
 *   - const char* filename: the name of the file to read
 * Outputs:
 *   - MapData: a new MapData struct with the processed contents
 * Description:
 *   - This function reads a file provided by 'filename' and reassembles a new MapData struct. Much like it's
 *     counterpart 'writeMapDataToFile', it first reads the entire first line, gathering all non vector data.
 *     Then, it will read any following lines into the two vectors, for R and Theta (similarly to the
 *     readCoordinateDataFromFile function).
 */
MapData readMapDataFromFile(const char* filename) {
    MapData data;
    std::cout << "Reading MapData struct from file..." << std::endl;

    // Create new file indicator
    FILE * pFile;

    // Open the file in read mode
    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        printf("Error opening file.\n");
        return data;
    } else {
        printf("File opened successfully!\n");
    }

    // Get the first line of data (6 doubles)
    char line[100];
    size_t len = 0;

    // Get the first line
    fgets(line, sizeof(line), pFile);

    // Process the first line
    sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &data.x, &data.y, &data.phi, &data.var_x, &data.var_y, &data.var_phi);

    // Get the next line
    fgets(line, sizeof(line), pFile);

    // Process the next line manually
    double r, theta;
    sscanf(line, "%lf %lf\n", &r, &theta);
    data.r.push_back(r);
    data.theta.push_back(theta);

    // Loop through the rest of the file
    while (fgets(line, sizeof(line), pFile) != NULL) {
        // Read the arrays
        if (sscanf(line, "%lf %lf\n", &r, &theta) == 2) {
            data.r.push_back(r);
            data.theta.push_back(theta);
        }
    }

    /* Old code (backup)
    // Create new file indicator
    FILE * pFile;

    // Open the file in read mode
    pFile = fopen(filename, "r");
    if (pFile == NULL) {
        printf("Error opening file.\n");
        return data;
    } else {
        printf("File opened successfully!\n");
    }

    // Read the coordinates from the file
    char line[100];
    // Read the first line of doubles
    sscanf(line, "%lf %lf %lf %lf %lf %lf\n", &data.x, &data.y, &data.phi, &data.var_x, &data.var_y, &data.var_phi);

    fscanf(pFile, "%*[^\n]\n");

    while (fgets(line, sizeof(line), pFile) != NULL) {
        // Read the arrays
        double r, theta;
        if (sscanf(line, "%lf %lf\n", &r, &theta) == 2) {
            data.r.push_back(r);
            data.theta.push_back(theta);
        }
    }

    // Close the file
    fclose(pFile);
    */

    return data;
}

/*
 * Function name:
 *   - generateRandomMapData
 * Inputs:
 *   - int points: the number of points for R and Theta
 *   - int precision: the desired precision for the random number generator
 *   - double lowerBound: the lower bound for the random number generator
 *   - double upperBound: the upper bound for the random number generator
 * Outputs:
 *   - MapData: a new MapData struct containing completely random data based on provided specifications
 * Description:
 *   - This function is to generate a completely random MapData struct for testing purposes. MapData has
 *     this function whereas CoordinateData does not due to the complexity of the MapData struct. Random
 *     numbers can be assigned manually for testing purposes, though this automates that process.
 */
MapData generateRandomMapData(int points, int precision, double lowerBound, double upperBound) {
    // Map Data struct definition
    MapData data;

    // Define the random device
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(lowerBound, upperBound);

    data.x = dis(gen);
    data.y = dis(gen);
    data.phi = dis(gen);
    data.var_x = dis(gen);
    data.var_y = dis(gen);
    data.var_phi = dis(gen);
    data.r = generateRandomArray(points, precision, lowerBound, upperBound);
    data.theta = generateRandomArray(points, precision, lowerBound, upperBound);
    return data;
}
};
