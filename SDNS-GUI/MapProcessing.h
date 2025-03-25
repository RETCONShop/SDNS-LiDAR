/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        MapProcessing.h
// Purpose:     A simple library for creating, reading, appending, and clearing CSV
//              files containing map data for SNDS-SNL Capstone
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

#ifndef MAPPROCESSING_H
#define MAPPROCESSING_H

// Library includes
#include <filesystem>	// Filesystem operations
#include <iostream>		// Standard input/output stream objects
#include <fstream>		// File stream objects
#include <sstream>		// String stream classes
#include <vector>		// Dynamic array class
#include <string>		// String class
#include <QDebug>		// Debugging output messages for QT
#include <random>		// Random number generator

// File names
std::string mainMapFileName = "mainmap.csv";
std::string bufferMapFileName = "buffer.csv";

// Struct for containing map data
struct MapStruct {
    QVector<double> xData;
    QVector<double> yData;
};

/*
 * Function name: 
 *   - createBlankCSVFile
 * Inputs: 
 *   - const std::string& filename: the name of the CSV file to create
 * Outputs: 
 *   - None
 * Description: 
 *   - Creates a blank CSV file with the specified filename in the 'WorkingDir' directory.
 */
void createBlankCSVFile(const std::string& filename) {
    std::filesystem::create_directory("WorkingDir"); // Creates a directory if it doesnt exist
    std::ofstream file("WorkingDir/" + filename); // Open a file for writing

    if (file.is_open()) {
        file << ""; // Writing an empty string to create a blank file
        file.close();
        qInfo() << "Blank CSV file '" << filename << "' created successfully.";
    } else {
        qCritical() << "Error creating CSV file.";
    }
}

/*
 * Function name: 
 *   - initializeFileStructure
 * Inputs: 
 *   - None
 * Outputs: 
 *   - None
 * Description: 
 *   - Initializes the file structure by creating two blank CSV files: 'mainmap.csv' and 'buffer.csv'.
 */
void initializeFileStructure() {
    createBlankCSVFile(mainMapFileName);
    createBlankCSVFile(bufferMapFileName);
}

/*
 * Function name: 
 *   - readCSVFile
 * Inputs: 
 *   - const std::string& filename: the name of the CSV file to read
 *   - QVector<double>& vector1: reference to the vector to store the first column data
 *   - QVector<double>& vector2: reference to the vector to store the second column data
 * Outputs: 
 *   - None
 * Description: 
 *   - Reads the contents of the specified CSV file and extracts the data into two vectors.
 */
void readCSVFile(const std::string& filename, QVector<double>& vector1, QVector<double>& vector2) {
    std::ifstream file("WorkingDir/" + filename); // Open file for reading
    if (!file.is_open()) {
        qCritical() << "Error opening file: " << filename;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value1, value2;

        if (std::getline(ss, value1, ',') && std::getline(ss, value2)) {
            double num1 = std::stod(value1);
            double num2 = std::stod(value2);
            vector1.push_back(num1);
            vector2.push_back(num2);
        }
    }

    file.close();
    qDebug() << "CSV file '" << filename << "' read successfully.";
}

/*
 * Function name: 
 *   - generateRandomVectorsAndSave
 * Inputs: 
 *   - const std::string& filename: the name of the CSV file to save the random vectors
 *   - int numPoints: the number of random points to generate
 * Outputs: 
 *   - None
 * Description: 
 *   - Generates random vectors with the specified number of points and saves them to the specified CSV file.
 */
void generateRandomVectorsAndSave(const std::string& filename, int numPoints) {
    std::ofstream file("WorkingDir/" + filename); // Open file for writing

    if (file.is_open()) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(1.0, 9.0); // Uniform distribution between 1.0 and 9.0

        std::vector<double> vector1, vector2;

        qDebug() << "Generating random vectors..";
        for (int i = 0; i < numPoints; ++i) {
            vector1.push_back(dis(gen)); // Generate random numbers for x
            vector2.push_back(dis(gen)); // Generate random numbers for y
        }

        for (int i = 0; i < numPoints; ++i) {
            file << vector1[i] << "," << vector2[i] << "\n";
        }

        file.close();
        qDebug() << "Random vectors saved to CSV file '" << filename << "' successfully.";
    } else {
        qCritical() << "Error opening CSV file.";
    }
}

/*
 * Function name: 
 *   - appendVectorsToCSVFile
 * Inputs: 
 *   - const std::string& filename: the name of the CSV file to append the vectors
 *   - const QVector<double>& vector1: reference to the vector containing the first column data
 *   - const QVector<double>& vector2: reference to the vector containing the second column data
 * Outputs: 
 *   - None
 * Description: 
 *   - Appends the contents of two vectors to the specified CSV file, creating new rows for each pair of elements.
 */
void appendVectorsToCSVFile(const std::string& filename, const QVector<double>& vector1, const QVector<double>& vector2) {
    std::ofstream file("WorkingDir/" + filename, std::ios::app); // Open file for appending

    if (file.is_open()) {
        qDebug() << "Found " << vector1.size() << " points";
        for (size_t i = 0; i < std::max(vector1.size(), vector2.size()); ++i) {
            if (i < vector1.size()) {
                file << vector1[i];
            }
            file << ","; // Add comma separator

            if (i < vector2.size()) {
                file << vector2[i];
            }
            file << "\n"; // Add newline character
        }

        file.close();
        qDebug() << "Vectors appended to CSV file '" << filename << "' successfully.";
    } else {
        qCritical() << "Error opening CSV file for appending.";
    }
}

/*
 * Function name: 
 *   - clearCSVFile
 * Inputs: 
 *   - const std::string& filename: the name of the CSV file to clear
 * Outputs: 
 *   - None
 * Description: 
 *   - Clears the contents of the specified CSV file, making it empty.
 */
void clearCSVFile(const std::string& filename) {
    std::ofstream file("WorkingDir/" + filename, std::ios::trunc); // Open file for truncation

    if (file.is_open()) {
        file.close();
        qDebug() << "CSV file '" << filename << "' cleared successfully.";
    } else {
        qCritical() << "Error clearing CSV file.";
    }
}

/*
 * Function name: 
 *   - combineBufferToMainMap
 * Inputs: 
 *   - None
 * Outputs: 
 *   - None
 * Description: 
 *   - Combines the contents of the 'buffer.csv' file into the 'mainmap.csv' file by appending buffer data to the main map.
 */
void combineBufferToMainMap() {
    // Import the buffer CSV
    QVector<double> xDataBuffer, yDataBuffer;
    readCSVFile("buffer.csv", xDataBuffer, yDataBuffer);

    // Clear the contents of the buffer
    clearCSVFile("buffer.csv");

    // Append the buffered data to the main map CSV file
    appendVectorsToCSVFile("mainmap.csv", xDataBuffer, yDataBuffer);
}
#endif // MAPPROCESSING_H
