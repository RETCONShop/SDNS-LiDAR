/////////////////////////////////////////////////////////////////////////////////////////////////////
// Name:        GUISupport.h
// Purpose:     A simple library for creating and managing the GUI
//              for SNDS-SNL Capstone
// Author:      Andrew Gallagher
// Created:     4/5/2024
// Revision History:
//   - 4.24.24 4:32 PM MT: Added Comments
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
#ifndef GUISUPPORT_H
#define GUISUPPORT_H

// Required includes
#include <QApplication>		// Qt library for managing application control flow and events
#include <QWidget> 			// Qt library for creating and managing user interface components
#include <QVBoxLayout>		// Qt library for arranging widgets vertically in a layout
#include <random>			// C++ library for generating random numbers
#include <string>			// C++ library for working with strings
#include "qcustomplot.h" 	// Custom plotting library for creating interactive graphs
#include "LightSerial.h"	// Custom library for converting structs into strings
#include "MapDataConversion.h" // Custom library for converting LiDAR data to XY map points
#include "WatchdogService.h" // Custom library for managing buffer files and data storage


//#include "MapProcessing.h"

// Function inits
void initCustomPlot(QWidget *parent);
void addNewSubgraph(int errorSize);
void createCustomPlot(QWidget *parent);
std::pair<double, double> findMinMax(const QVector<double>& values);
void updatePlotSize(QVector<double> xData, QVector<double> yData);
void plotPoints(QVector<double> xData, QVector<double> yData);
void refreshGraph();
void generateTestData();
void saveMessage(std::string msg);
MapData pruneMapData(MapData data);
void loadBufferToMap();


QCustomPlot *customPlot;

LightSerial lsr;
std::string bufferName = "recvBuffer";
std::string bufferDir = "Buffers/";

// Variables for subplots
int plotID = 0;
bool hasInitGraph = false;

MapDataConversion mdc;

// Initialize the watchdog service to manage buffer files
// Watchdog service is responsible for managing buffer files used for data storage and retrieval
// It keeps track of the current buffer index and provides functionality for saving and loading data from files
WatchdogService watchdog;

/*
 * Function name: initCustomPlot
 * Inputs: 
 *   - QWidget *parent: Pointer to the parent widget
 * Outputs: 
 *   - None
 * Description: 
 *   - Initializes a custom plot widget and adds it to the specified parent widget's layout.
 */
void initCustomPlot(QWidget *parent) {
    // Create a QVBoxLayout to manage the layout
    QVBoxLayout *layout;

    // Check if the parent widget already has a layout
    if (parent->layout()) {
        // If a layout already exists, use the existing layout
        layout = qobject_cast<QVBoxLayout*>(parent->layout());
    } else {
        // If no layout exists, create a new QVBoxLayout
        layout = new QVBoxLayout(parent);
        parent->setLayout(layout);
    }

    // Create a QCustomPlot widget
    customPlot = new QCustomPlot(parent);

    // Add the QCustomPlot widget to the layout
    layout->addWidget(customPlot);
}

/*
 * Function name: addNewSubgraph
 * Inputs: 
 *   - int errorSize: Size of the error in the subgraph
 * Outputs: 
 *   - None
 * Description: 
 *   - Adds a new subgraph to the custom plot with the specified error size.
 */
void addNewSubgraph(int errorSize) {
    customPlot->addGraph();

    // Set the data of the graph
    customPlot->graph(plotID)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, Qt::white, errorSize)); // Set scatter style
    customPlot->graph(plotID)->setLineStyle(QCPGraph::lsNone);

    // Formatting of the graph
    customPlot->graph(plotID)->selectionDecorator()->setPen(QPen(Qt::white));
    customPlot->xAxis->setTickLabelColor(Qt::white);
    customPlot->xAxis->setBasePen(QPen(Qt::white));
    customPlot->xAxis->setLabelColor(Qt::white);
    customPlot->yAxis->setTickLabelColor(Qt::white);
    customPlot->yAxis->setBasePen(QPen(Qt::white));
    customPlot->yAxis->setLabelColor(Qt::white);

    // Disable grids
    customPlot->xAxis->grid()->setVisible(false);
    customPlot->yAxis->grid()->setVisible(false);

    // Increment the plot ID
    plotID += 1;
}

/*
 * Function name: createCustomPlot
 * Inputs: 
 *   - QWidget *parent: Pointer to the parent widget
 * Outputs: 
 *   - None
 * Description: 
 *   - Creates a custom plot widget and configures it with default settings.
 */
// Generate the default graph. Most of this code is redundant, as the actual plotting takes place in addNewSubgraph.
// Leaving this code here specifically for learning purposes (future capstone teams), and in case I need to yoink
// some of the code that is confirmed 100% functional. -AG
void createCustomPlot(QWidget *parent)
{
    // Initialize the graph
    if (!hasInitGraph) {
        initCustomPlot(parent);
    }

    // Add a graph to the plot
    customPlot->addGraph();

    // Set the data of the graph
    customPlot->graph(plotID)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, Qt::white, 7)); // Set scatter style
    customPlot->graph(plotID)->setLineStyle(QCPGraph::lsNone);

    // Set labels for the axes
    customPlot->xAxis->setLabel("X Data");
    customPlot->yAxis->setLabel("Y Data");

    // Set colors
    // Linux color: 42, 46, 50, 255)
    customPlot->setBackground(QColor(25, 25, 25, 255));
    customPlot->graph(plotID)->selectionDecorator()->setPen(QPen(Qt::white));
    customPlot->xAxis->setTickLabelColor(Qt::white);
    customPlot->xAxis->setBasePen(QPen(Qt::white));
    customPlot->xAxis->setLabelColor(Qt::white);
    customPlot->yAxis->setTickLabelColor(Qt::white);

    // Update the min and max values for the graph
    //auto [yMin, yMax] = findMinMax(yData);

    // Update the bounds of the graph
    customPlot->xAxis->setRange(-10000, 10000);
    customPlot->yAxis->setRange(-10000, 10000);

    customPlot->yAxis->setBasePen(QPen(Qt::white));
    customPlot->yAxis->setLabelColor(Qt::white);

    // Disable grids
    customPlot->xAxis->grid()->setVisible(false);
    customPlot->yAxis->grid()->setVisible(false);

    customPlot->replot();

    //plotID += 1;
}

/*
 * Function name: findMinMax
 * Inputs: 
 *   - const QVector<double>& values: Vector of double values
 * Outputs: 
 *   - std::pair<double, double>: Pair containing the minimum and maximum values in the input vector
 * Description: 
 *   - Finds the minimum and maximum values in a QVector<double>.
 */
std::pair<double, double> findMinMax(const QVector<double>& values) {
    if (values.empty()) {
        return {0.0, 0.0}; // return 0.0 for both if the vector is empty
    }

    double minVal = values[0];
    double maxVal = values[0];

    for (size_t i = 1; i < values.size(); ++i) {
        if (values[i] < minVal) {
            minVal = values[i];
        }
        if (values[i] > maxVal) {
            maxVal = values[i];
        }
    }

    return {minVal, maxVal};
}

/*
 * Function name: updatePlotSize
 * Inputs: 
 *   - QVector<double> xData: Vector of X data points
 *   - QVector<double> yData: Vector of Y data points
 * Outputs: 
 *   - None
 * Description: 
 *   - Updates the plot size based on the range of X and Y data points.
 */
void updatePlotSize(QVector<double> xData, QVector<double> yData) {
    auto [xMin, xMax] = findMinMax(xData);
    auto [yMin, yMax] = findMinMax(yData);

    // Rigged version

    //int xMin = -10000;
    //int yMin = -10000;
    //int xMax = 10000;
    //int yMax = 10000;

    // Update the bounds of the graph
    customPlot->xAxis->setRange(xMin, xMax);
    customPlot->yAxis->setRange(yMin, yMax);
}

/*
 * Function name: plotPoints
 * Inputs: 
 *   - QVector<double> xData: Vector of X data points
 *   - QVector<double> yData: Vector of Y data points
 * Outputs: 
 *   - None
 * Description: 
 *   - Plots the specified data points on the custom plot.
 */
void plotPoints(QVector<double> xData, QVector<double> yData) {
    updatePlotSize(xData, yData);
    customPlot->graph(plotID)->setData(xData, yData);
}

/*
 * Function name: refreshGraph
 * Inputs: 
 *   - None
 * Outputs: 
 *   - None
 * Description: 
 *   - Refreshes the custom plot, updating it with any changes made.
 */
void refreshGraph() {
    qDebug() << "Refreshed graph";
    customPlot->replot();
}

/*
 * Function name: generateTestData
 * Inputs: 
 *   - None
 * Outputs: 
 *   - None
 * Description: 
 *   - Generates random test data and plots it on the custom plot.
 */
void generateTestData() {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(2, 8);
    QVector<double> xData, yData;

    for (int i = 0; i < 10; ++i) {
        xData.push_back(dis(gen));
        yData.push_back(dis(gen));
    }

    plotPoints(xData, yData);
    addNewSubgraph(dis(gen));

    refreshGraph();
}

/*
 * Function name: saveMessage
 * Inputs: 
 *   - std::string msg: Message to be saved
 * Outputs: 
 *   - None
 * Description: 
 *   - Saves the received message to a new file.
 */
void saveMessage(std::string msg) {
    std::string bufferGenName = bufferDir + bufferName + std::to_string(watchdog.bufferTally) + ".bin";
    lsr.writeStringToFile(bufferGenName.c_str(), msg);
    watchdog.bufferTally += 1;
}

/*
 * Function name: pruneMapData
 * Inputs: 
 *   - MapData data: MapData struct to be pruned
 * Outputs: 
 *   - MapData: Pruned MapData struct
 * Description: 
 *   - Removes error values from a MapData struct.
 */
MapData pruneMapData(MapData data) {
    MapData tempData = data;
    int numberOfBadPoints = 0;
    // Loop through the arrays, removing 0 values
    int numberOfEntries = tempData.r.size();
    for (int i = 0; i < numberOfEntries; ++i) {
        if (tempData.r[i] == 0.0) {
            tempData.r.erase(tempData.r.begin() + i);
            tempData.theta.erase(tempData.theta.begin() + i);
            i -= 1;
            numberOfBadPoints += 1;
        }

    }
    std::cout << "Found " << std::to_string(numberOfBadPoints) << " bad points." << std::endl;

    return tempData;
}

/*
 * Function name: loadBufferToMap
 * Inputs: 
 *   - None
 * Outputs: 
 *   - None
 * Description: 
 *   - Loads buffer data to the map, converting it to XY coordinates and plotting it on the custom plot.
 */
void loadBufferToMap() {
    // Declare the map conversion class
    MapDataConversion mdc;

    // Create an error struct for each map scan
    ErrorSize err;

    // Read the buffer from file into a string
    std::string bufferToRead = bufferDir + bufferName + std::to_string(watchdog.currentBuffer) + ".bin";
    std::cout << "Trying to open: " << bufferToRead << std::endl;

    // Convert the string into a MapData struct
    MapData data = lsr.readMapDataFromFile(bufferToRead.c_str());
    watchdog.currentBuffer += 1;

    // Prune bad points (if necessary)
    // PruneMapData function removes erroneous points (e.g., zero values) from the map data to ensure data integrity.
    // It loops through the map data and removes points with zero range values, updating the data structure accordingly.
    // The function returns a modified version of the input data with bad points removed.
    // data = pruneMapData(data);

    // Convert to X,Y coordinates
    // ConvertToXYMapNew function converts polar coordinates to Cartesian coordinates for plotting on the graph.
    // It loops through the range and angle data in the MapData struct, applies coordinate transformations,
    // and stores the resulting X and Y coordinates in a MapPointsQVector data structure.
    MapPointsQVector points = mdc.convertToXYMapNew(data);

    // Calculate the a, b error values for error representation
    // Error coefficients 'a' and 'b' are calculated to represent errors in the graph based on data variance.
    // 'a' represents error along the x-axis, 'b' represents error along the y-axis.
    // The values are derived from the variance of the angle data and a constant representing the change in angle.
    // These coefficients are used to adjust the appearance of the plotted data to reflect the uncertainty in the measurements.
    // Error representation is crucial for accurately interpreting the data and assessing its reliability.
    double dTheta = M_PI / 180;
    err.a = 1;
    err.b = tan(sqrt(pow(data.var_phi, 2) + pow(dTheta, 2)));

    std::cout << "X: " << data.x << "\nY: " << data.y << std::endl;

    // Add the points to the graph
    plotPoints(points.xData, points.yData);

    // Create a new graph for error representation
    // AddNewSubgraph function adds a new subplot to the graph for representing errors.
    // The error size is determined by the error coefficient 'b' multiplied by a scaling factor.
    std::cout << err.b << std::endl;
    addNewSubgraph(err.b * 200);

    // Append the points to the CSV Map File (temporary)
    //appendVectorsToCSVFile("buffer.csv", xData, yData);
    //combineBufferToMainMap();

    // Refresh the graph
    refreshGraph();
}


#endif // GUISUPPORT_H
