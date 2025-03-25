// graphwidget.cpp
#include "graphwidget.h"

// This doesn't really do much. Ignore this initializer, and use the one in "GUISupport.h"
GraphWidget::GraphWidget(QWidget *parent) : QWidget(parent)
{
    customPlot = new QCustomPlot(this);
    // Set up the custom plot
    customPlot->addGraph();
    customPlot->xAxis->setLabel("X Axis Label");
    customPlot->yAxis->setLabel("Y Axis Label");
    customPlot->legend->setVisible(true);
    customPlot->legend->setFont(QFont("Helvetica", 9));
    customPlot->legend->setBrush(QColor(255, 255, 255, 150));

    // Customize the plot appearance
    customPlot->setBackground(QBrush(QColor(240, 240, 240)));
    customPlot->xAxis->setBasePen(QPen(Qt::black, 1));
    customPlot->yAxis->setBasePen(QPen(Qt::black, 1));
    customPlot->xAxis->setTickPen(QPen(Qt::black, 1));
    customPlot->yAxis->setTickPen(QPen(Qt::black, 1));

    // Set up interactions
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    // Add data if needed
    QVector<double> xData, yData;
    // Populate xData and yData with your data points
    xData << 1 << 5 << 10 << 6 << 7 << 8 << 12 << 13 << 17 << 18 << 2 << 4 << 9 << 11 << 14 << 15 << 16 << 19 << 20;
    yData << 1 << 9 << 19 << 11 << 13 << 15 << 23 << 25 << 33 << 35 << 3 << 7 << 17 << 21 << 27 << 29 << 31 << 37 << 39;

    customPlot->graph(0)->setData(xData, yData);

    // Set the size of the widget
    //customPlot->resize(800, 400);

    // Rescale and replot the graph
    customPlot->rescaleAxes();
    customPlot->replot();}
