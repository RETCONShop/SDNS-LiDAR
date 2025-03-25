// graphwidget.h
#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QWidget>
#include "qcustomplot.h"

class GraphWidget : public QWidget
{
    Q_OBJECT

public:
    explicit GraphWidget(QWidget *parent = nullptr);
    QCustomPlot *customPlot;
private:

};

#endif // GRAPHWIDGET_H
