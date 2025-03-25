#include "mainwindow.h"
#include <QApplication>
#include <QWidget>
#include <QVBoxLayout>

QMainWindow *sdnsMainWindow;

// Main function
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    sdnsMainWindow = new MainWindow();
    //QObject::connect(ui, &MainWindow::logDataUpdated, loggers, &ConsoleLogger::setData);

    //loggers.initConsoleLogger(sdnsMainWindow);

    sdnsMainWindow->show();
    return a.exec();
}


