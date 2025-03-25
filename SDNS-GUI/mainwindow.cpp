#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include "GUISupport.h"
#include "MapProcessing.h"
#include "TCPSocket.h"
#include <QApplication>
#include <QMainWindow>
#include <QObject>

bool showDebugMenu = false;

// Creates the main window with all of it's widgets. For future teams, try and use QObject::connect for linking
// the console logger class to this class (or perhaps GUISupport) for printing logs to console window
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    Ui::MainWindow *ui_test = ui;

    // Graph Widget Creation
    createCustomPlot(ui->widgetGraph);
    ui->widgetGraph->show();

    // Initialize CSV files
    initializeFileStructure();

    // Link the signals and slots for MainWindow and ConsoleLogger
    //QObject::connect(ui, &MainWindow::logDataUpdated, loggers, &ConsoleLogger::setData);
}


// Custom signals for ConsoleLogger
void MainWindow::logDataUpdated(QString data) {
    ui->textEditConsoleLogs->setText(data);
}

Ui::MainWindow *MainWindow::getUi() {
    return ui;
}

QTextEdit* MainWindow::getTextBox() {
    return ui->textEditConsoleLogs;
}

// Initialize Socket - For future teams, a second robot can be added in via a second declaration of
// Local server:
//TCPSocket server(5555, "127.0.0.1");
// IP address of the server (not the robot)
TCPSocket server(5555, "192.168.68.101");

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButtonGeneratePoints_clicked()
{
    qDebug() << "Generate points button clicked";
    generateTestData();
    ui->textEditConsoleLogs->setText("Hello");
}


void MainWindow::on_pushButtonGenRandomCSV_clicked()
{
    generateRandomVectorsAndSave("buffer.csv", 10);
}


void MainWindow::on_pushButtonImportBufferToMap_clicked()
{
    combineBufferToMainMap();
}


void MainWindow::on_pushButtonUpdateGraph_clicked()
{
    MapStruct *map = new MapStruct();
    readCSVFile("mainmap.csv", map->xData, map->yData);
    plotPoints(map->xData, map->yData);
    refreshGraph();
}


void MainWindow::on_pushButtonClearMap_clicked()
{
    clearCSVFile("mainmap.csv");
    clearCSVFile("buffer.csv");
}


void MainWindow::on_pushButtonOpenServerSocket_clicked()
{
    // Create a TCPSocket instance for the server
    server.startServer();

}


void MainWindow::on_pushButtonCloseServerSocket_clicked()
{
    // Close the server
}


void MainWindow::on_pushButtonSendMessageToClient_clicked()
{

}


void MainWindow::on_pushButtonLoadBufferToMap_clicked()
{
    loadBufferToMap();
}


void MainWindow::on_pushButtonStartWatchdog_clicked()
{
    // Start the watchdog for new plots
    watchdog.startWatchdog();
}


void MainWindow::on_pushButtonTestConsoleLog_clicked()
{
    //loggers.logNormal("Test");
}

