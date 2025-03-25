#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "graphwidget.h"
#include "ConsoleLogger.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    Ui::MainWindow *ui;
    GraphWidget *graphWidget;

    // Public get method for the UI
    QTextEdit* getTextBox();
    Ui::MainWindow* getUi();

    // Console logger
    ConsoleLogger loggers;
public slots:
    void logDataUpdated(QString data);

private slots:
    void on_pushButtonGeneratePoints_clicked();

    void on_pushButtonGenRandomCSV_clicked();

    void on_pushButtonImportBufferToMap_clicked();

    void on_pushButtonUpdateGraph_clicked();

    void on_pushButtonClearMap_clicked();

    void on_pushButtonOpenServerSocket_clicked();

    void on_pushButtonCloseServerSocket_clicked();

    void on_pushButtonSendMessageToClient_clicked();

    void on_pushButtonLoadBufferToMap_clicked();

    void on_pushButtonStartWatchdog_clicked();

    void on_pushButtonTestConsoleLog_clicked();

private:
};
#endif // MAINWINDOW_H


