/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 6.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QPushButton *pushButtonGeneratePoints;
    QPushButton *pushButtonUpdateGraph;
    QPushButton *pushButtonGenRandomCSV;
    QWidget *widgetGraph;
    QPushButton *pushButtonImportBufferToMap;
    QPushButton *pushButtonClearMap;
    QPushButton *pushButtonOpenServerSocket;
    QPushButton *pushButtonCloseServerSocket;
    QPushButton *pushButtonSendMessageToClient;
    QPushButton *pushButtonLoadBufferToMap;
    QPushButton *pushButtonStartWatchdog;
    QLabel *label;
    QLabel *label_2;
    QTextEdit *textEditConsoleLogs;
    QPushButton *pushButtonMainOpenServer;
    QPushButton *pushButtonMainCloseServer;
    QPushButton *pushButtonMainClearBuffers;
    QPushButton *pushButtonMainResetMapData;
    QPushButton *pushButtonMainUpdateGraph;
    QPushButton *pushButtonTestConsoleLog;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName("MainWindow");
        MainWindow->resize(1200, 950);
        MainWindow->setMinimumSize(QSize(1200, 950));
        MainWindow->setMaximumSize(QSize(1200, 950));
        QIcon icon(QIcon::fromTheme(QString::fromUtf8("applications-engineering")));
        MainWindow->setWindowIcon(icon);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName("centralwidget");
        pushButtonGeneratePoints = new QPushButton(centralwidget);
        pushButtonGeneratePoints->setObjectName("pushButtonGeneratePoints");
        pushButtonGeneratePoints->setGeometry(QRect(20, 770, 111, 71));
        pushButtonUpdateGraph = new QPushButton(centralwidget);
        pushButtonUpdateGraph->setObjectName("pushButtonUpdateGraph");
        pushButtonUpdateGraph->setGeometry(QRect(130, 770, 111, 71));
        pushButtonGenRandomCSV = new QPushButton(centralwidget);
        pushButtonGenRandomCSV->setObjectName("pushButtonGenRandomCSV");
        pushButtonGenRandomCSV->setGeometry(QRect(240, 770, 111, 71));
        widgetGraph = new QWidget(centralwidget);
        widgetGraph->setObjectName("widgetGraph");
        widgetGraph->setGeometry(QRect(20, 80, 891, 641));
        widgetGraph->setAutoFillBackground(true);
        pushButtonImportBufferToMap = new QPushButton(centralwidget);
        pushButtonImportBufferToMap->setObjectName("pushButtonImportBufferToMap");
        pushButtonImportBufferToMap->setEnabled(true);
        pushButtonImportBufferToMap->setGeometry(QRect(350, 770, 111, 71));
        pushButtonClearMap = new QPushButton(centralwidget);
        pushButtonClearMap->setObjectName("pushButtonClearMap");
        pushButtonClearMap->setEnabled(true);
        pushButtonClearMap->setGeometry(QRect(460, 770, 111, 71));
        pushButtonOpenServerSocket = new QPushButton(centralwidget);
        pushButtonOpenServerSocket->setObjectName("pushButtonOpenServerSocket");
        pushButtonOpenServerSocket->setEnabled(true);
        pushButtonOpenServerSocket->setGeometry(QRect(20, 840, 111, 71));
        pushButtonCloseServerSocket = new QPushButton(centralwidget);
        pushButtonCloseServerSocket->setObjectName("pushButtonCloseServerSocket");
        pushButtonCloseServerSocket->setEnabled(true);
        pushButtonCloseServerSocket->setGeometry(QRect(130, 840, 111, 71));
        pushButtonSendMessageToClient = new QPushButton(centralwidget);
        pushButtonSendMessageToClient->setObjectName("pushButtonSendMessageToClient");
        pushButtonSendMessageToClient->setEnabled(true);
        pushButtonSendMessageToClient->setGeometry(QRect(240, 840, 111, 71));
        pushButtonLoadBufferToMap = new QPushButton(centralwidget);
        pushButtonLoadBufferToMap->setObjectName("pushButtonLoadBufferToMap");
        pushButtonLoadBufferToMap->setEnabled(true);
        pushButtonLoadBufferToMap->setGeometry(QRect(350, 840, 111, 71));
        pushButtonStartWatchdog = new QPushButton(centralwidget);
        pushButtonStartWatchdog->setObjectName("pushButtonStartWatchdog");
        pushButtonStartWatchdog->setEnabled(true);
        pushButtonStartWatchdog->setGeometry(QRect(460, 840, 111, 71));
        label = new QLabel(centralwidget);
        label->setObjectName("label");
        label->setGeometry(QRect(20, 730, 191, 41));
        QFont font;
        font.setFamilies({QString::fromUtf8("Source Code Pro")});
        font.setPointSize(16);
        label->setFont(font);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName("label_2");
        label_2->setGeometry(QRect(20, 10, 481, 51));
        QFont font1;
        font1.setFamilies({QString::fromUtf8("Source Code Pro")});
        font1.setPointSize(24);
        label_2->setFont(font1);
        textEditConsoleLogs = new QTextEdit(centralwidget);
        textEditConsoleLogs->setObjectName("textEditConsoleLogs");
        textEditConsoleLogs->setGeometry(QRect(920, 90, 261, 441));
        textEditConsoleLogs->setReadOnly(true);
        pushButtonMainOpenServer = new QPushButton(centralwidget);
        pushButtonMainOpenServer->setObjectName("pushButtonMainOpenServer");
        pushButtonMainOpenServer->setGeometry(QRect(920, 540, 81, 81));
        pushButtonMainCloseServer = new QPushButton(centralwidget);
        pushButtonMainCloseServer->setObjectName("pushButtonMainCloseServer");
        pushButtonMainCloseServer->setGeometry(QRect(1010, 540, 81, 81));
        pushButtonMainClearBuffers = new QPushButton(centralwidget);
        pushButtonMainClearBuffers->setObjectName("pushButtonMainClearBuffers");
        pushButtonMainClearBuffers->setGeometry(QRect(1100, 540, 81, 81));
        pushButtonMainResetMapData = new QPushButton(centralwidget);
        pushButtonMainResetMapData->setObjectName("pushButtonMainResetMapData");
        pushButtonMainResetMapData->setGeometry(QRect(1050, 630, 81, 81));
        pushButtonMainUpdateGraph = new QPushButton(centralwidget);
        pushButtonMainUpdateGraph->setObjectName("pushButtonMainUpdateGraph");
        pushButtonMainUpdateGraph->setGeometry(QRect(960, 630, 81, 81));
        pushButtonTestConsoleLog = new QPushButton(centralwidget);
        pushButtonTestConsoleLog->setObjectName("pushButtonTestConsoleLog");
        pushButtonTestConsoleLog->setEnabled(true);
        pushButtonTestConsoleLog->setGeometry(QRect(570, 770, 111, 71));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName("menubar");
        menubar->setGeometry(QRect(0, 0, 1200, 19));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName("statusbar");
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "SDNS Commander V1", nullptr));
        pushButtonGeneratePoints->setText(QCoreApplication::translate("MainWindow", "Generate\n"
"Random\n"
"Points", nullptr));
        pushButtonUpdateGraph->setText(QCoreApplication::translate("MainWindow", "Update\n"
"Graph", nullptr));
        pushButtonGenRandomCSV->setText(QCoreApplication::translate("MainWindow", "Generate\n"
"Random\n"
"CSV", nullptr));
        pushButtonImportBufferToMap->setText(QCoreApplication::translate("MainWindow", "Import\n"
"Buffer to\n"
"Map", nullptr));
        pushButtonClearMap->setText(QCoreApplication::translate("MainWindow", "Clear\n"
"Map\n"
"File", nullptr));
        pushButtonOpenServerSocket->setText(QCoreApplication::translate("MainWindow", "Open\n"
"Server\n"
"Socket", nullptr));
        pushButtonCloseServerSocket->setText(QCoreApplication::translate("MainWindow", "Close\n"
"Server\n"
"Socket", nullptr));
        pushButtonSendMessageToClient->setText(QCoreApplication::translate("MainWindow", "Send a\n"
"Message to\n"
"Client", nullptr));
        pushButtonLoadBufferToMap->setText(QCoreApplication::translate("MainWindow", "Load Buffer\n"
"To Map", nullptr));
        pushButtonStartWatchdog->setText(QCoreApplication::translate("MainWindow", "Start\n"
"Watchdog", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Debug Options", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "SDNS Commander", nullptr));
        pushButtonMainOpenServer->setText(QCoreApplication::translate("MainWindow", "Open\n"
"Server", nullptr));
        pushButtonMainCloseServer->setText(QCoreApplication::translate("MainWindow", "Close\n"
"Server", nullptr));
        pushButtonMainClearBuffers->setText(QCoreApplication::translate("MainWindow", "Clear\n"
"Buffers", nullptr));
        pushButtonMainResetMapData->setText(QCoreApplication::translate("MainWindow", "Reset\n"
"Map Data", nullptr));
        pushButtonMainUpdateGraph->setText(QCoreApplication::translate("MainWindow", "Update\n"
"Graph", nullptr));
        pushButtonTestConsoleLog->setText(QCoreApplication::translate("MainWindow", "Test Console\n"
"Logs", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
