#ifndef CONSOLELOGGER_H
#define CONSOLELOGGER_H

#include <QMainWindow>
#include <QApplication>


class ConsoleLogger : public QObject {
public:
    void logNormal(QString data) {
        consoleData += data + "\n";
    }

public slots:
    void setData(QString data) {
        //emit(MainWindow::logDataUpdated(data));
    }

private:
    QString consoleData = "";
};

#endif // CONSOLELOGGER_H
