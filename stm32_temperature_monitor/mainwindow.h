#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSerialPort>
#include <QByteArray>

class QComboBox;
class QPushButton;
class QLabel;
class QTextEdit;
class QTableWidget;
class QDoubleSpinBox;
class QSpinBox;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void refreshPorts();
    void connectPort();
    void disconnectPort();
    void readPort();
    void serialError(QSerialPort::SerialPortError error);
    void applyLm75Settings();
    void applyDs18b20Settings();

private:
    void createUi();
    void setConnected(bool connected);
    void appendLog(const QString &text);
    void parseLine(const QString &line);
    void setSensor(int row, double temp, bool ok, double warningLimit);
    void setAlarm(const QString &text, bool active);
    void sendCommand(const QString &command);

private:
    QSerialPort serial;
    QByteArray buffer;

    QComboBox *portBox = nullptr;
    QPushButton *refreshButton = nullptr;
    QPushButton *connectButton = nullptr;
    QPushButton *disconnectButton = nullptr;

    QLabel *statusLabel = nullptr;
    QLabel *alarmLabel = nullptr;
    QTableWidget *table = nullptr;
    QTextEdit *logEdit = nullptr;

    QDoubleSpinBox *lm75OnSpin = nullptr;
    QDoubleSpinBox *lm75OffSpin = nullptr;
    QPushButton *applyLm75Button = nullptr;

    QComboBox *dsTargetBox = nullptr;
    QComboBox *dsResolutionBox = nullptr;
    QSpinBox *dsTlSpin = nullptr;
    QSpinBox *dsThSpin = nullptr;
    QPushButton *applyDsButton = nullptr;
};

#endif