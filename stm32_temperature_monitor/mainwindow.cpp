#include "mainwindow.h"

#include <QSerialPortInfo>
#include <QComboBox>
#include <QPushButton>
#include <QLabel>
#include <QTextEdit>
#include <QTableWidget>
#include <QHeaderView>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDateTime>
#include <QMessageBox>
#include <QMap>
#include <QBrush>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    createUi();
    refreshPorts();
    setConnected(false);

    connect(&serial, &QSerialPort::readyRead, this, &MainWindow::readPort);
    connect(&serial, &QSerialPort::errorOccurred, this, &MainWindow::serialError);
}

MainWindow::~MainWindow()
{
    if (serial.isOpen()) {
        serial.close();
    }
}

void MainWindow::createUi()
{
    setWindowTitle("STM32 Temperature Monitor");
    resize(850, 520);

    QWidget *central = new QWidget(this);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    QLabel *title = new QLabel("Мониторинг температуры STM32", this);
    title->setObjectName("title");

    QLabel *subtitle = new QLabel("LM75A / LM75B / DS18B20 x2 / UART", this);
    subtitle->setObjectName("subtitle");

    statusLabel = new QLabel("Отключено", this);
    statusLabel->setObjectName("status");

    QHBoxLayout *header = new QHBoxLayout();
    QVBoxLayout *titleBlock = new QVBoxLayout();

    titleBlock->addWidget(title);
    titleBlock->addWidget(subtitle);

    header->addLayout(titleBlock);
    header->addStretch();
    header->addWidget(statusLabel);

    mainLayout->addLayout(header);

    QHBoxLayout *connection = new QHBoxLayout();

    portBox = new QComboBox(this);
    refreshButton = new QPushButton("Обновить", this);
    connectButton = new QPushButton("Подключить", this);
    disconnectButton = new QPushButton("Отключить", this);

    connection->addWidget(new QLabel("COM-порт:", this));
    connection->addWidget(portBox);
    connection->addWidget(refreshButton);
    connection->addWidget(connectButton);
    connection->addWidget(disconnectButton);
    connection->addStretch();

    mainLayout->addLayout(connection);

    table = new QTableWidget(4, 4, this);
    table->setHorizontalHeaderLabels({"Датчик", "Интерфейс", "Температура", "Состояние"});
    table->verticalHeader()->hide();
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionMode(QAbstractItemView::NoSelection);

    const QString rows[4][2] = {
        {"LM75A", "I2C"},
        {"LM75B", "I2C"},
        {"DS18B20 №1", "1-Wire"},
        {"DS18B20 №2", "1-Wire"}
    };

    for (int r = 0; r < 4; r++) {
        table->setItem(r, 0, new QTableWidgetItem(rows[r][0]));
        table->setItem(r, 1, new QTableWidgetItem(rows[r][1]));
        table->setItem(r, 2, new QTableWidgetItem("--.-- °C"));
        table->setItem(r, 3, new QTableWidgetItem("Нет данных"));
    }

    mainLayout->addWidget(table);

    alarmLabel = new QLabel("Аварийный сигнал OS: нет данных", this);
    alarmLabel->setAlignment(Qt::AlignCenter);
    alarmLabel->setObjectName("alarm");

    mainLayout->addWidget(alarmLabel);

    logEdit = new QTextEdit(this);
    logEdit->setReadOnly(true);
    logEdit->setPlaceholderText("Журнал событий...");

    mainLayout->addWidget(logEdit);

    setCentralWidget(central);

    connect(refreshButton, &QPushButton::clicked, this, &MainWindow::refreshPorts);
    connect(connectButton, &QPushButton::clicked, this, &MainWindow::connectPort);
    connect(disconnectButton, &QPushButton::clicked, this, &MainWindow::disconnectPort);

    setStyleSheet(R"(
        QWidget {
            background: #eef4fb;
            color: #102b45;
            font-family: Segoe UI;
            font-size: 13px;
        }

        #title {
            font-size: 22px;
            font-weight: bold;
            color: #0f2e4f;
        }

        #subtitle {
            color: #6b7d90;
        }

        #status, #alarm {
            background: white;
            border: 1px solid #cbdced;
            border-radius: 8px;
            padding: 8px 12px;
            font-weight: bold;
        }

        QPushButton {
            background: #e8f1ff;
            border: 1px solid #bdd4ee;
            border-radius: 6px;
            padding: 7px 12px;
            font-weight: bold;
        }

        QPushButton:hover {
            background: #d8eaff;
        }

        QPushButton:disabled {
            background: #d4dbe3;
            color: #7c8894;
        }

        QComboBox, QTextEdit, QTableWidget {
            background: white;
            border: 1px solid #cbdced;
            border-radius: 8px;
        }

        QHeaderView::section {
            background: #dceaf7;
            padding: 6px;
            border: none;
            font-weight: bold;
        }
    )");
}

void MainWindow::refreshPorts()
{
    portBox->clear();

    const auto ports = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &port : ports) {
        portBox->addItem(port.portName());
    }

    appendLog("Список COM-портов обновлён");
}

void MainWindow::connectPort()
{
    if (portBox->currentText().isEmpty()) {
        QMessageBox::warning(this, "Ошибка", "COM-порт не выбран.");
        return;
    }

    serial.setPortName(portBox->currentText());
    serial.setBaudRate(QSerialPort::Baud115200);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "Ошибка COM-порта", serial.errorString());
        appendLog("Ошибка подключения: " + serial.errorString());
        return;
    }

    buffer.clear();
    setConnected(true);
    appendLog("Подключено к " + serial.portName());
}

void MainWindow::disconnectPort()
{
    if (serial.isOpen()) {
        serial.close();
    }

    setConnected(false);
    appendLog("Отключено");
}

void MainWindow::readPort()
{
    buffer += serial.readAll();

    while (buffer.contains('\n')) {
        int pos = buffer.indexOf('\n');
        QString line = QString::fromUtf8(buffer.left(pos)).trimmed();

        buffer.remove(0, pos + 1);

        if (!line.isEmpty()) {
            parseLine(line);
        }
    }
}

void MainWindow::parseLine(const QString &line)
{
    appendLog("RX: " + line);

    QMap<QString, QString> data;

    for (const QString &part : line.split(';', Qt::SkipEmptyParts)) {
        int p = part.indexOf('=');

        if (p < 0) {
            p = part.indexOf(':');
        }

        if (p > 0) {
            QString key = part.left(p).trimmed().toUpper();
            QString value = part.mid(p + 1).trimmed();
            data[key] = value;
        }
    }

    auto readTemp = [&](QStringList keys, int row) {
        for (const QString &key : keys) {
            QString k = key.toUpper();

            if (data.contains(k)) {
                bool ok = false;
                double temp = data[k].toDouble(&ok);
                setSensor(row, temp, ok);
                return;
            }
        }
    };

    readTemp({"LM75A", "LM75_1"}, 0);
    readTemp({"LM75B", "LM75_2"}, 1);
    readTemp({"DS1", "DS18B20_1"}, 2);
    readTemp({"DS2", "DS18B20_2"}, 3);

    QString os = data.value("OS", "0");
    QString alarm = data.value("ALARM", "0");

    if (alarm == "1" || os != "0") {
        setAlarm("Аварийный сигнал OS: " + os, true);
    } else {
        setAlarm("Аварийный сигнал OS: норма", false);
    }

    statusLabel->setText("Данные получены");
}

void MainWindow::setSensor(int row, double temp, bool ok)
{
    QTableWidgetItem *tempItem = table->item(row, 2);
    QTableWidgetItem *stateItem = table->item(row, 3);

    if (!ok) {
        tempItem->setText("--.-- °C");
        stateItem->setText("Ошибка данных");
        tempItem->setForeground(QBrush(Qt::darkRed));
        stateItem->setForeground(QBrush(Qt::darkRed));
        return;
    }

    bool warning = temp > 29.0;

    tempItem->setText(QString("%1 °C").arg(temp, 0, 'f', 3));
    stateItem->setText(warning ? "Тревога" : "Норма");

    tempItem->setForeground(QBrush(warning ? QColor("#d87900") : QColor("#0aa64f")));
    stateItem->setForeground(QBrush(warning ? QColor("#d87900") : QColor("#0aa64f")));
}

void MainWindow::setAlarm(const QString &text, bool active)
{
    alarmLabel->setText(text);

    if (active) {
        alarmLabel->setStyleSheet(
            "background: #ffe2e2;"
            "border: 1px solid #ff7777;"
            "border-radius: 8px;"
            "padding: 8px 12px;"
            "font-weight: bold;"
            "color: #c92828;"
        );
    } else {
        alarmLabel->setStyleSheet(
            "background: white;"
            "border: 1px solid #cbdced;"
            "border-radius: 8px;"
            "padding: 8px 12px;"
            "font-weight: bold;"
            "color: #0aa64f;"
        );
    }
}

void MainWindow::setConnected(bool connected)
{
    connectButton->setEnabled(!connected);
    disconnectButton->setEnabled(connected);
    refreshButton->setEnabled(!connected);
    portBox->setEnabled(!connected);

    statusLabel->setText(connected ? "Подключено" : "Отключено");
}

void MainWindow::appendLog(const QString &text)
{
    QString time = QDateTime::currentDateTime().toString("HH:mm:ss");
    logEdit->append(QString("[%1] %2").arg(time, text));
}

void MainWindow::serialError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::NoError) {
        return;
    }

    appendLog("Ошибка COM-порта: " + serial.errorString());

    if (serial.isOpen()) {
        serial.close();
        setConnected(false);
    }
}