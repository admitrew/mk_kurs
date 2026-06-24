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
#include <QRegularExpression>
#include <QGroupBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QSpinBox>

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

    QLabel *subtitle = new QLabel("LM75A / DS18B20 x2 / UART", this);
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

    table = new QTableWidget(3, 4, this);
    table->setHorizontalHeaderLabels({"Датчик", "Интерфейс", "Температура", "Состояние"});
    table->verticalHeader()->hide();
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table->setSelectionMode(QAbstractItemView::NoSelection);

    const QString rows[3][2] = {
    {"LM75A", "I2C"},
    {"DS18B20 №1", "1-Wire"},
    {"DS18B20 №2", "1-Wire"}
    };

    for (int r = 0; r < 3; r++) {
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
    
    QGroupBox *settingsGroup = new QGroupBox("Настройки датчиков", this);
    QHBoxLayout *settingsLayout = new QHBoxLayout(settingsGroup);
    QGroupBox *lm75Group = new QGroupBox("LM75A", settingsGroup);
    QFormLayout *lm75Layout = new QFormLayout(lm75Group);

    lm75OnSpin = new QDoubleSpinBox(lm75Group);
    lm75OnSpin->setRange(-55.0, 125.0);
    lm75OnSpin->setDecimals(1);
    lm75OnSpin->setSingleStep(0.5);
    lm75OnSpin->setValue(29.0);
    lm75OnSpin->setSuffix(" °C");

    lm75OffSpin = new QDoubleSpinBox(lm75Group);
    lm75OffSpin->setRange(-55.0, 125.0);
    lm75OffSpin->setDecimals(1);
    lm75OffSpin->setSingleStep(0.5);
    lm75OffSpin->setValue(28.0);
    lm75OffSpin->setSuffix(" °C");

    applyLm75Button = new QPushButton("Применить LM75A", lm75Group);

    lm75Layout->addRow("Включить тревогу:", lm75OnSpin);
    lm75Layout->addRow("Выключить тревогу:", lm75OffSpin);
    lm75Layout->addRow(applyLm75Button);

    QGroupBox *dsGroup = new QGroupBox("DS18B20", settingsGroup);
    QFormLayout *dsLayout = new QFormLayout(dsGroup);

    dsTargetBox = new QComboBox(dsGroup);
    dsTargetBox->addItem("Все датчики", 0);
    dsTargetBox->addItem("DS18B20 №1", 1);
    dsTargetBox->addItem("DS18B20 №2", 2);

    dsResolutionBox = new QComboBox(dsGroup);
    dsResolutionBox->addItem("9", 9);
    dsResolutionBox->addItem("10", 10);
    dsResolutionBox->addItem("11", 11);
    dsResolutionBox->addItem("12", 12);
    dsResolutionBox->setCurrentText("12");

    dsTlSpin = new QSpinBox(dsGroup);
    dsTlSpin->setRange(-55, 125);
    dsTlSpin->setValue(0);
    dsTlSpin->setSuffix(" °C");

    dsThSpin = new QSpinBox(dsGroup);
    dsThSpin->setRange(-55, 125);
    dsThSpin->setValue(40);
    dsThSpin->setSuffix(" °C");

    applyDsButton = new QPushButton("Применить DS18B20", dsGroup);

    dsLayout->addRow("Датчик:", dsTargetBox);
    dsLayout->addRow("Разрядность:", dsResolutionBox);
    dsLayout->addRow("TL:", dsTlSpin);
    dsLayout->addRow("TH:", dsThSpin);
    dsLayout->addRow(applyDsButton);

    settingsLayout->addWidget(lm75Group);
    settingsLayout->addWidget(dsGroup);

    mainLayout->addWidget(settingsGroup);

    connect(applyLm75Button, &QPushButton::clicked,
            this, &MainWindow::applyLm75Settings);

    connect(applyDsButton, &QPushButton::clicked,
        this, &MainWindow::applyDs18b20Settings);

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
    serial.setBaudRate(QSerialPort::Baud9600);
    serial.setDataBits(QSerialPort::Data8);
    serial.setParity(QSerialPort::NoParity);
    serial.setStopBits(QSerialPort::OneStop);
    serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!serial.open(QIODevice::ReadWrite)) {
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
    if (line.startsWith("ACK;")) {
        appendLog("Ответ STM32: " + line);
        return;
    }

    if (line.startsWith("ERR;")) {
        appendLog("Ошибка STM32: " + line);
        return;
    }
    
    if (!line.startsWith("DATA;")) {
        appendLog("RX: " + line);
        return;
    }

    appendLog("RX: " + line);

    QString dataLine = line.mid(QString("DATA;").length());

    QMap<QString, QString> data;

    for (const QString &part : dataLine.split(';', Qt::SkipEmptyParts)) {
        int p = part.indexOf('=');

        if (p > 0) {
            QString key = part.left(p).trimmed().toUpper();
            QString value = part.mid(p + 1).trimmed();
            data[key] = value;
        }
    }

    auto parseNumber = [](QString text, double &value) -> bool {
        QRegularExpression re(R"((-?\d+(?:[.,]\d+)?))");
        QRegularExpressionMatch match = re.match(text);

        if (!match.hasMatch()) {
            return false;
        }

        QString number = match.captured(1);
        number.replace(',', '.');

        bool ok = false;
        value = number.toDouble(&ok);
        return ok;
    };

    auto readTemp = [&](QString key, int row, double warningLimit) {
        key = key.toUpper();

        if (data.contains(key)) {
            double temp = 0.0;
            bool ok = parseNumber(data[key], temp);
            setSensor(row, temp, ok, warningLimit);
        }
    };

    table->setUpdatesEnabled(false);

    readTemp("LM75A", 0, lm75OnSpin->value());
    readTemp("DS1", 1, dsThSpin->value());
    readTemp("DS2", 2, dsThSpin->value());

    table->setUpdatesEnabled(true);

    QString os = data.value("OS", "0").trimmed().toUpper();
    QString alarm = data.value("ALARM", "0").trimmed().toUpper();

    bool osActive = (os != "0" && os != "NO" && os != "NONE" && os != "FALSE");
    bool alarmActive = (alarm == "1" || alarm == "TRUE");

    if (osActive) {
        setAlarm("Аварийный сигнал OS: " + os, true);
    } else if (alarmActive) {
        setAlarm("Температурная тревога: один или несколько датчиков выше порога", true);
    } else {
        setAlarm("Аварийный сигнал OS: норма", false);
    }

    statusLabel->setText("Данные получены");
}

void MainWindow::setSensor(int row, double temp, bool ok, double warningLimit)
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

    bool warning = temp >= warningLimit;

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

void MainWindow::sendCommand(const QString &command)
{
    if (!serial.isOpen()) {
        appendLog("Команда не отправлена: COM-порт не подключен");
        return;
    }

    QByteArray bytes = command.toLatin1();
    bytes.append("\r\n");

    serial.write(bytes);

    appendLog("TX: " + command);
}

void MainWindow::applyLm75Settings()
{
    double onValue = lm75OnSpin->value();
    double offValue = lm75OffSpin->value();

    if (onValue <= offValue) {
        appendLog("Ошибка настройки LM75A: порог включения должен быть выше порога выключения");
        return;
    }

    QString command = QString("@L;%1;%2")
            .arg(onValue, 0, 'f', 1)
            .arg(offValue, 0, 'f', 1);

    sendCommand(command);
}

void MainWindow::applyDs18b20Settings()
{
    int slot = dsTargetBox->currentData().toInt();
    int resolution = dsResolutionBox->currentData().toInt();
    int tl = dsTlSpin->value();
    int th = dsThSpin->value();

    if (tl >= th) {
        appendLog("Ошибка настройки DS18B20: TL должен быть меньше TH");
        return;
    }

    QString command = QString("@D;%1;%2;%3;%4")
            .arg(slot)
            .arg(resolution)
            .arg(tl)
            .arg(th);

    sendCommand(command);
}