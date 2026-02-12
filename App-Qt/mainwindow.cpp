#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFile>
#include <QMessageBox>
#include <QNetworkInterface>
#include <QtNetwork/QUdpSocket>
#include <QColor>
#include <QPainterPath>
#include "qpaintbox.h"
#include <math.h>

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>

velocista::velocista(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::velocista)
{
    ui->setupUi(this);

    myTimer = new QTimer(this);
    myTimer2 = new QTimer(this);

    mySerial = new QSerialPort(this);
    mySocket = new QUdpSocket(this);
    mySettings = new SettingsDialog();
    myPaintBox = new QPaintBox(0,0,ui->Grafica);
    // my3DPaintBox = new QPaintBox(0,0,ui->Grafica_2);


    series = new QLineSeries();
    seriesP = new QLineSeries();
    seriesI = new QLineSeries();
    seriesD = new QLineSeries();
    chart = new QChart();
    axisX = new QValueAxis();
    axisY = new QValueAxis();
    upperSeries = new QLineSeries();
    lowerSeries = new QLineSeries();
    highlightArea = new QAreaSeries(upperSeries, lowerSeries);
    upperSeries2 = new QLineSeries();
    lowerSeries2 = new QLineSeries();
    highlightArea2 = new QAreaSeries(upperSeries2, lowerSeries2);

    seriesAx = new QLineSeries();
    seriesAy = new QLineSeries();
    seriesAz = new QLineSeries();
    chart_Tracker = new QChart();
    axisX_Tracker = new QValueAxis();
    axisY_Tracker = new QValueAxis();

    seriesGx = new QLineSeries();
    seriesGy = new QLineSeries();
    seriesGz = new QLineSeries();
    chart_Tracker_G = new QChart();
    axisX_Tracker_G = new QValueAxis();
    axisY_Tracker_G = new QValueAxis();


    seriesXY = new QLineSeries();
    chart_Pos = new QChart();
    axisX_Pos = new QValueAxis();
    axisY_Pos = new QValueAxis();

    estado = new QLabel;
    wifi_estado = new QLabel;
    mySocket->bind(QHostAddress::AnyIPv4,udpPort);

    QString iniPath = QCoreApplication::applicationDirPath() + "/init.ini";
    settings = new QSettings(iniPath, QSettings::IniFormat, this);


    chart_PID_Setup();
    chart_Tracker_Setup();
    chart_Tracker_G_Setup();
    chart_Pos_Setup();

    localAddress = mySocket->localAddress();

    const QList<QHostAddress> &allAddresses = QNetworkInterface::allAddresses();
    for (const QHostAddress &addr : allAddresses) {
        if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
            addr != QHostAddress(QHostAddress::LocalHost)) {
            myIP = addr.toString();
            break;
        }
    }

    estado->setText("USB desconectado...");
    wifi_estado->setText("ESP01 desconectado de " + myIP);

    ui->statusbar->addWidget(estado);
    ui->statusbar->addWidget(wifi_estado);

    mySettings->setStyleSheet(this->styleSheet());

    ///Conexión de eventos
    connect(mySerial,&QSerialPort::readyRead,this, &velocista::readyUSBRead); //Si llega recibir
    connect(mySocket,&QUdpSocket::readyRead,this,&velocista::readyUDPRead);
    connect(myTimer, &QTimer::timeout,this, &velocista::myTimerOnTime); //intervalo de tiempo
    // connect(myTimer2, &QTimer::timeout,this, &velocista::drawAxes);
    connect(ui->pushButton_settings,&QPushButton::clicked,mySettings, &SettingsDialog::show);
    connect(ui->pushButton_open,&QPushButton::clicked,this, &velocista::openSerialPort); //Abrir puerto
    connect(ui->pushButton_close, &QPushButton::clicked, this, &velocista::closeSerialPort); //Cerrar puerto
    connect(ui->actionSalir,&QAction::triggered,this,&velocista::close); //Cerrar programa

    myTimer->start(10);
    myTimer2->start(100);

    workState = IDLE;

    init();
}

velocista::~velocista()
{
    delete ui;
}

//Tareas a realizar cuando se establece conexion
void velocista::openSerialPort()
{
    SettingsDialog::Settings p = mySettings->settings();
    //Configuracion de comunicacion
    mySerial->setPortName(p.name);
    mySerial->setBaudRate(p.baudRate);
    mySerial->setDataBits(p.dataBits);
    mySerial->setParity(p.parity);
    mySerial->setStopBits(p.stopBits);
    mySerial->setFlowControl(p.flowControl);
    mySerial->open(QSerialPort::ReadWrite);
    if(mySerial->isOpen()){
        ui->actionConectar->setEnabled(false);
        ui->actionDesconectar->setEnabled(true);
        estado->setText(tr("USB conectado a  %1 : %2, %3, %4, %5, %6  %7 ")
                            .arg(p.name).arg(p.stringBaudRate).arg(p.stringDataBits)
                            .arg(p.stringParity).arg(p.stringStopBits).arg(p.stringFlowControl)
                            .arg(p.fabricante));
    }
    else{
        QMessageBox::warning(this,"Menu Conectar","No se pudo abrir el puerto Serie.");
    }
}

//Tareas a realizar cuando se desconecta
void velocista::closeSerialPort()
{
    if(mySerial->isOpen()){
        mySerial->close();
        ui->actionDesconectar->setEnabled(false);
        ui->actionConectar->setEnabled(true);
        estado->setText("USB desconectado...");
    }
    else{
        estado->setText("USB desconectado...");
    }
}

//Primera conexión con la ESP01.
void velocista::openESP01(){

}

//Si timeout verificar si hay datos para recibir
void velocista::myTimerOnTime()
{
    // chart_Pos_Update(0,0,0);
    //Si timeout verificar si hay datos para recibir
    if(rxData.timeOut!=0){
        rxData.timeOut--;
    }else{
        estadoProtocolo = START;
    }
}

void velocista::main_print(QString txt,QColor color)
{
    QString mensaje;
    QString strHex;
    QString timestamp = QTime::currentTime().toString("HH:mm:ss");

    for (int i = 0; i <7 + txData.index; ++i) {
        mensaje.append(QChar(txData.payLoad[i]));
    }

    if(ui->time_stamp_main->isChecked()){
        timestamp = "[" + timestamp + "] ";
    }else{
        timestamp = "";
    }

    if(ui->hex_main->isChecked()){
        strHex = "";
        for(int i=0; i<7 + txData.index; i++){
            if (txData.payLoad[i] <= 33 || txData.payLoad[i] >= 150){
                strHex += QString("{%1}").arg(txData.payLoad[i], 2, 16, QChar('0')).toUpper();
            }else{
                strHex += QString("%1").arg(char(txData.payLoad[i]));
            }
        }
        strHex = "\n" + strHex;
    }else{
        strHex = "";
    }

    if(ui->raw_main->isChecked()){
        mensaje = ": " + mensaje;
    }else{
        mensaje = "";
    }

    if(ui->line_space_main->isChecked()){
        ui->textBrowser->setTextColor(color);
        ui->textBrowser->append("\n" + timestamp + txt + mensaje + strHex);
    }else{
        ui->textBrowser->setTextColor(color);
        ui->textBrowser->append(timestamp + txt + mensaje + strHex);
    }
}

void velocista::net_print(QString txt,QColor color)
{
    QString mensaje;
    QString strHex;
    QString timestamp = QTime::currentTime().toString("HH:mm:ss");

    for (int i = 0; i <7 + txData.index; ++i) {
        mensaje.append(QChar(txData.payLoad[i]));
    }

    if(ui->time_stamp_net->isChecked()){
        timestamp = "[" + timestamp + "] ";
    }else{
        timestamp = "";
    }

    if(ui->hex_net->isChecked()){
        strHex = "";
        for(int i=0; i<7 + txData.index; i++){
            if (txData.payLoad[i] <= 33 || txData.payLoad[i] >= 150){
                strHex += QString("{%1}").arg(txData.payLoad[i], 2, 16, QChar('0')).toUpper();
            }else{
                strHex += QString("%1").arg(char(txData.payLoad[i]));
            }
        }
        strHex = "\n" + strHex;
    }else{
        strHex = "";
    }

    if(ui->raw_net->isChecked()){
        mensaje = ": " + mensaje;
    }else{
        mensaje = "";
    }

    if(ui->line_space_net->isChecked()){
        ui->textBrowser->setTextColor(color);
        ui->textBrowser->append("\n" + timestamp + txt + mensaje + strHex);
    }else{
        ui->textBrowser->setTextColor(color);
        ui->textBrowser->append(timestamp + txt + mensaje + strHex);
    }
}

// ╔══════════════════════════════════════════════════════════════╗
// ║                            INIT                              ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::init(){
    int idx = settings->value("Appearance/StyleIndex", 0).toInt();


    // CONFIG_A
    settings->beginGroup("CONFIG_A");
    automatic.send = settings->value("send").toInt();
    automatic.deadZone = settings->value("deadZone").toInt();
    automatic.returnZone = settings->value("returnZone").toInt();
    automatic.accEntry = settings->value("accEntry").toUInt();
    automatic.accZone = settings->value("accZone").toUInt();
    automatic.accSpeed = settings->value("accSpeed").toUInt();
    automatic.backSpeed = settings->value("backSpeed").toUInt();
    automatic.lostSpeed = settings->value("lostSpeed").toUInt();
    automatic.kp_L = settings->value("kp_L").toUInt();
    automatic.ki_L = settings->value("ki_L").toUInt();
    automatic.kd_L = settings->value("kd_L").toUInt();
    automatic.kp_R = settings->value("kp_R").toUInt();
    automatic.ki_R = settings->value("ki_R").toUInt();
    automatic.kd_R = settings->value("kd_R").toUInt();
    automatic.mid_Speed = settings->value("mid_speed").toULongLong();
    settings->endGroup();


    ui->lineEdit_deadZone->setText(QString::number(automatic.deadZone));
    ui->lineEdit_returnZone->setText(QString::number(automatic.returnZone));
    ui->config_lineEdit->setText(QString::number(automatic.send));
    ui->lineEdit_accZone->setText(QString::number(automatic.accZone));
    ui->lineEdit_accEntry->setText(QString::number(automatic.accEntry));
    ui->lineEdit_accSpeed->setText(QString::number(automatic.accSpeed));
    ui->lineEdit_backSpeed->setText(QString::number(automatic.backSpeed));
    ui->lineEdit_lostSpeed->setText(QString::number(automatic.lostSpeed));
    ui->P_L_lineEdit->setText(QString::number(automatic.kp_L));
    ui->I_L_lineEdit->setText(QString::number(automatic.ki_L));
    ui->D_L_lineEdit->setText(QString::number(automatic.kd_L));
    ui->P_R_lineEdit->setText(QString::number(automatic.kp_R));
    ui->I_R_lineEdit->setText(QString::number(automatic.ki_R));
    ui->D_R_lineEdit->setText(QString::number(automatic.kd_R));
    ui->mid_Speed_lineEdit->setText(QString::number(automatic.mid_Speed));

    // CONFIG_T
    settings->beginGroup("CONFIG_T");
    tracker.send = settings->value("send").toInt();
    tracker.deadZone = settings->value("deadZone").toInt();
    tracker.returnZone = settings->value("returnZone").toInt();
    tracker.accEntry = settings->value("accEntry").toUInt();
    tracker.accZone = settings->value("accZone").toUInt();
    tracker.accSpeed = settings->value("accSpeed").toUInt();
    tracker.backSpeed = settings->value("backSpeed").toUInt();
    tracker.lostSpeed = settings->value("lostSpeed").toUInt();
    tracker.kp_L = settings->value("kp_L").toUInt();
    tracker.ki_L = settings->value("ki_L").toUInt();
    tracker.kd_L = settings->value("kd_L").toUInt();
    tracker.kp_R = settings->value("kp_R").toUInt();
    tracker.ki_R = settings->value("ki_R").toUInt();
    tracker.kd_R = settings->value("kd_R").toUInt();
    tracker.mid_Speed = settings->value("mid_speed").toULongLong();
    settings->endGroup();


    ui->lineEdit_deadZone_2->setText(QString::number(tracker.deadZone));
    ui->lineEdit_returnZone_2->setText(QString::number(tracker.returnZone));
    ui->config_lineEdit_2->setText(QString::number(tracker.send));
    ui->lineEdit_accZone_2->setText(QString::number(tracker.accZone));
    ui->lineEdit_accEntry_2->setText(QString::number(tracker.accEntry));
    ui->lineEdit_accSpeed_2->setText(QString::number(tracker.accSpeed));
    ui->lineEdit_backSpeed_2->setText(QString::number(tracker.backSpeed));
    ui->lineEdit_lostSpeed_2->setText(QString::number(tracker.lostSpeed));
    ui->P_L_lineEdit_2->setText(QString::number(tracker.kp_L));
    ui->I_L_lineEdit_2->setText(QString::number(tracker.ki_L));
    ui->D_L_lineEdit_2->setText(QString::number(tracker.kd_L));
    ui->P_R_lineEdit_2->setText(QString::number(tracker.kp_R));
    ui->I_R_lineEdit_2->setText(QString::number(tracker.ki_R));
    ui->D_R_lineEdit_2->setText(QString::number(tracker.kd_R));
    ui->mid_Speed_lineEdit_2->setText(QString::number(tracker.mid_Speed));

    ui->horizontalSlider->setSliderPosition(idx);
    on_horizontalSlider_valueChanged(idx);

    ui->checkBox_Error->setChecked(true);
}

// ╔══════════════════════════════════════════════════════════════╗
// ║                       dataRecived                            ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::dataRecived(uint8_t *incomingBuffer, int count, uint8_t via)
{
    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
            case START:
                if (incomingBuffer[i]=='U'){
                    estadoProtocolo=HEADER_1;
                    rxData.cheksum=0;
                }
                break;
            case HEADER_1:
                if (incomingBuffer[i]=='N'){
                    estadoProtocolo=HEADER_2;
                }else{
                    estadoProtocolo=START;
                }
                break;
            case HEADER_2:
                if (incomingBuffer[i]=='E'){
                    estadoProtocolo=HEADER_3;
                }else{
                    estadoProtocolo=START;
                }
                break;
            case HEADER_3:
                if (incomingBuffer[i]=='R'){
                    estadoProtocolo=NBYTES;
                }else{
                    estadoProtocolo=START;
                }
                break;
            case NBYTES:
                rxData.nBytes=incomingBuffer[i];
                estadoProtocolo=TOKEN;
                break;
            case TOKEN:
                if (incomingBuffer[i]==':'){
                    estadoProtocolo=ID;
                    rxData.index=0;
                }
                else{
                    estadoProtocolo=START;
                }
                break;
            case ID:
                rxData.id = incomingBuffer[i];
                rxData.cheksum = 'U' ^ 'N' ^ 'E' ^ 'R' ^ rxData.nBytes ^ ':' ^ incomingBuffer[i];
                rxData.nBytes--;
                if(rxData.nBytes == 0)
                    estadoProtocolo = CHECKSUM;
                else
                    estadoProtocolo=PAYLOAD;

                break;
            case PAYLOAD:
                rxData.payLoad[rxData.index++]=incomingBuffer[i];
                rxData.cheksum^=incomingBuffer[i];
                rxData.nBytes--;

                if(rxData.nBytes == 0){
                    estadoProtocolo = CHECKSUM;

                    QString strHex;
                    strHex = "";
                    for(int i=0; i<count; i++){
                        if (incomingBuffer[i] <= 33 || incomingBuffer[i] >= 150){
                            strHex += QString("{%1}").arg(incomingBuffer[i], 2, 16, QChar('0')).toUpper();
                        }else{
                            strHex += QString("%1").arg(char(incomingBuffer[i]));
                        }
                    }
                }

                break;
            case CHECKSUM:
                if(rxData.cheksum == incomingBuffer[i]){
                    decodeData(via);
                }
                break;
            default:
                estadoProtocolo=START;
                break;
        }
    }
    delete [] incomingBuffer;
}

// ╔══════════════════════════════════════════════════════════════╗
// ║                       decodeData                             ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::decodeData(uint8_t via)
{
    QString mensaje;
    QString timestamp = QTime::currentTime().toString("HH:mm:ss");

    switch (rxData.id) {
    case ACK:
        main_print("Micro: ACK",Qt::blue);
        // ui->textBrowser->setTextColor(Qt::blue);
        // ui->textBrowser->append("[" + timestamp + "] Micro: ACK");
        break;
    case ALIVE:
        if(via == 0){//USB
            ui->textBrowser->setTextColor(Qt::cyan);
            ui->textBrowser->append("[" + timestamp + "] Micro: ALIVE");
        }else{//UDP
            ui->textBrowser->setTextColor(Qt::blue);
            ui->textBrowser->append("[" + timestamp + "] Micro: ALIVE 1");

            wifi_estado->setText(tr("ESP01 conectado a %1 desde %2 : %3")
                                     .arg(myIP)
                                     .arg(datagram.senderAddress().toString())
                                     .arg(datagram.senderPort()));
        }
        break;
    case CMD_MPU:
        if(via == 0){//USB
            ui->textBrowser->setTextColor(Qt::blue);
            ui->textBrowser->append("[" + timestamp + "] Micro: MPU");
        }else{//UDP
            ui->textBrowser->setTextColor(Qt::darkBlue);
            ui->textBrowser->append("[" + timestamp + "] ESP01: MPU");
            offset = 0;
            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            A_X = myWord.as_float;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            A_Y = myWord.as_float;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            A_Z = myWord.as_float;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            G_X = myWord.as_float;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            G_Y = myWord.as_float;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            G_Z = myWord.as_float;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            angle = myWord.int16_parts.low_int16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[0] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[1] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[2] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[3] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[4] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[5] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[6] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            adcVal[7] = myWord.uint16_parts.low_uint16;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            error_int = myWord.as_int32;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            P = myWord.as_int32;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            I = myWord.as_int32;

            myWord.bytes.byte0 = rxData.payLoad[offset++];
            myWord.bytes.byte1 = rxData.payLoad[offset++];
            myWord.bytes.byte2 = rxData.payLoad[offset++];
            myWord.bytes.byte3 = rxData.payLoad[offset++];
            D = myWord.as_int32;


            // display

            ui->aX_MPU_lcdNumber->display(A_X);
            ui->aY_MPU_lcdNumber->display(A_Y);
            ui->aZ_MPU_lcdNumber->display(A_Z);
            ui->gX_MPU_lcdNumber->display(G_X);
            ui->gY_MPU_lcdNumber->display(G_Y);
            ui->gZ_MPU_lcdNumber->display(G_Z);

            // ui->ADC0_lcdNumber_1->display(adcVal[0]);
            // ui->ADC0_lcdNumber_2->display(adcVal[1]);
            // ui->ADC0_lcdNumber_3->display(adcVal[2]);
            // ui->ADC0_lcdNumber_4->display(adcVal[3]);
            // ui->ADC0_lcdNumber_5->display(adcVal[4]);
            // ui->ADC0_lcdNumber_6->display(adcVal[5]);
            // ui->ADC0_lcdNumber_7->display(adcVal[6]);
            // ui->ADC0_lcdNumber_8->display(adcVal[7]);

            // ui->ADC0_1->setValue(adcVal[0]);
            // ui->ADC0_2->setValue(adcVal[1]);
            // ui->ADC0_3->setValue(adcVal[2]);
            // ui->ADC0_4->setValue(adcVal[3]);
            // ui->ADC0_5->setValue(adcVal[4]);
            // ui->ADC0_6->setValue(adcVal[5]);
            // ui->ADC0_7->setValue(adcVal[6]);
            // ui->ADC0_8->setValue(adcVal[7]);

            ui->error_lcdNumber->display(error_int);
            ui->P_lcdNumber->display(P);
            ui->I_lcdNumber->display(I);
            ui->D_lcdNumber->display(D);




            // Error - PID
            chart_PID_Update(error_int, P, I, D);
            chart_Tracker_Update(A_X,A_Y,A_Z);
            chart_Tracker_G_Update(G_X,G_Y,G_Z);
            chart_Pos_Update(A_X,A_Y,angle);
            paint();

            error_prom(error_int);



        }
        break;
    case ESP01_CONFIG:
        for (int i = 0; i < rxData.index; ++i) {
            mensaje.append(QChar(rxData.payLoad[i]));
        }
        ui->textBrowser->setTextColor(Qt::darkGray);
        ui->textBrowser->append("[" + timestamp + "] ESP01: " + mensaje);
        break;
    case CMD_WORK_MODE:
        offset = 0;
        mode = rxData.payLoad[offset++];
        // ui->mode_label->setNum(mode);
        ui->textBrowser->setTextColor(Qt::blue);
        ui->textBrowser->append("[" + timestamp + "] Micro: MODE: " + QString::number(mode));
        break;
    default:
        for (int i = 0; i < rxData.index; ++i) {
            mensaje.append(QChar(rxData.payLoad[i]));
        }
        ui->textBrowser->setTextColor(Qt::red);
        ui->textBrowser->append("[" + timestamp + "] ID ERROR: " + mensaje);
        break;
    }
}

// ╔══════════════════════════════════════════════════════════════╗
// ║                          ENVÍO USB                           ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::sendUSB()
{
    QString mensaje;
    QString timestamp = QTime::currentTime().toString("HH:mm:ss");

    txData.index=0;
    txData.payLoad[0]='U';
    txData.payLoad[1]='N';
    txData.payLoad[2]='E';
    txData.payLoad[3]='R';
    txData.payLoad[5]=':';

    switch (estadoUSB) {
    case ACK:
        txData.payLoad[6]=ACK;
        main_print("PC: ACK", Qt::green);
        // ui->textBrowser->setTextColor(Qt::darkGreen);
        // ui->textBrowser->append("[" + timestamp + "] PC: ACK");
        break;
    case ALIVE:
        txData.payLoad[6]=ALIVE;
        ui->textBrowser->setTextColor(Qt::darkGreen);
        ui->textBrowser->append("[" + timestamp + "] PC: ALIVE");
        break;
    case ESP01_QT_READY:
        // txData.payLoad[txData.index++]=ESP01_QT_READY;
        // ui->textBrowser->setTextColor(Qt::darkGreen);
        // ui->textBrowser->append("[" + timestamp + "] PC: ESP01_QT_READY");
        break;
    case ESP01_CONFIG:
        txData.payLoad[6]=ESP01_CONFIG;

        txData.payLoad[7 + txData.index++]=':'; // SSID

        if(currentConfig.ssid[0] != '\0'){
            for (size_t i = 0; i < strlen(currentConfig.ssid); ++i) {
              txData.payLoad[7 + txData.index++]=currentConfig.ssid[i];
            }
        }

        txData.payLoad[7 + txData.index++]=':'; // PASS

        if(currentConfig.password[0] != '\0'){
            for (size_t i = 0; i < strlen(currentConfig.password); ++i) {
                txData.payLoad[7 + txData.index++]=currentConfig.password[i];
            }
        }
        txData.payLoad[7 + txData.index++]=':'; // IP

        if(currentConfig.remoteIP[0] != '\0'){
            for (size_t i = 0; i < strlen(currentConfig.remoteIP); ++i) {
                txData.payLoad[7 + txData.index++]=currentConfig.remoteIP[i];
            }
        }

        txData.payLoad[7 + txData.index++]=':';

        main_print("CONFIG",Qt::blue);

        break;
    default:
        break;
    }

    txData.payLoad[4] = txData.index + 1;
    txData.cheksum=0;
    for(int a=0 ;a<7+txData.index;a++)
        txData.cheksum^=txData.payLoad[a];
    txData.payLoad[7 + txData.index]=txData.cheksum;


    if(mySerial->isWritable()){
        mySerial->write((char *)txData.payLoad,txData.index + 8);
    }

    QString strHex;
    strHex = "";
    for(int i=0; i<8+txData.index; i++){
        if (txData.payLoad[i] <= 33 || txData.payLoad[i] >= 150){
            strHex += QString("{%1}").arg(txData.payLoad[i], 2, 16, QChar('0')).toUpper();
        }else{
            strHex += QString("%1").arg(char(txData.payLoad[i]));
        }
    }

    for (int i = 0; i <7+txData.index; ++i) {
        mensaje.append(QChar(txData.payLoad[i]));
    }
    ui->textBrowser_Net->setTextColor(Qt::red);
    ui->textBrowser_Net->append("[" + timestamp + "] USB DB: " + mensaje);
    ui->textBrowser_Net->append("[" + timestamp + "] USB HEX: " + strHex);


}

// ╔══════════════════════════════════════════════════════════════╗
// ║                           ENVÍO UDP                          ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::sendUDP()
{
    float value;

    txData.index=0;
    txData.payLoad[0]='U';
    txData.payLoad[1]='N';
    txData.payLoad[2]='E';
    txData.payLoad[3]='R';
    txData.payLoad[5]=':';

    switch (estadoUDP) {
    case ESP01_QT_READY:
        txData.payLoad[6]=ESP01_QT_READY;
        break;
    case ESP01_QT_OUT:
        txData.payLoad[6]=ESP01_QT_OUT;
        break;
    case CMD_MOTORS:
        txData.payLoad[6]=CMD_MOTORS;

        value = ui->mid_Speed_lineEdit->text().toUShort();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        // value = ui->left_Speed_lineEdit->text().toUShort();

        // myWord.uint16_parts.low_uint16 = value;
        // txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        // txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        // value = ui->right_Speed_lineEdit->text().toUShort();

        // myWord.uint16_parts.low_uint16 = value;
        // txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        // txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        break;
    case CMD_CONFIG_A:
        txData.payLoad[6]=CMD_CONFIG_A;

        value = ui->mid_Speed_lineEdit->text().toUShort();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        value = ui->P_L_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->I_L_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->D_L_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->P_R_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->I_R_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->D_R_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->config_lineEdit->text().toULong();

        myWord.as_uint32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->lineEdit_deadZone->text().toULong();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        value = ui->lineEdit_returnZone->text().toULong();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        value = ui->lineEdit_accZone->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_accEntry->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_accSpeed->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_backSpeed->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_lostSpeed->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        break;
    case CMD_CONFIG_T:
        txData.payLoad[6]=CMD_CONFIG_T;

        value = ui->mid_Speed_lineEdit_2->text().toUShort();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        value = ui->P_L_lineEdit_2->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->I_L_lineEdit_2->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->D_L_lineEdit_2->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->P_R_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->I_R_lineEdit_2->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->D_R_lineEdit->text().toFloat();

        myWord.as_int32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->config_lineEdit_2->text().toULong();

        myWord.as_uint32 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte2;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte3;

        value = ui->lineEdit_deadZone_2->text().toULong();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        value = ui->lineEdit_returnZone_2->text().toULong();

        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte1;

        value = ui->lineEdit_accZone_2->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_accEntry_2->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_accSpeed_2->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_backSpeed_2->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;

        value = ui->lineEdit_lostSpeed_2->text().toUInt();
        myWord.uint16_parts.low_uint16 = value;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        break;
    case CMD_WORK_MODE:
        txData.payLoad[6]=CMD_WORK_MODE;

        myWord.uint16_parts.low_uint16 = mode;
        txData.payLoad[7 + txData.index++] = myWord.bytes.byte0;
        break;
    case CMD_MPU:
        txData.payLoad[6]=CMD_MPU;

        break;
    default:
        break;
    }

    txData.payLoad[4] = txData.index + 1;
    txData.cheksum=0;
    for(int a = 0 ;a < 7 + txData.index;a++)
        txData.cheksum ^= txData.payLoad[a];
    txData.payLoad[7 + txData.index] = txData.cheksum;

    esp01Address = datagram.senderAddress();
    QByteArray data(QByteArray::fromRawData((char*)txData.payLoad,txData.index + 8));
    mySocket->writeDatagram(data,esp01Address,udpPortESP01);

    // quint16 localPort = mySocket->localPort();

    // ui->textBrowser->append("[ " + myIP + " : " + QString::number(localPort) + " ] > " + data);

    QString mensaje;
    QString timestamp = QTime::currentTime().toString("HH:mm:ss");

    for (int i = 0; i < 7+txData.index; ++i) {
        mensaje.append(QChar(txData.payLoad[i]));
    }
    ui->textBrowser->setTextColor(Qt::red);
    ui->textBrowser->append("[" + timestamp + "] ESP01 DB: " + mensaje);

    QString strHex;
    strHex = "";
    for(int i=0; i<8+txData.index; i++){
        if (txData.payLoad[i] <= 33 || txData.payLoad[i] >= 150){
            strHex += QString("{%1}").arg(txData.payLoad[i], 2, 16, QChar('0')).toUpper();
        }else{
            strHex += QString("%1").arg(char(txData.payLoad[i]));
        }
    }

    ui->textBrowser_Net->setTextColor(Qt::yellow);
    ui->textBrowser_Net->append("[" + timestamp + "] UDP HEX: " + strHex);


}

// ╔══════════════════════════════════════════════════════════════╗
// ║                       RECEPCIÓN UDP                          ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::readyUDPRead()
{
    unsigned char *incomingBuffer;
    datagram = mySocket->receiveDatagram();
    uint8_t size = datagram.data().size();
    flags1.bit.via = true;
    incomingBuffer = new unsigned char[size];
    memcpy(incomingBuffer, datagram.data().constData(), size);
    dataRecived(incomingBuffer, size,flags1.bit.via);
    localAddress = mySocket->localAddress();
}

// ╔══════════════════════════════════════════════════════════════╗
// ║                       RECEPCIÓN USB                          ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::readyUSBRead(){
    QString mensaje;
    QString timestamp = QTime::currentTime().toString("HH:mm:ss");
    unsigned char *incomingBuffer;
    int count;

    flags1.bit.via = false;

    count = sizeof(incomingBuffer);

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    mySerial->read((char *)incomingBuffer,count);

    for (int i = 0; i < count; ++i) {
        mensaje.append(QChar(incomingBuffer[i]));
    }


    QString strHex;
    strHex = "";
    for(int i=0; i<count; i++){
        if (incomingBuffer[i] <= 33 || incomingBuffer[i] >= 150){
            strHex += QString("{%1}").arg(incomingBuffer[i], 2, 16, QChar('0')).toUpper();
        }else{
            strHex += QString("%1").arg(char(incomingBuffer[i]));
        }
    }

    ui->textBrowser_Net->setTextColor(Qt::yellow);
    ui->textBrowser_Net->append("USB: " + mensaje);
    ui->textBrowser_Net->setTextColor(Qt::green);
    ui->textBrowser_Net->append("[" + timestamp + "] USB HEX: " + strHex);

    dataRecived(incomingBuffer, count, flags1.bit.via);
}

// Botón para enviar datos
void velocista::on_USB_Push_Button_clicked()
{
    // // QString message = ui->lineEdit->text().trimmed(); // Obtener y limpiar texto
    // QByteArray byteArray = message.toUtf8();  // Convertir a QByteArray

    // // int selectedIndex = ui->CMD_comboBox->currentIndex();
    // auxATLength = byteArray.size(); // Máximo 49 caracteres + terminador nulo

    // switch (selectedIndex) {
    // case 0:
    //     estadoUSB = ACK;
    //     break;
    // case 1:
    //     estadoUSB = ALIVE;
    //     break;
    // case 2:
    //     estadoUSB = ESP01_CONFIG;
    //     // qstrncpy(currentConfig.ssid, byteArray_SSID.constData(), 63);
    //     // qstrncpy(currentConfig.password, byteArray_PASSWORD.constData(), 63);
    //     // qstrncpy(currentConfig.remoteIP, byteArray_IP.constData(), 63);
    //     // currentConfig.remotePort = _PORT;
    //     // currentConfig.localPort = _LOCALPORT;
    //     break;
    // default:
    //     break;
    // }

    // sendUSB();
    // ui->lineEdit->clear(); // Limpiar el campo después de enviar

}


void velocista::on_WiFi_Push_Button_clicked()
{
    estadoUSB = ESP01_CONFIG;
    mySocket->writeDatagram((char *)txData.payLoad,txData.payLoad[NBYTES]+6,datagram.senderAddress(),datagram.senderPort());
}





void velocista::on_motores_pushButton_clicked()
{
    estadoUDP = CMD_MOTORS;
    sendUDP();
}

void velocista::on_ON_clicked()
{
    estadoUDP = ESP01_QT_READY;
    sendUDP();
}


void velocista::on_OFF_clicked()
{
    estadoUDP = ESP01_QT_OUT;
    sendUDP();
}



void velocista::on_pushButton_IP_clicked()
{

    QString message_SSID = ui->lineEdit_SSID->text().trimmed();
    QString message_PASSWORD = ui->lineEdit_PASS->text().trimmed();
    QString message_IP = ui->lineEdit_IP->text().trimmed();

    QByteArray byteArray_SSID = message_SSID.toUtf8();
    QByteArray byteArray_PASSWORD = message_PASSWORD.toUtf8();
    QByteArray byteArray_IP = message_IP.toUtf8();

    qstrncpy(currentConfig.ssid, byteArray_SSID.constData(), 63);
    qstrncpy(currentConfig.password, byteArray_PASSWORD.constData(), 63);
    qstrncpy(currentConfig.remoteIP, byteArray_IP.constData(), 63);

    estadoUSB = ESP01_CONFIG;
    sendUSB();
}


void velocista::on_send_Speed_pushButton_clicked()
{
    estadoUDP = CMD_MOTORS;
    sendUDP();
}


void velocista::on_Network_comboBox_currentIndexChanged(int index)
{
    QString section = ui->Network_comboBox->itemText(index);

    QString ssid = settings->value(section + "/SSID", "").toString();
    QString pass = settings->value(section + "/PASS", "").toString();

    ui->lineEdit_SSID->setText(ssid);
    ui->lineEdit_PASS->setText(pass);
}


void velocista::on_checkBox_stateChanged(int arg1)
{
    if (arg1 == Qt::Checked) {
        ui->lineEdit_PASS->setEchoMode(QLineEdit::Normal); // Mostrar el texto
    } else {
        ui->lineEdit_PASS->setEchoMode(QLineEdit::Password); // Ocultar el texto
    }
}

// ╔══════════════════════════════════════════════════════════════╗
// ║                       STYLE SELECTOR                         ║
// ╚══════════════════════════════════════════════════════════════╝
void velocista::on_horizontalSlider_valueChanged(int value)
{
    QFile  styleFile;

    if (value == 0) {
        styleFile.setFileName(QCoreApplication::applicationDirPath() + "/qss/dark_red.qss");
    } else if (value == 1) {
        styleFile.setFileName(QCoreApplication::applicationDirPath() + "/qss/light_red.qss");
    }

    styleFile.open(QFile::ReadOnly);

    QString  style(styleFile.readAll());
    setStyleSheet(style);
    mySettings->setStyleSheet(style);

    settings->setValue("Appearance/StyleIndex", value);
}




void velocista::on_pushButton_clicked()
{
    ui->textBrowser->clear();
}


void velocista::on_pushButton_save_clicked()
{


}


void velocista::error_prom(int32_t error){

    if(prom_init == 0){
        for (int i = 0; i < 200; ++i) {
            error_promedio[i] = 0;
        }
        prom_init = 1;
    }
    error_prom_suma -= error_promedio[index];

    error_promedio[index] = error;

    error_prom_suma += error;

    index++;
    if(index >= 200){
        ui->lcdNumber_error_prom->setStyleSheet("QLCDNumber { color: red; }");
        index = 0;
    }

    error_prom_cont = error_prom_suma / 200;

    ui->lcdNumber_error_prom->display(error_prom_cont);
}

void velocista::save_A(){
    // CONFIG_A
    automatic.kp_L = ui->P_L_lineEdit->text().toUInt();
    automatic.ki_L = ui->I_L_lineEdit->text().toUInt();
    automatic.kd_L = ui->D_L_lineEdit->text().toUInt();
    automatic.kp_R = ui->P_R_lineEdit->text().toUInt();
    automatic.ki_R = ui->I_R_lineEdit->text().toUInt();
    automatic.kd_R = ui->D_R_lineEdit->text().toUInt();
    automatic.mid_Speed = ui->mid_Speed_lineEdit->text().toULong();
    automatic.send = ui->config_lineEdit->text().toInt();
    automatic.deadZone = ui->lineEdit_deadZone->text().toInt();
    automatic.returnZone = ui->lineEdit_returnZone->text().toInt();
    automatic.accEntry = ui->lineEdit_accEntry->text().toUInt();
    automatic.accZone = ui->lineEdit_accZone->text().toUInt();
    automatic.accSpeed = ui->lineEdit_accSpeed->text().toUInt();
    automatic.backSpeed = ui->lineEdit_backSpeed->text().toUInt();
    automatic.lostSpeed = ui->lineEdit_lostSpeed->text().toUInt();

    settings->beginGroup("CONFIG_A");
    settings->setValue("send",automatic.send);
    settings->setValue("deadZone",automatic.deadZone);
    settings->setValue("returnZone",automatic.returnZone);
    settings->setValue("accEntry",automatic.accEntry);
    settings->setValue("accZone",automatic.accZone);
    settings->setValue("accSpeed",automatic.accSpeed);
    settings->setValue("backSpeed",automatic.backSpeed);
    settings->setValue("lostSpeed",automatic.lostSpeed);
    settings->setValue("kp_L",QString::number(automatic.kp_L));
    settings->setValue("ki_L",QString::number(automatic.ki_L));
    settings->setValue("kd_L",QString::number(automatic.kd_L));
    settings->setValue("kp_R",QString::number(automatic.kp_R));
    settings->setValue("ki_R",QString::number(automatic.ki_R));
    settings->setValue("kd_R",QString::number(automatic.kd_R));
    settings->setValue("mid_speed",QString::number(automatic.mid_Speed));
    settings->endGroup();

}

void velocista::save_T(){
    // CONFIG_T
    tracker.kp_L = ui->P_L_lineEdit_2->text().toUInt();
    tracker.ki_L = ui->I_L_lineEdit_2->text().toUInt();
    tracker.kd_L = ui->D_L_lineEdit_2->text().toUInt();
    tracker.kp_R = ui->P_R_lineEdit_2->text().toUInt();
    tracker.ki_R = ui->I_R_lineEdit_2->text().toUInt();
    tracker.kd_R = ui->D_R_lineEdit_2->text().toUInt();
    tracker.mid_Speed = ui->mid_Speed_lineEdit_2->text().toULong();
    tracker.send = ui->config_lineEdit_2->text().toInt();
    tracker.deadZone = ui->lineEdit_deadZone_2->text().toInt();
    tracker.returnZone = ui->lineEdit_returnZone_2->text().toInt();
    tracker.accEntry = ui->lineEdit_accEntry_2->text().toUInt();
    tracker.accZone = ui->lineEdit_accZone_2->text().toUInt();
    tracker.accSpeed = ui->lineEdit_accSpeed_2->text().toUInt();
    tracker.backSpeed = ui->lineEdit_backSpeed_2->text().toUInt();
    tracker.lostSpeed = ui->lineEdit_lostSpeed_2->text().toUInt();

    settings->beginGroup("CONFIG_T");
    settings->setValue("send",tracker.send);
    settings->setValue("deadZone",tracker.deadZone);
    settings->setValue("returnZone",tracker.returnZone);
    settings->setValue("accEntry",tracker.accEntry);
    settings->setValue("accZone",tracker.accZone);
    settings->setValue("accSpeed",tracker.accSpeed);
    settings->setValue("backSpeed",tracker.backSpeed);
    settings->setValue("lostSpeed",tracker.lostSpeed);
    settings->setValue("kp_L",QString::number(tracker.kp_L));
    settings->setValue("ki_L",QString::number(tracker.ki_L));
    settings->setValue("kd_L",QString::number(tracker.kd_L));
    settings->setValue("kp_R",QString::number(tracker.kp_R));
    settings->setValue("ki_R",QString::number(tracker.ki_R));
    settings->setValue("kd_R",QString::number(tracker.kd_R));
    settings->setValue("mid_speed",QString::number(tracker.mid_Speed));
    settings->endGroup();
}

void velocista::chart_PID_Setup(){
    // Crear la serie
    series->setName("Error");
    series->setColor(Qt::darkBlue);

    seriesP->setName("P");
    seriesP->setColor(Qt::red);

    seriesI->setName("I");
    seriesI->setColor(Qt::green);

    seriesD->setName("D");
    seriesD->setColor(Qt::blue);

    // Crear el gráfico
    chart->addSeries(series);
    chart->addSeries(seriesP);
    chart->addSeries(seriesI);
    chart->addSeries(seriesD);
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);

    // Creamos el área con esas series
    highlightArea->setBrush(QColor(66, 245, 144, 40));   // naranja con transparencia
    highlightArea->setPen(Qt::NoPen);

    highlightArea2->setBrush(QColor(255, 117, 117, 40));   // naranja con transparencia
    highlightArea2->setPen(Qt::NoPen);

    // Eje X (tiempo/muestras)
    axisX->setRange(0, 100);
    axisX->setLabelFormat("%d");

    // Eje Y (error)
    axisY->setRange(-1000, 1000);
    axisY->setLabelFormat("%d");
    axisY->setTitleText("Error");


    // Agregar ejes al gráfico
    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);

    // área
    chart->addSeries(highlightArea);
    highlightArea->attachAxis(axisX);
    highlightArea->attachAxis(axisY);

    chart->addSeries(highlightArea2);
    highlightArea2->attachAxis(axisX);
    highlightArea2->attachAxis(axisY);

    // Vincular la serie a los ejes
    series->attachAxis(axisX);
    series->attachAxis(axisY);
    seriesP->attachAxis(axisX);
    seriesP->attachAxis(axisY);
    seriesI->attachAxis(axisX);
    seriesI->attachAxis(axisY);
    seriesD->attachAxis(axisX);
    seriesD->attachAxis(axisY);

    // Mostrar en el chartView promovido
    ui->chartView->setChart(chart);
    ui->chartView->setRenderHint(QPainter::Antialiasing);
}


void velocista::chart_Tracker_Setup(){
    // Crear la serie
    seriesAx->setName("Ax");
    seriesAx->setColor(Qt::blue);

    seriesAy->setName("Ay");
    seriesAy->setColor(Qt::red);

    seriesAz->setName("Az");
    seriesAz->setColor(Qt::green);

    // Crear el gráfico
    chart_Tracker->addSeries(seriesAx);
    chart_Tracker->addSeries(seriesAy);
    chart_Tracker->addSeries(seriesAz);
    chart_Tracker->legend()->setVisible(true);
    chart_Tracker->legend()->setAlignment(Qt::AlignBottom);

    // Eje X
    axisX_Tracker->setRange(0, 100);
    axisX_Tracker->setLabelFormat("%d");

    // Eje Y
    axisY_Tracker->setRange(-1000, 1000);
    axisY_Tracker->setLabelFormat("%d");
    // axisY_Tracker->setTitleText("Acc");

    // Agregar ejes al gráfico
    chart_Tracker->addAxis(axisX_Tracker, Qt::AlignBottom);
    chart_Tracker->addAxis(axisY_Tracker, Qt::AlignLeft);

    // Vincular la serie a los ejes
    seriesAx->attachAxis(axisX_Tracker);
    seriesAx->attachAxis(axisY_Tracker);
    seriesAy->attachAxis(axisX_Tracker);
    seriesAy->attachAxis(axisY_Tracker);
    seriesAz->attachAxis(axisX_Tracker);
    seriesAz->attachAxis(axisY_Tracker);

    // Mostrar en el chartView promovido
    ui->chartView_Tracker->setChart(chart_Tracker);
    ui->chartView_Tracker->setRenderHint(QPainter::Antialiasing);
}

void velocista::chart_Tracker_G_Setup(){
    // Crear la serie
    seriesGx->setName("Gx");
    seriesGx->setColor(Qt::blue);

    seriesGy->setName("Gy");
    seriesGy->setColor(Qt::red);

    seriesGz->setName("Gz");
    seriesGz->setColor(Qt::green);

    // Crear el gráfico
    chart_Tracker_G->addSeries(seriesGx);
    chart_Tracker_G->addSeries(seriesGy);
    chart_Tracker_G->addSeries(seriesGz);
    chart_Tracker_G->legend()->setVisible(true);
    chart_Tracker_G->legend()->setAlignment(Qt::AlignBottom);

    // Eje X
    axisX_Tracker_G->setRange(0, 100);
    axisX_Tracker_G->setLabelFormat("%d");

    // Eje Y
    axisY_Tracker_G->setRange(-200, 200);
    axisY_Tracker_G->setLabelFormat("%d");

    // Agregar ejes al gráfico
    chart_Tracker_G->addAxis(axisX_Tracker_G, Qt::AlignBottom);
    chart_Tracker_G->addAxis(axisY_Tracker_G, Qt::AlignLeft);

    // Vincular la serie a los ejes
    seriesGx->attachAxis(axisX_Tracker_G);
    seriesGx->attachAxis(axisY_Tracker_G);
    seriesGy->attachAxis(axisX_Tracker_G);
    seriesGy->attachAxis(axisY_Tracker_G);
    seriesGz->attachAxis(axisX_Tracker_G);
    seriesGz->attachAxis(axisY_Tracker_G);

    // Mostrar en el chartView promovido
    ui->chartView_Tracker_G->setChart(chart_Tracker_G);
    ui->chartView_Tracker_G->setRenderHint(QPainter::Antialiasing);
}

void velocista::chart_Pos_Setup(){
    // Crear la serie
    seriesXY->setName("Pos");
    seriesXY->setColor(Qt::blue);

    // Crear el gráfico
    chart_Pos->addSeries(seriesXY);
    chart_Pos->legend()->setVisible(true);
    chart_Pos->legend()->setAlignment(Qt::AlignBottom);

    // Eje X
    axisX_Pos->setRange(-1000, 1000);
    axisX_Pos->setLabelFormat("%d");

    // Eje Y
    axisY_Pos->setRange(-1000, 1000);
    axisY_Pos->setLabelFormat("%d");
    // axisY_Tracker_G->setTitleText("Acc");

    // Agregar ejes al gráfico
    chart_Pos->addAxis(axisX_Pos, Qt::AlignBottom);
    chart_Pos->addAxis(axisY_Pos, Qt::AlignLeft);

    // Vincular la serie a los ejes
    seriesXY->attachAxis(axisX_Pos);
    seriesXY->attachAxis(axisY_Pos);

    // Mostrar en el chartView promovido
    ui->chartView_Pos->setChart(chart_Pos);
    ui->chartView_Pos->setRenderHint(QPainter::Antialiasing);
}

void velocista::chart_PID_Update(int32_t errorValue, int32_t P, int32_t I, int32_t D) {
    static int sampleCount = 0;

    upperSeries->append(sampleCount, automatic.deadZone);
    lowerSeries->append(sampleCount, -automatic.deadZone);

    upperSeries2->append(sampleCount, automatic.returnZone);
    lowerSeries2->append(sampleCount, -automatic.returnZone);

    if(ui->checkBox_Error->isChecked())
        series->append(sampleCount, errorValue);
    if(ui->checkBox_P->isChecked())
        seriesP->append(sampleCount, P);
    if(ui->checkBox_I->isChecked())
        seriesI->append(sampleCount, I);
    if(ui->checkBox_D->isChecked())
        seriesD->append(sampleCount, D);

    if (series->count() > 100) {

        if(upperSeries->count() > 100) upperSeries->remove(0);
        if(lowerSeries->count() > 100) lowerSeries->remove(0);

        if(upperSeries2->count() > 100) upperSeries2->remove(0);
        if(lowerSeries2->count() > 100) lowerSeries2->remove(0);

        if(series->count() > 100) series->remove(0);
        if(seriesP->count() > 100) seriesP->remove(0);
        if(seriesI->count() > 100) seriesI->remove(0);
        if(seriesD->count() > 100) seriesD->remove(0);

        axisX->setRange(sampleCount - 99, sampleCount + 1);
    } else {
        // Ajustar el rango inicial
        axisX->setRange(0, qMax(100, sampleCount + 1));
    }

    sampleCount++;

    ui->chartView->update();
}

void velocista::chart_Tracker_Update(int32_t Ax, int32_t Ay, int32_t Az){

    static int sampleCount = 0;

    seriesAx->append(sampleCount, Ax);
    seriesAy->append(sampleCount, Ay);
    seriesAz->append(sampleCount, Az);

    if (seriesAx->count() > 100) {


        if(seriesAx->count() > 100) seriesAx->remove(0);
        if(seriesAy->count() > 100) seriesAy->remove(0);
        if(seriesAz->count() > 100) seriesAz->remove(0);


        axisX_Tracker->setRange(sampleCount - 99, sampleCount + 1);
    } else {
        // Ajustar el rango inicial
        axisX_Tracker->setRange(0, qMax(100, sampleCount + 1));
    }

    sampleCount++;

    ui->chartView_Tracker->update();
}

void velocista::chart_Tracker_G_Update(int32_t Gx, int32_t Gy, int32_t Gz){
    static int sampleCount = 0;

    seriesGx->append(sampleCount, Gx);
    seriesGy->append(sampleCount, Gy);
    seriesGz->append(sampleCount, Gz);

    if (seriesGx->count() > 100) {


        if(seriesGx->count() > 100) seriesGx->remove(0);
        if(seriesGy->count() > 100) seriesGy->remove(0);
        if(seriesGz->count() > 100) seriesGz->remove(0);


        axisX_Tracker_G->setRange(sampleCount - 99, sampleCount + 1);
    } else {
        // Ajustar el rango inicial
        axisX_Tracker_G->setRange(0, qMax(100, sampleCount + 1));
    }

    sampleCount++;

    ui->chartView_Tracker_G->update();
}

void velocista::chart_Pos_Update(int32_t Ax, int32_t Ay, int16_t Rz){

    // Calcular dt automáticamente
    static auto last_time = std::chrono::high_resolution_clock::now();
    auto current_time = std::chrono::high_resolution_clock::now();
    float dt = std::chrono::duration<float>(current_time - last_time).count();
    last_time = current_time;

    int32_t dAx = Ax - lastAx;
    int32_t dAy = Ay - lastAy;

    // Evitar dA muy pequeños
    if(dAx <= 2)
        dAx = 0;

    if(dAy <= 2)
        dAy = 0;

    float angle_rad = Rz * (M_PI / 180.0);

    Xvel = dAx;// * dt;
    Yvel = dAy;// * dt;

    // Transformar velocidades del sistema local al sistema global usando la orientación
    float Xvel_global = Xvel * cos(angle_rad) + Yvel * sin(angle_rad);
    float Yvel_global = Xvel * sin(angle_rad) - Yvel * cos(angle_rad);

    // Integración: velocidad da el cambio de posición
    Xpos += Xvel_global;
    Ypos += Yvel_global;

    // Guardar valores para la próxima iteración
    lastAx = Ax;
    lastAy = Ay;

    ui->lcdNumber_Xvel->display(Xvel);
    ui->lcdNumber_Yvel->display(Yvel);

    ui->lcdNumber_Ypos->display(Ypos);
    ui->lcdNumber_Xpos->display(Xpos);

    // // Actualizar gráfico
    seriesXY->append(Xpos, Ypos);
    if (seriesXY->count() > 1000)
        seriesXY->remove(0);
    ui->chartView_Pos->update();
}

void velocista::on_config_pushButton_clicked()
{
    estadoUDP = CMD_CONFIG_A;
    automatic.send = ui->config_lineEdit->text().toInt();
    automatic.deadZone = ui->lineEdit_deadZone->text().toInt();
    automatic.returnZone = ui->lineEdit_returnZone->text().toInt();
    automatic.accEntry = ui->lineEdit_accEntry->text().toUInt();
    automatic.accZone = ui->lineEdit_accZone->text().toUInt();
    automatic.accSpeed = ui->lineEdit_accSpeed->text().toUInt();
    automatic.backSpeed = ui->lineEdit_backSpeed->text().toUInt();
    automatic.lostSpeed = ui->lineEdit_lostSpeed->text().toUInt();
    automatic.kp_L = ui->P_L_lineEdit->text().toLong();
    automatic.ki_L = ui->I_L_lineEdit->text().toLong();
    automatic.kd_L = ui->D_L_lineEdit->text().toLong();
    automatic.kp_R = ui->P_R_lineEdit->text().toLong();
    automatic.ki_R = ui->I_R_lineEdit->text().toLong();
    automatic.kd_R = ui->I_R_lineEdit->text().toLong();
    save_A();
    sendUDP();
}

void velocista::on_config_pushButton_2_clicked()
{
    estadoUDP = CMD_CONFIG_T;
    tracker.send = ui->config_lineEdit_2->text().toInt();
    tracker.deadZone = ui->lineEdit_deadZone_2->text().toInt();
    tracker.returnZone = ui->lineEdit_returnZone_2->text().toInt();
    tracker.accEntry = ui->lineEdit_accEntry_2->text().toUInt();
    tracker.accZone = ui->lineEdit_accZone_2->text().toUInt();
    tracker.accSpeed = ui->lineEdit_accSpeed_2->text().toUInt();
    tracker.backSpeed = ui->lineEdit_backSpeed_2->text().toUInt();
    tracker.lostSpeed = ui->lineEdit_lostSpeed_2->text().toUInt();
    tracker.kp_L = ui->P_L_lineEdit_2->text().toLong();
    tracker.ki_L = ui->I_L_lineEdit_2->text().toLong();
    tracker.kd_L = ui->D_L_lineEdit_2->text().toLong();
    tracker.kp_R = ui->P_R_lineEdit_2->text().toLong();
    tracker.ki_R = ui->I_R_lineEdit_2->text().toLong();
    tracker.kd_R = ui->I_R_lineEdit_2->text().toLong();
    save_T();
    sendUDP();
}


void velocista::on_manual_pushButton_clicked()
{
    mode = 0;
    estadoUDP = CMD_WORK_MODE;
    sendUDP();
}

void velocista::on_run_pushButton_clicked()
{
    mode = 1;
    estadoUDP = CMD_WORK_MODE;
    sendUDP();
}

void velocista::on_mant_pushButton_clicked()
{
    mode = 2;
    estadoUDP = CMD_WORK_MODE;
    sendUDP();
}


void velocista::on_pushButton_Reset_Offset_clicked()
{
    estadoUDP = CMD_MPU;
    sendUDP();
}

void velocista::on_pushButton_Set_Cero_clicked()
{
    Xpos = 0;
    Ypos = 0;
}


void velocista::on_pushButton_Clear_clicked()
{
    // chart_Pos->removeSeries(seriesXY);
    seriesXY->clear();
    ui->chartView_Pos->update();
}

void velocista::paint(){
    // Fondo negro
    if(ui->horizontalSlider->value() == 0){
        myPaintBox->getCanvas()->fill(QColor("#2d2d2d"));
    }else{
        myPaintBox->getCanvas()->fill(QColor("#ffffff"));
    }

    QPainter paint(myPaintBox->getCanvas());
    paint.setRenderHint(QPainter::Antialiasing);

    int width = myPaintBox->width();
    int height = myPaintBox->height();
    int centerX = width / 2;
    int centerY = height / 2;
    int radius = qMin(width, height) / 2 - 30;

    // Dibujar círculo de referencia
    QPen penCircle(Qt::darkGray, 1);
    paint.setPen(penCircle);
    paint.drawEllipse(centerX - radius, centerY - radius, radius * 2, radius * 2);

    // Dibujar marcas cada 30 grados
    QPen penMarks(Qt::gray, 1);
    paint.setPen(penMarks);
    paint.setFont(QFont("Arial", 8));

    for(int i = 0; i < 360; i += 30) {
        double markAngle = ((i) * M_PI) / 180.0;
        int markX = centerX + (int)((radius + 5) * cos(markAngle));
        int markY = centerY + (int)((radius + 5) * sin(markAngle));
        int innerX = centerX + (int)((radius - 5) * cos(markAngle));
        int innerY = centerY + (int)((radius - 5) * sin(markAngle));

        paint.drawLine(innerX, innerY, markX, markY);

        // Números cada 90 grados
        if(i % 90 == 0) {
            int textX = centerX + (int)((radius + 15) * cos(markAngle));
            int textY = centerY + (int)((radius + 15) * sin(markAngle));
            paint.drawText(textX - 5, textY + 3, QString::number(i));
        }
    }

    // Línea azul
    QPen penLine(Qt::blue, 4);
    paint.setPen(penLine);

    double angleRad = ((angle+90) * M_PI) / 180.0;
    int needleLength = qMin(width, height) / 2 - 20;

    // Calcular punto final (desde el centro hacia afuera)
    int endX = centerX + (int)((needleLength - 20) * cos(-angleRad));
    int endY = centerY + (int)((needleLength - 20) * sin(-angleRad));

    // Dibujar la aguja
    paint.drawLine(centerX, centerY, endX, endY);

    // Dibujar triángulo en la punta
    QPen penTriangle(Qt::blue, 2);
    paint.setPen(penTriangle);
    paint.setBrush(QBrush(Qt::blue));

    // Tamaño del triángulo
    const int triangleSize = 8;
    endX = centerX + (int)((needleLength-10) * cos(-angleRad));
    endY = centerY + (int)((needleLength-10) * sin(-angleRad));
    // Calcular puntos del triángulo
    QPointF tipPoint(endX, endY);
    QPointF direction(cos(-angleRad), sin(-angleRad));
    QPointF perpendicular(-direction.y(), direction.x()); // Vector perpendicular

    QPointF basePoint = tipPoint - triangleSize * direction;
    QPointF leftPoint = basePoint + triangleSize * 0.5 * perpendicular;
    QPointF rightPoint = basePoint - triangleSize * 0.5 * perpendicular;

    QPolygonF triangle;
    triangle << tipPoint << leftPoint << rightPoint;
    paint.drawPolygon(triangle);

    // Centro
    QPen penCenter(Qt::white, 2);
    paint.setPen(penCenter);
    paint.setBrush(QBrush(Qt::red));
    paint.drawEllipse(centerX - 5, centerY - 5, 10, 10);


    // Opcional: Mostrar el ángulo actual
    QPen penText(Qt::black, 1);
    if(ui->horizontalSlider->value() == 0){
        penText.setColor(Qt::white);
    }else{
        penText.setColor(Qt::black);
    }

    paint.setPen(penText);
    paint.setFont(QFont("Arial", 8));
    paint.drawText(10,20,"Ángulo: " + QString::number(angle));

    // Actualizar la pantalla
    myPaintBox->update();
}










void velocista::on_pushButton_2_clicked()
{
    chart->zoomOut();
    ui->chartView->update();
}


void velocista::on_pushButton_3_clicked()
{
    chart->zoomIn();
    ui->chartView->update();
}


void velocista::on_pushButton_4_clicked()
{
    chart_Pos->zoomOut();
    ui->chartView_Pos->update();
}


void velocista::on_pushButton_5_clicked()
{
    chart_Pos->zoomIn();
    ui->chartView_Pos->update();
}




void velocista::on_pushButton_SMA_Zero_clicked()
{
    error_prom_cont = 0;
    index = 0;
    error_prom_suma = 0;
    for (int j = 0; j < 200; ++j) {
        error_promedio[j] = 0;
    }
    ui->lcdNumber_error_prom->setStyleSheet("QLCDNumber { color: white; }");

}

