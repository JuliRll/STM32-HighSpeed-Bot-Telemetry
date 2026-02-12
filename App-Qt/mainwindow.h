#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QTimer>
#include <QTime>
#include <QLabel>
#include "settingsdialog.h"
#include <QNetworkDatagram>
#include <QtNetwork/QUdpSocket>
#include <QNetworkInterface>
#include <QStyle>
#include <QSettings>
#include <QColor>
#include <math.h>
#include "qpaintbox.h"

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QtCharts/QValueAxis>
#include <QtCharts/QAreaSeries>

QT_BEGIN_NAMESPACE
namespace Ui {
class velocista;
}
QT_END_NAMESPACE

class velocista : public QMainWindow
{
    Q_OBJECT

public:
    velocista(QWidget *parent = nullptr);
    ~velocista();

private slots:

    void dataRecived(uint8_t *incomingBuffer,int count,uint8_t via);

    void sendUSB();

    void decodeData(uint8_t via);

    void sendUDP();

    void readyUSBRead();

    void readyUDPRead();

    void myTimerOnTime();

    void openSerialPort();

    void closeSerialPort();

    void openESP01();

    void on_USB_Push_Button_clicked();

    void on_WiFi_Push_Button_clicked();

    void on_motores_pushButton_clicked();

    void init();

    void on_ON_clicked();

    void on_OFF_clicked();

    void on_pushButton_IP_clicked();

    void on_send_Speed_pushButton_clicked();

    void main_print(QString txt,QColor color);

    void net_print(QString txt,QColor color);

    void on_Network_comboBox_currentIndexChanged(int index);

    void on_checkBox_stateChanged(int arg1);

    void on_horizontalSlider_valueChanged(int value);

    void on_pushButton_clicked();

    void chart_PID_Setup();

    void chart_Tracker_Setup();

    void chart_Tracker_G_Setup();

    void chart_Pos_Setup();

    void chart_PID_Update(int32_t errorValue, int32_t P, int32_t I, int32_t D);

    void chart_Tracker_Update(int32_t Ax, int32_t Ay, int32_t Az);

    void chart_Tracker_G_Update(int32_t Gx, int32_t Gy, int32_t Gz);

    void chart_Pos_Update(int32_t Ax, int32_t Ay, int16_t Rz);

    void paint();

    void on_config_pushButton_clicked();

    void on_run_pushButton_clicked();

    void on_manual_pushButton_clicked();

    void on_mant_pushButton_clicked();

    void on_pushButton_save_clicked();

    void on_pushButton_Set_Cero_clicked();

    void on_pushButton_Clear_clicked();

    void on_pushButton_Reset_Offset_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_4_clicked();

    void on_pushButton_5_clicked();

    void on_config_pushButton_2_clicked();

    void save_A();

    void save_T();

    void error_prom(int32_t error);

    void on_pushButton_SMA_Zero_clicked();

private:
    Ui::velocista *ui;
    QSerialPort *mySerial;
    QUdpSocket *mySocket;
    QTimer *myTimer;
    QTimer *myTimer2;
    QTime last_time;
    SettingsDialog *mySettings;
    QLabel *estado;
    QLabel *wifi_estado;
    QSettings *settings;
    QPaintBox *myPaintBox,*my3DPaintBox;

    //PID chart
    QLineSeries *series, *seriesP, *seriesI, *seriesD;
    QChart *chart;
    QValueAxis *axisX;
    QValueAxis *axisY;
    int sampleIndex,sampleIndexPID; // contador de puntos
    QLineSeries *upperSeries,*lowerSeries,*upperSeries2,*lowerSeries2;
    QAreaSeries *highlightArea,*highlightArea2;

    //Tracker Pos
    QLineSeries *seriesXY;
    QChart *chart_Pos;
    QValueAxis *axisX_Pos;
    QValueAxis *axisY_Pos;

    //Tracker A
    QLineSeries *seriesAx,*seriesAy, *seriesAz;
    QChart *chart_Tracker;
    QValueAxis *axisX_Tracker;
    QValueAxis *axisY_Tracker;

    //Tracker G
    QLineSeries *seriesGx,*seriesGy, *seriesGz;
    QChart *chart_Tracker_G;
    QValueAxis *axisX_Tracker_G;
    QValueAxis *axisY_Tracker_G;

    QHostAddress sender;
    QNetworkDatagram datagram;
    QHostAddress localAddress,esp01Address;
    QString myIP;

    typedef enum{
        START,
        HEADER_1,
        HEADER_2,
        HEADER_3,
        NBYTES,
        TOKEN,
        ID,
        PAYLOAD,
        CHECKSUM
    }_eProtocolo;

    _eProtocolo estadoProtocolo;

    typedef enum{
        ACK = 0x0D,
        ALIVE = 0xF0,
        AT = 0x0A,

        ESP01_CONFIG = 0xF1,
        ESP01_QT_READY = 0xF2,
        ESP01_QT_OUT = 0xB2,
        CMD_MPU = 0xF3,
        CMD_MOTORS = 0xF4,
        CMD_CONFIG_A = 0xF6,
        CMD_CONFIG_T = 0xF5,
        CMD_WORK_MODE = 0xF8,
        OTHERS
    }_eID;

    _eID estadoUSB,estadoUDP;

    typedef enum{
        ESP01_CONFIG_SSID,
        ESP01_CONFIG_PASSWORD,
        ESP01_CONFIG_IP,
        ESP01_CONFIG_PORT,
        ESP01_CONFIG_LOCALPORT,
    }_eCONF;

    _eCONF conf;

    typedef enum{
        IDLE,
        FIRST_READY,
        PRINT_SCREEN,
        ESP01_ACK,
    } _eWORKSTATES;

    _eWORKSTATES workState;

    // Configuración de conexión
    typedef struct {
        char ssid[64];
        char password[64];
        char remoteIP[16];
        uint16_t remotePort;
        uint16_t localPort;
    } ConnectionConfig;

    ConnectionConfig currentConfig;

    typedef union {
        // Tipos completos de 32 bits
        float    as_float;
        int32_t  as_int32;
        uint32_t as_uint32;

        struct {
            uint8_t byte0;
            uint8_t byte1;
            uint8_t byte2;
            uint8_t byte3;
        } bytes;

        struct {
            int16_t  low_int16;   // Bytes 0-1
            int16_t  high_int16;  // Bytes 2-3 1010 0111 - 0000 1100
        } int16_parts;

        struct {
            uint16_t low_uint16;
            uint16_t high_uint16;
        } uint16_parts;

    } DataUnion32;

    DataUnion32 myWord;

    typedef union{
        struct{
            uint8_t via: 1;
            uint8_t f1: 1;
            uint8_t f2: 1;
            uint8_t f3: 1;
            uint8_t f4: 1;
            uint8_t f5: 1;
            uint8_t f6: 1;
            uint8_t f7: 1;
        }bit;
        uint8_t byte;
    }flags;

    flags flags1;

    typedef struct{
        uint8_t timeOut;
        uint8_t cheksum;
        uint8_t payLoad[256];
        uint8_t nBytes;
        uint8_t index;
        uint8_t id;
    }_sDatos ;

    _sDatos rxData, txData;

    // CONFIG
    typedef struct{
        uint32_t send;
        uint8_t follow_state;
        int16_t mid_Speed;
        uint16_t deadZone;
        uint16_t returnZone;
        uint8_t accZone;
        uint8_t accEntry;
        uint8_t accSpeed;
        uint8_t backSpeed;
        uint8_t lostSpeed;
        int32_t kp_L;
        int32_t ki_L;
        int32_t kd_L;
        int32_t kp_R;
        int32_t ki_R;
        int32_t kd_R;
    }mode_config;

    mode_config automatic, tracker;

    uint8_t auxATPayload[50];
    uint8_t auxATLength;

    uint16_t udpPort = 30001;
    uint16_t udpPortESP01 = 30010;

    // Show ADC Values
    uint16_t adcVal[8];

    // Show Motor Speed
    uint16_t speed;
    uint8_t mode;

    // PID
    int32_t error_int,P,I,D;
    int32_t error_promedio[200],error_prom_cont = 0, error_prom_suma = 0, index = 0;
    bool prom_init = 0;

    // MPU Values
    uint8_t offset = 0;
    float G_X,G_Y,G_Z;
    int16_t angle;
    int32_t A_X, A_Y, A_Z;
    int32_t Xvel = 0, Yvel = 0, Zvel;
    int32_t Xpos = 0, Ypos = 0;
    int32_t lastAx = 0;
    int32_t lastAy = 0;

};
#endif // MAINWINDOW_H
