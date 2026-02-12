/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  * Julián Rllepca
  *
  * Universidad Nacional de Entre Ríos
  * Facultad de Ciencias de la Alimentación
  *
  * Ingeniería Mecatrónica
  *
  * 2025
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <math.h>
#include <stdint.h>


#include "ssd1306.h"
#include "esp01_handler.h"
#include "fonts.h"
#include "protocol_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Comandos del protocolo de comunicación USB/WiFi
typedef enum{
	ACK = 0x0D,
	ALIVE = 0xF0,
	ESP01_CONFIG = 0xF1,
	ESP01_QT_READY = 0xF2,
	ESP01_QT_OUT = 0xB2,
	CMD_MPU = 0xF3,
	CMD_CONFIG_A = 0xF6,
	CMD_CONFIG_T = 0xF5,
	CMD_WORK_MODE = 0xF8,
	OTHERS
}_eID;

_eID estadoComandos;

typedef enum{
	ESP01_FIRST_READY,
	ESP01_RECONNECT,
	ESP01_MODE,
	ESP01_MPU,
} _eESP01STATES;

_eESP01STATES currentESP01states;

// Estados del botón
typedef enum {
  BTN_STATE_UP,      // Botón sin presionar
  BTN_STATE_DOWN,    // Botón presionado
  BTN_STATE_RISING,  // Transición de presionado a liberado
  BTN_STATE_FALLING  // Transición de liberado a presionado
} ButtonState;

// Flags
static union{
	struct{
		uint8_t payloadReadyUSB: 1;
		uint8_t sendReadyUSB: 1;
		uint8_t USB_state: 1;
		uint8_t updateScreen: 1;
		uint8_t screenState: 1;
		uint8_t motores: 1;
		uint8_t error_pHandler: 1;
		uint8_t samples: 1;
	}bit1;

	struct{
		uint8_t Acc_Ready_MPU: 1;
		uint8_t Gyr_Ready_MPU: 1;
		uint8_t Temp_Ready_MPU: 1;
		uint8_t Acc_Ask_MPU: 1;
		uint8_t Gyr_Ask_MPU: 1;
		uint8_t Temp_Ask_MPU: 1;
		uint8_t on_line: 1;
		uint8_t Gyr_First_Offset: 1;
	}bit2;

	struct{
		uint8_t turn_L: 1;
		uint8_t turn_R: 1;
		uint8_t f2: 1;
		uint8_t f3: 1;
		uint8_t f4: 1;
		uint8_t f5: 1;
		uint8_t f6: 1;
		uint8_t f7: 1;
	}bit3;

    struct {
        uint8_t byte1;  // Byte original
        uint8_t byte2;  // Nuevo byte
        uint8_t byte3;
    } bytes;
}flags;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TRUE 1
#define FALSE 0

// HARDWARE
#define NUM_SENSORS     8

// Time Management
#define TIME_MULTIPLIER 4

#define TIME_ELAPSED(now, last, interval_ms) \
    ((uint32_t)((now) - (last)) >= ((uint32_t)(interval_ms) * TIME_MULTIPLIER)) // Macro para rápida comparación de tiempos

#define UART_TIMEOUT_MS 500  // ms de inactividad para considerar la trama completa

// Button time management
#define DEBOUNCE_TIME 50 // Tiempo de debounce en milisegundos
#define HOLD_TIME 1000  // 1 segundo para detectar "hold"

// FILTRO SMA
#define N 5

// MPU6050
#define MPU6050_ADDR 0x68 << 1
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define FILTER	0x1A
#define WHO_AM_I_REG 0x75
#define ACC_RANGE 9.8f // Rango máximo asumido para aceleración en X e Y (en m/s²)

// Definiciones para el área del gráfico de barras Seguidor de línea
#define GRAPH_TOP_Y     22
#define GRAPH_BOTTOM_Y  48
#define GRAPH_HEIGHT    (GRAPH_BOTTOM_Y - GRAPH_TOP_Y)
#define GRAPH_LEFT_X    0
#define GRAPH_WIDTH     128

#define BAR_WIDTH       12
#define SPACING         4

#define CENTER_X 107
#define CENTER_Y 43
#define RADIUS   20



// Estados de seguimiento de línea
#define LOST		0
#define FOLLOWING	1

#define MOTOR_MIN_SPEED 50	// Velocidad mínima, para no forzar el driver


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c2_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

// ╔══════════════════════════════════════════════════════════════╗
// ║                    VARIABLES GLOBALES                        ║
// ╚══════════════════════════════════════════════════════════════╝

// Estructura para filtro SMA
typedef struct {
    uint16_t buffer[NUM_SENSORS][N];
    uint8_t idx;
    uint16_t sum[NUM_SENSORS];
    uint8_t conteo;
} _SMA_Filter;

_SMA_Filter SMA_Filter;

// Estructura para puntos de nivel
typedef struct {
    int16_t x;
    int16_t y;
} Point_t;

// Unión para trabajar tipos de datos
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
        int16_t  high_int16;  // Bytes 2-3
    } int16_parts;

    struct {
        uint16_t low_uint16;
        uint16_t high_uint16;
    } uint16_parts;

} DataUnion32;

DataUnion32 myWord;

// Estructura para trabajar botones
typedef struct {
  GPIO_TypeDef* GPIO_Port;  // Puerto GPIO
  uint16_t GPIO_Pin;        // Pin GPIO
  ButtonState state;        // Estado actual (UP/DOWN/RISING/FALLING)
  uint8_t last_state;       // Último estado leído (sin debounce)
  uint8_t debounced_state;  // Estado actual con debounce
  uint32_t last_time;       // Último tiempo de cambio
  uint32_t hold_time;       // Tiempo de inicio del "hold"
  uint8_t hold_detected;    // Flag para evitar repetición
  uint8_t hold_triggered;
  uint8_t hold_release;
} Button;


// Estructura de datos para comunicación WiFi
typedef struct {
	char ssid[64];
	char password[64];
    char remoteIP[16];
    char localIP[16];
    uint16_t remotePort;
    uint16_t localPort;
    bool needsUpdate; // Bandera para indicar si los parámetros deben actualizarse
} ConnectionConfig;

char *ip;

/*
	.ssid = "LabPrototip",
	.password = "labproto",
	.remoteIP = "192.168.2.103",
 */
/*
 	.ssid = "InternetPlus_ecf7f0",
	.password = "wlan13080f",
	.remoteIP = "192.168.1.2",
 */

// Estado inicial de conexión a la facultad
ConnectionConfig currentConfig = {
	.ssid = "LabPrototip",
	.password = "labproto",
	.remoteIP = "192.168.2.112",
	.localIP = "",
	.remotePort = 30010,
	.localPort = 30001,
	.needsUpdate = false
};

// Variables para manejo de datos de comunicación
_sRingBuffer bufferUSB;
_sRingBuffer bufferESP01;
_sRingBuffer bufferSTM32;

uint8_t payloadOffset = 0;

uint8_t tempRxUART[2];
uint8_t stateESP01;
uint8_t network;
uint8_t send_index;
uint16_t sendTime;

// HeartBeat
uint32_t hbStartTime = 0;
uint8_t hbStep = 0;

// SSD1306
char str[30];
uint8_t auxDB = 20;
int8_t screenState;
uint8_t optionState;
uint8_t CMD_Received;

// MPU6050
int16_t A_X, A_Y, A_Z;
int16_t G_X, G_Y, G_Z, Gz_offset;
int16_t T_MPU;
int32_t Gz_dps;
uint8_t i_raw,i_G,data_ac = 0;
int16_t angle = 0;

int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
int16_t T_RAW;

uint8_t Rec_Data_A[6],Rec_Data_G[6],Rec_Data_T[2];

// Botones
uint32_t last_time = 0;
uint8_t last_state = 1;
Button buttons[2] = {
  {GPIOB, GPIO_PIN_15, BTN_STATE_UP, 1, 1, 0}, // Botón 1 en PB15
  {GPIOB, GPIO_PIN_14, BTN_STATE_UP, 1, 1, 0}  // Botón 2 en PB14
};


//char ipBuffer[16] = {0};
//uint8_t data_tx[2];
//uint8_t data;

//ADC
uint8_t line = 0, last_line = 0,in_out = 0;
uint16_t count = 0, samples = 0, media = 0;
uint16_t adc_raw[8], adc_clean[8], adc_bit[8];

// WORK PID
uint8_t mode,accelCounter = 0;
int16_t left_Speed,M_left_Speed;
int16_t right_Speed,M_right_Speed;

int32_t PID_L,PID_R;
int32_t error_p, error_i, error_d;
int32_t last_d_error = 0;
int32_t error_t, P, I, D;

// Datos de configuración
typedef struct{
	uint32_t send_time;
	uint8_t follow_state;
	int16_t mid_Speed;
	uint16_t deadZone;
	uint16_t returnZone;
	uint8_t accZone;
	uint8_t accEntry;
	uint8_t acc_Speed;
	uint8_t back_Speed;
	uint8_t lost_Speed;
	int32_t kp_L;
	int32_t ki_L;
	int32_t kd_L;
	int32_t kp_R;
	int32_t ki_R;
	int32_t kd_R;
}mode_config;

mode_config automatic, tracker;

uint8_t turn_L = 0, turn_R = 0;

// Timers
typedef struct {
	uint32_t heartBeat;
	uint32_t screen;
	uint32_t uartReceive;
	uint32_t esp01Event;
	uint32_t esp01Send;
	uint32_t esp01Receive;
	uint32_t usbReceive;
	uint32_t adc;
	uint32_t ssd1306;
	uint32_t mpu;
	uint32_t gyro_time;
	uint32_t mpu_last_time;
	uint32_t error_p;
	uint32_t oneSec; // contador global de ticks (250 µs cada tick)
	uint32_t accel;
	uint32_t timeout_back_on_track;
	uint32_t PID;
} TimeEvents;


TimeEvents elapsed;

// error_p
uint8_t error_pLedState = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

// ╔══════════════════════════════════════════════════════════════╗
// ║                      FUNCIONES UTILES                        ║
// ╚══════════════════════════════════════════════════════════════╝

/**
 * @brief Inicializa el filtro de media móvil (SMA)
 *
 * Esta función inicializa la estructura del filtro SMA para los sensores.
 * Establece a cero todas las sumas acumuladas y limpia el buffer de muestras.
 * Reinicia el índice de posición y el contador de muestras.
 * Esta función debe ejecutarse antes de utilizar cualquier otra función del filtro SMA.
 *
 * @param [in] filter: Puntero a la estructura _SMA_Filter que será inicializada
 *
 */
void SMA_Init(_SMA_Filter *filter);// Sample Moving Average

/**
 * @brief Actualiza el filtro de media móvil (SMA) con nuevas muestras
 *
 * Esta función procesa nuevas lecturas del ADC a través del filtro SMA para múltiples sensores.
 * Durante las primeras N muestras (fase de llenado), calcula la media acumulativa con las muestras disponibles.
 * Una vez que el buffer está lleno, reemplaza la muestra más antigua con la nueva y calcula la media móvil.
 * También calcula una media global de todos los sensores después de procesar cada uno.
 * Esta función debe ejecutarse periódicamente con nuevas lecturas del ADC.
 *
 * @param [in] filter: Puntero a la estructura _SMA_Filter que contiene el estado del filtro
 *
 * @note Utiliza las variables globales adc_raw[] para entrada y actualiza adc_clean[] y media
 */
void SMA_Update(_SMA_Filter *filter);

/**
 * @brief Calcula la posición de la línea y actualiza el control PID
 *
 * Esta función procesa las lecturas de los sensores para determinar la posición de la línea
 * utilizando interpolación cuadrática.
 *
 * Encuentra el sensor con el valor mínimo (línea negra) y aplica un algoritmo de interpolación parabólica
 * para estimar la posición precisa de la línea.
 *
 * Calcula el error de posición respecto al centro y aplica control PID según el modo de operación
 * (automático o tracker) para generar las señales de control para los motores izquierdo y derecho.
 *
 * @note Utiliza variables globales: adc_clean[], adc_bit[], last_line, mode, automatic, tracker
 * @note Actualiza variables globales: error_t, error_p, error_i, error_d, last_d_error, PID_L, PID_R
 * @note La función no tiene parámetros de entrada pero opera sobre variables globales del sistema
 */
void position_IC();

/**
 * @brief Determina el estado de seguimiento de línea (FOLLOWING o LOST)
 *
 * Esta función evalúa las lecturas de los sensores para determinar si se
 * está detectando la línea.
 *
 * Cuenta la cantidad de sensores activos y establece el estado de seguimiento correspondiente.
 *
 * Si no se detecta línea (todos los sensores inactivos), activa el estado LOST.
 * Si se detecta línea, evalúa si el error de posición está dentro de la zona de retorno
 * para establecer el estado FOLLOWING según el modo de operación actual.
 *
 * @note Utiliza variables globales: adc_bit[], line, flags, tracker, automatic, mode, error_t
 * @note Actualiza variables globales: flags.bit2.on_line, tracker.follow_state, automatic.follow_state
 * @note La función no tiene parámetros de entrada pero opera sobre variables globales del sistema
 */
void follow_State();



/**
 * @brief Realiza el debounce y gestión de estados de botones
 *
 * Esta función gestiona el antirrebote (debounce) de un botón y detecta
 * los cambios de estado (presionado, liberado) así como la detección de
 * pulsación mantenida (hold).
 *
 * Utiliza una máquina de estados para gestionar las transiciones entre
 * estados del botón y considera los tiempos de rebote y hold configurados.
 *
 * @param [in] btn: Puntero a la estructura Button que contiene la configuración
 *                  y estado actual del botón a gestionar
 *
 * @note Utiliza las constantes DEBOUNCE_TIME y HOLD_TIME para los tiempos de
 *       debounce y detección de hold respectivamente
 */
void BT_Debounce(Button *btn);



/**
 * @brief Calcula el tamaño actual de un buffer circular (ring buffer)
 *
 * Esta función determina la cantidad de datos presentes en el buffer circular
 * para la recepción (Rx) o transmisión (Tx), según el tipo especificado.
 *
 * Calcula la diferencia entre los índices de escritura y lectura, ajustando
 * por el tamaño máximo del buffer en caso de desbordamiento circular.
 *
 * @param [in] estructura: Puntero a la estructura _sRingBuffer que contiene los índices del buffer
 * @param [in] tipo: Tipo de buffer a calcular (0 = Buffer Rx, 1 = Buffer Tx)
 *
 * @return uint16_t: Tamaño actual del buffer especificado (número de elementos ocupados)
 *
 * @note Los buffers deben ser inicializados correctamente antes de usar esta función
 */
uint16_t ringBufferSize(_sRingBuffer *estructura, uint8_t tipo);



/**
 * @brief Maneja los datos recibidos por USB
 *
 * Esta función se encarga de copiar los datos recibidos desde el USB al buffer circular
 * de recepción.
 *
 * Actualiza el índice de escritura del buffer (Rxiw) con la longitud de
 * datos recibidos y registra el tiempo de la última recepción USB para control de tiempo.
 *
 * @param [in] data: Puntero a los datos recibidos que se copiarán al buffer
 * @param [in] length: Longitud de los datos recibidos (en bytes)
 *
 * @note Utiliza la variable global bufferUSB para gestionar el buffer circular
 * @note Actualiza elapsed.usbReceive con el tiempo de la última recepción
 */
void onUSBRx(uint8_t *data, uint32_t length);

/**
 * @brief Decodifica y procesa los datos recibidos en el buffer circular
 *
 * Esta función interpreta el comando recibido (identificado por el campo 'id' de la estructura)
 * y procesa el payload correspondiente según el tipo de comando.
 *
 * Los datos se extraen del payload y se asignan a las variables globales correspondientes.
 *
 * @param [in] estructura: Puntero a la estructura _sRingBuffer que contiene el comando y payload a decodificar
 *
 * @note Utiliza variables globales: flags, estadoComandos, currentConfig, currentESP01states, CMD_Received
 * @note Actualiza múltiples variables globales: automatic, tracker, mode, optionState, Gz_offset, angle, etc.
 * @note El campo 'id' de la estructura se resetea a 0x00 después del procesamiento
 */
void decodeData(_sRingBuffer *estructura);



/**
 * @brief Establece el estado del pin Chip Enable de la ESP01
 *
 * Esta función controla el pin de Chip Enable (CH_PD) del módulo ESP01 mediante
 * la escritura de un valor en el pin GPIO correspondiente.
 *
 * Permite habilitar o deshabilitar el módulo según el valor proporcionado.
 *
 * @param [in] value: Valor a escribir (0 para deshabilitar, 1 para habilitar)
 */
void ESP01_CH_PD_Set(uint8_t value);

/**
 * @brief Recibe un byte del UART de la ESP01 y lo almacena en el buffer circular
 *
 * Esta función se llama cuando se recibe un byte desde el UART conectado a la ESP01.
 *
 * El byte se almacena en el buffer circular de recepción (bufferRx) y se incrementa el índice de escritura (Rxiw).
 *
 * Si el índice de escritura alcanza el tamaño máximo del buffer, se vuelve a cero (comportamiento circular).
 *
 * Además, actualiza la variable elapsed.esp01Receive con el tiempo actual para llevar el control de la última recepción.
 *
 * @param [in] value: Byte recibido desde el UART de la ESP01
 *
 * @note Utiliza variables globales: bufferESP01, elapsed
 */
void ESP01_UART_Receive(uint8_t value);

/**
 * @brief Envía un byte a través del UART a la ESP01
 *
 * Esta función transmite un solo byte al módulo ESP01 a través del puerto UART
 * utilizando la función HAL_UART_Transmit.
 *
 * La transmisión se realiza con un tiempo de espera predeterminado de 10 ms.
 *
 * @param [in] value: Byte a transmitir a la ESP01
 *
 * @return int: Siempre retorna 1 indicando que la operación se completó
 *
 * @note Utiliza la instancia huart1 para la transmisión UART
 */
int ESP01_UART_Send(uint8_t value);

/**
 * @brief Cambia el estado de la ESP01 y actualiza la configuración local
 *
 * Esta función actualiza el estado global de la ESP01 según el estado recibido.
 *
 * En casos específicos, como cuando se conecta UDP/TCP, también obtiene la dirección IP local
 * y la copia a la configuración actual.
 *
 * @param [in] statusESP01: Nuevo estado de la ESP01 que se va a procesar
 *
 * @note Utiliza variables globales: stateESP01, ip, currentConfig
 */
void ESP01_ChangeState(_eESP01STATUS statusESP01);



/**
 * @brief Inicializa el sensor MPU6050
 *
 * Esta función realiza la inicialización y configuración del sensor MPU6050.
 *
 * Primero verifica la identidad del sensor leyendo el registro WHO_AM_I (0x68).
 *
 * Luego realiza un reset del sensor, configura la tasa de muestreo, el filtro digital
 * y los rangos de medición para el acelerómetro y giroscopio.
 *
 * Utiliza la configuración para el filtro digital (DLPF) con ancho de banda de 94Hz para acelerómetro
 * y 98Hz para giroscopio.
 *
 * @note Utiliza la interfaz I2C (hi2c2) para comunicarse con el sensor
 * @note Incluye un delay de 100ms después del reset para asegurar la inicialización
 */
void MPU6050_Init();

/**
 * @brief Lee los datos del acelerómetro del MPU6050 usando DMA
 *
 * Esta función inicia la lectura de los registros del acelerómetro (X, Y, Z) del sensor MPU6050
 * utilizando transmisión por DMA.
 *
 * Los datos se almacenan en el buffer Rec_Data_A y se activa
 * un flag para indicar que se ha solicitado una lectura del acelerómetro.
 *
 * La lectura se realiza desde el registro ACCEL_XOUT_H_REG y se obtienen 6 bytes (2 por eje).
 *
 * @note Utiliza la interfaz I2C (hi2c2) en modo DMA para la lectura
 * @note Los datos quedan disponibles en el buffer global Rec_Data_A después de la lectura
 * @note Actualiza el flag flags.bit2.Acc_Ask_MPU para indicar que se solicitó la lectura
 */
void MPU6050_Read_Accel ();

/**
 * @brief Lee los datos del giroscopio del MPU6050 usando DMA
 *
 * Esta función inicia la lectura de los registros del giroscopio (X, Y, Z) del sensor MPU6050
 * utilizando transmisión por DMA.
 *
 * Los datos se almacenan en el buffer Rec_Data_G y se activa
 * un flag para indicar que se ha solicitado una lectura del giroscopio.
 *
 * La lectura se realiza desde el registro GYRO_XOUT_H_REG y se obtienen 6 bytes (2 por eje).
 *
 * @note Utiliza la interfaz I2C (hi2c2) en modo DMA para la lectura
 * @note Los datos quedan disponibles en el buffer global Rec_Data_G después de la lectura
 * @note Actualiza el flag flags.bit2.Gyr_Ask_MPU para indicar que se solicitó la lectura
 */
void MPU6050_Read_Gyro();

/**
 * @brief Lee los datos de temperatura del MPU6050 usando DMA
 *
 * Esta función inicia la lectura del registro de temperatura del sensor MPU6050
 * utilizando transmisión por DMA.
 *
 * Los datos se almacenan en el buffer Rec_Data_T y se activa un flag para indicar
 * que se ha solicitado una lectura de temperatura.
 *
 * La lectura se realiza desde el registro TEMP_OUT_H_REG y se obtienen 2 bytes.
 *
 * @note Utiliza la interfaz I2C (hi2c2) en modo DMA para la lectura
 * @note Los datos quedan disponibles en el buffer global Rec_Data_T después de la lectura
 * @note Actualiza el flag flags.bit2.Temp_Ask_MPU para indicar que se solicitó la lectura
 */
void MPU6050_Read_Temp();

/**
 * @brief Procesa los datos del giroscopio MPU6050 usando un filtro de media móvil
 *
 * Esta función calcula el ángulo de rotación integrando las lecturas del giroscopio (eje Z)
 * después de aplicar un offset y un umbral que varía según el modo de operación.
 *
 * El ángulo se actualiza sumando la velocidad angular (Gz_dps) dividida por 20 (factor de escala),
 * y se mantiene dentro del rango de 0 a 360 grados mediante ajuste circular.
 *
 * @note Utiliza variables globales: mode, G_Z, Gz_offset, Gz_dps, angle
 * @note El umbral de sensibilidad es mayor en modo automatic (0.15) que en modo tracker (0.08)
 * @note La función no tiene parámetros de entrada/salida, opera directamente sobre variables globales
 */
void MPU6050_SMA_G();

/**
 * @brief Calcula el offset del giroscopio promediando 20 muestras
 *
 * Esta función acumula lecturas del giroscopio en el eje Z (G_Z) y después de 20 muestras
 * calcula el valor promedio que se utiliza como offset para calibrar el giroscopio.
 *
 * Una vez completada la calibración, desactiva el flag de primera offset (Gyr_First_Offset).
 *
 * @note Utiliza variables globales: i_G, G_Z, Gz_offset, flags
 * @note La función debe llamarse repetidamente hasta que se completen las 20 muestras
 * @note El offset se calcula solo cuando se han recolectado 20 muestras
 */
void MPU6050_Gyro_Setup();



/**
 * @brief Gestiona la visualización en la pantalla OLED SSD1306
 *
 * Esta función controla la interfaz gráfica del sistema, mostrando diferentes pantallas
 * según el estado actual (screenState).
 *
 * Cada pantalla presenta información específica del sistema como menú de opciones, parámetros de control,
 * estado de conexión, datos de sensores y configuración de motores.
 *
 * Utiliza diferentes fuentes y elementos gráficos para presentar la información de manera clara y organizada.
 *
 * @note Utiliza múltiples variables globales: screenState, optionState, mode, flags, etc.
 * @note Actualiza el flag flags.bit1.screenState al finalizar la renderización
 */
void SSD1306_Display();



/**
 * @brief Gestiona el flujo de control automático del velocista
 *
 * Esta función ajusta las velocidades de los motores según el estado de seguimiento de línea
 * en modo automatic.
 *
 * En estado LOST, gira en la dirección de la última línea detectada.
 *
 * En estado FOLLOWING, utiliza el control PID para mantener el seguimiento, con una zona muerta donde acelera
 * después de un tiempo estable.
 *
 * @note Utiliza variables globales: automatic.follow_state, last_line, error_t, accelCounter, etc.
 * @note Actualiza variables globales: left_Speed, right_Speed, turn_L, turn_R, accelCounter
 */
void auto_flow();

/**
 * @brief Gestiona el flujo de control del modo tracker del velocista
 *
 * Esta función ajusta las velocidades de los motores según el estado de seguimiento de línea
 * en modo tracker.
 *
 * En estado LOST, gira en la dirección de la última línea detectada.
 *
 * En estado FOLLOWING, utiliza el control PID para mantener el seguimiento con una zona muerta
 * donde ambos motores mantienen la velocidad media.
 *
 * @note Utiliza variables globales: tracker.follow_state, last_line, error_t, etc.
 * @note Actualiza variables globales: left_Speed, right_Speed, turn_L, turn_R
 */
void tracker_flow();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void SMA_Init(_SMA_Filter *filter){
    for(int i = 0; i < NUM_SENSORS; i++) {
        filter->sum[i] = 0;
        for(int j = 0; j < N; j++) {
        	filter->buffer[i][j] = 0;
        }
    }
    filter->idx = 0;
    filter->conteo = 0;
}


void SMA_Update(_SMA_Filter *filter){
	uint16_t valor_antiguo;

	if (filter->conteo < N) {
        for(int i = 0; i < NUM_SENSORS; i++) {
        	filter->sum[i] += adc_raw[i];
            filter->buffer[i][filter->idx] = adc_raw[i];
            adc_clean[i] = filter->sum[i] / (filter->conteo + 1);
        }
        filter->idx++;
        if (filter->idx >= N){
            filter->idx = 0;
        }
		filter->conteo++;
	} else {
		media = 0;
        for(int i = 0; i < NUM_SENSORS; i++) {
            valor_antiguo = filter->buffer[i][filter->idx];
            filter->sum[i] -= valor_antiguo;
            filter->buffer[i][filter->idx] = adc_raw[i];
            filter->sum[i] += adc_raw[i];
            adc_clean[i] = filter->sum[i] / N;
            media = adc_clean[i];
        }
        media = media/NUM_SENSORS;
        filter->idx++;
        if (filter->idx >= N){
            filter->idx = 0;
        }
	}
}


void position_IC(){
	uint8_t i_min = 0;

	int8_t x0, x1, x2;
	uint16_t y0, y1, y2;

	uint16_t center = ((NUM_SENSORS - 1)*100) / 2;  // 700/2 = 350 para 8 sensores
	int32_t a,b;
	int32_t error;
	float pos;


	for (uint8_t i = 0; i < NUM_SENSORS; i++) {
		if (adc_clean[i] < adc_clean[i_min]) {
			i_min = i;
		}
	}

	for (int i = 0; i < NUM_SENSORS; ++i) {
		if(adc_clean[i] > 2700){
			adc_bit[i] = 0;
		} else {
			adc_bit[i] = 1;
			last_line = i;
		}
	}

	if (i_min <= 0) {
		x0 = -1;  // Sensor fantasma izquierdo
		x1 = 0;
		x2 = 1;

		y0 = 4000;  // Valor alto para sensor fantasma (sin línea)
		y1 = adc_clean[0];
		y2 = adc_clean[1];

	} else if (i_min == (NUM_SENSORS - 1)) {
		x0 = NUM_SENSORS - 2;
		x1 = NUM_SENSORS - 1;
		x2 = NUM_SENSORS;  // Sensor fantasma derecho

		y0 = adc_clean[NUM_SENSORS - 2];
		y1 = adc_clean[NUM_SENSORS - 1];
		y2 = 4000;  // Valor alto para sensor fantasma (sin línea)

	} else {
		x0 = i_min - 1;
		x1 = i_min;
		x2 = i_min + 1;

		y0 = adc_clean[i_min - 1];
		y1 = adc_clean[i_min];
		y2 = adc_clean[i_min + 1];
	}

	//y = ax² + bx + c
	int16_t denom = ((x0 - x1) * (x0 - x2) * (x1 - x2));

	if (denom != 0) {
	    a = (x2*(y1 - y0) + x1*(y0 - y2) + x0*(y2 - y1))/denom;
	    b = (x2*x2*(y0 - y1) + x1*x1*(y2 - y0) + x0*x0*(y1 - y2))/denom;

		if (a > 10 || a < -10) {
			pos = ((-b) / (2.0 * a))*100;
		} else {
			pos = i_min*100;
		}
	} else {
		pos = i_min*100;
	}
	error = ((uint16_t)pos - center);  // Rango aproximado: -350 a +350

	// Limites del error
	if (error > 350)
		error = 350;

	if (error < -350)
		error = -350;

	error_t = error;

	if(mode == 1){
		// Left
	    error_p = error_t;
	    P = (error_p*automatic.kp_L)/100;// divide 100 para mejorar la resolución del control de kp

		error_i += error_t;
		I = error_i*automatic.ki_L;

	    error_d = (error_t - last_d_error);
	    last_d_error = error_t;
		D = error_d*automatic.kd_L;

		PID_L = P + I + D;

		// Right
	    error_p = error_t;
	    P = (error_p*automatic.kp_R)/100;// divide 100 para mejorar la resolución del control de kp

		error_i += error_t;
		I = error_i*automatic.ki_R;

	    error_d = (error_t - last_d_error);
	    last_d_error = error_t;
		D = error_d*automatic.kd_R;

		PID_R = P + I + D;
	}
	if(mode == 2){
		// Left
	    error_p = error_t;
	    P = (error_p*tracker.kp_L)/100;// divide 100 para mejorar la resolución del control de kp

		error_i += error_t;
		I = error_i*tracker.ki_L;

	    error_d = (error_t - last_d_error);
	    last_d_error = error_t;
		D = error_d*tracker.kd_L;

		PID_L = P + I + D;

		// Right
	    error_p = error_t;
	    P = (error_p*tracker.kp_R)/100;// divide 100 para mejorar la resolución del control de kp

		error_i += error_t;
		I = error_i*tracker.ki_R;

	    error_d = (error_t - last_d_error);
	    last_d_error = error_t;
		D = error_d*tracker.kd_R;

		PID_R = P + I + D;
	}
}


void follow_State(){
	line = 0;

	for(int i = 0; i < NUM_SENSORS; i++){
		line += adc_bit[i];
	}

	if(line == 0){		// Entrar en modo LOST
		flags.bit2.on_line = FALSE;
		tracker.follow_state = LOST;
		automatic.follow_state = LOST;
	}else{				// Entrar en modo FOLLOWING
		flags.bit2.on_line = TRUE;

		if(mode == 1){
			if(abs(error_t) <= automatic.returnZone){
				automatic.follow_state = FOLLOWING;
			}
		}

		if(mode == 2){
			if(abs(error_t) <= tracker.returnZone){
				tracker.follow_state = FOLLOWING;
			}
		}
	}
}





void BT_Debounce(Button *btn){
  uint8_t current_state = HAL_GPIO_ReadPin(btn->GPIO_Port, btn->GPIO_Pin);

  if (current_state != btn->last_state && (HAL_GetTick() - btn->last_time) > DEBOUNCE_TIME) {
    btn->debounced_state = current_state;
    btn->last_state = current_state;
    btn->last_time = HAL_GetTick();
  }

  switch (btn->state) {
    case BTN_STATE_UP:
      if (btn->debounced_state == 0) {
        btn->state = BTN_STATE_FALLING;
      }
      break;

    case BTN_STATE_DOWN:
	  if (btn->debounced_state == 1) {
		btn->state = BTN_STATE_RISING;
	  } else {
		  if(btn->hold_triggered == FALSE){
			  if (!btn->hold_detected && (HAL_GetTick() - btn->hold_time) >= HOLD_TIME) {
				btn->hold_detected = TRUE;
			  }
		  }
	  }
      break;

    case BTN_STATE_RISING:
      btn->state = BTN_STATE_UP;
      break;

    case BTN_STATE_FALLING:
      btn->state = BTN_STATE_DOWN;
      btn->hold_time = HAL_GetTick();
      btn->hold_detected = FALSE;
      btn->hold_triggered = FALSE;
      btn->hold_release = FALSE;
      break;
  }
}





uint16_t ringBufferSize(_sRingBuffer *estructura, uint8_t tipo){
	//tipo 0 = Buffer Rx
	//tipo 1 = Buffer Tx
    int16_t diff;

    if(tipo == 0){
        diff = estructura->Rxiw - estructura->Rxir;
        if (diff < 0)
            diff += MAX_BUFFER_SIZE;
        return (uint16_t)diff;
    } else {
        diff = estructura->Txiw - estructura->Txir;
        if (diff < 0)
            diff += MAX_BUFFER_SIZE;
        return (uint16_t)diff;
    }
}

void onUSBRx(uint8_t *data, uint32_t length) {
	memcpy(&bufferUSB.bufferRx[bufferUSB.Rxiw],data,length);
	bufferUSB.Rxiw += length;

	elapsed.usbReceive = elapsed.oneSec;
}


void decodeData(_sRingBuffer *estructura){
	uint8_t i = 0;
	uint8_t config, currentByte;
	uint8_t j;

    switch (estructura->id) {
		case ACK:
			flags.bit1.sendReadyUSB = TRUE;
			estructura->payloadTxLength = 0;
			bufferUSB.payloadTxLength = 0;
			estadoComandos = ACK;
			break;
		case ALIVE:
			flags.bit1.sendReadyUSB = TRUE;
			estructura->payloadTxLength = 0;
			bufferUSB.payloadTxLength = 0;
			estadoComandos = ALIVE;
			break;
		case ESP01_CONFIG:
			config = 0;
			memset(currentConfig.ssid, 0, sizeof(currentConfig.ssid));
			memset(currentConfig.password, 0, sizeof(currentConfig.password));
			memset(currentConfig.remoteIP, 0, sizeof(currentConfig.remoteIP));
			j = 0;

			while(i <= estructura->payloadRxLength){
				currentByte = estructura->payloadRx[i++];
				switch (config) {
					case 0:
						if(currentByte == ':')
							config++;
						break;
					case 1:
						if(currentByte == ':'){
							config++;
							j = 0;
						}else{
							currentConfig.ssid[j++] = currentByte;
						}
						break;
					case 2:
						if(currentByte == ':'){
							config++;
							j = 0;
						}else{
							currentConfig.password[j++] = currentByte;
						}
						break;
					case 3:
						if(currentByte == ':'){
							config++;
							j = 0;
						}else{
							currentConfig.remoteIP[j++] = currentByte;
						}
						break;
					case 4:
						if(currentByte == ':'){
							break;
						}
						break;
					default:
						break;
				}
			}
			currentESP01states = ESP01_RECONNECT;
			break;
		case ESP01_QT_READY:
			currentESP01states = ESP01_MPU;
			CMD_Received = ESP01_QT_READY;
			break;
		case ESP01_QT_OUT:
			currentESP01states = ESP01_FIRST_READY;
			CMD_Received = ESP01_QT_OUT;
			break;
		case CMD_MPU:
			flags.bit2.Gyr_First_Offset = TRUE;
			Gz_offset = 0;
			angle = 0;
			CMD_Received = CMD_MPU;
			break;
		case CMD_CONFIG_A:
			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			automatic.mid_Speed = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.kp_L = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.ki_L = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.kd_L = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.kp_R = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.ki_R = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.kd_R = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			automatic.send_time = myWord.as_uint32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			automatic.deadZone = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			automatic.returnZone = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			automatic.accZone = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			automatic.accEntry = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			automatic.acc_Speed = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			automatic.back_Speed = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			automatic.lost_Speed = myWord.uint16_parts.low_uint16;

			CMD_Received = CMD_CONFIG_A;
			break;
		case CMD_CONFIG_T:
			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			tracker.mid_Speed = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.kp_L = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.ki_L = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.kd_L = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.kp_R = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.ki_R = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.kd_R = myWord.as_int32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			myWord.bytes.byte2 = estructura->payloadRx[i++];
			myWord.bytes.byte3 = estructura->payloadRx[i++];
			tracker.send_time = myWord.as_uint32;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			tracker.deadZone = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			myWord.bytes.byte1 = estructura->payloadRx[i++];
			tracker.returnZone = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			tracker.accZone = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			tracker.accEntry = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			tracker.acc_Speed = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			tracker.back_Speed = myWord.uint16_parts.low_uint16;

			myWord.bytes.byte0 = estructura->payloadRx[i++];
			tracker.lost_Speed = myWord.uint16_parts.low_uint16;

			CMD_Received = CMD_CONFIG_T;
			break;
		case CMD_WORK_MODE:
			myWord.bytes.byte0 = estructura->payloadRx[i++];
			mode = myWord.uint16_parts.low_uint16;
			optionState = mode;
			CMD_Received = CMD_WORK_MODE;
			break;
		default:
			break;
    }
    estructura->id = 0x00;
}





void ESP01_CH_PD_Set(uint8_t value){
	HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, value);
}


void ESP01_UART_Receive(uint8_t value){
	bufferESP01.bufferRx[bufferESP01.Rxiw++] = value;
	if (bufferESP01.Rxiw >= MAX_BUFFER_SIZE) {
		bufferESP01.Rxiw = 0;
	}

	elapsed.esp01Receive = elapsed.oneSec;
}


int ESP01_UART_Send(uint8_t value){
	HAL_UART_Transmit(&huart1, &value, 1, 10);
	return  1;
}


void ESP01_ChangeState(_eESP01STATUS statusESP01){
	switch (statusESP01) {
		case ESP01_READY:							//	1
			stateESP01 = 1;
			break;
		case ESP01_WIFI_CONNECTED:					//	2
			stateESP01 = 2;
			break;
		case ESP01_WIFI_NEW_IP:						//	3
			stateESP01 = 3;
			break;
		case ESP01_UDPTCP_CONNECTED:				//	4
			stateESP01 = 4;
			ip = ESP01_GetLocalIP();
			strcpy(currentConfig.localIP,ip);
			currentConfig.localIP[15] = '\0';
			break;
		case ESP01_WIFI_NOT_SETED:
			break;
		case ESP01_WIFI_CONNECTING_WIFI:
			break;
		case ESP01_UDPTCP_DISCONNECTED:
			break;
		case ESP01_UDPTCP_CONNECTING:
			break;
		case ESP01_SEND_BUSY:
			break;
		case ESP01_SEND_READY:
			break;
		case ESP01_SEND_ERROR:
			break;
		default:
			break;
	}
}





void MPU6050_Init(){
	uint8_t check,data;
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1 , 1000);

	if (check == 0x68)
	{
        data = 0x80;  // Bit de reset
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);
        HAL_Delay(100);

        data = 0x00;
        HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 1000);

		data = 0x07;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 1000);

/*
DLPF_CFG	Acelerómetro (Hz)	Delay Acel. (ms)	Giroscopio (Hz)	Delay Giro (ms)	Fs (kHz)
	0			260 Hz				0 ms				256 Hz			0.98 ms		8 kHz
	1			184 Hz				2.0 ms				188 Hz			1.9 ms		1 kHz
==>	2			94 Hz				3.0 ms				98 Hz			2.8 ms		1 kHz
	3			44 Hz				4.9 ms				42 Hz			4.8 ms		1 kHz
	4			21 Hz				8.5 ms				20 Hz			8.3 ms		1 kHz
	5			10 Hz				13.8 ms				10 Hz			13.4 ms		1 kHz
	6			5 Hz				19.0 ms				5 Hz			18.6 ms		1 kHz
*/
		data = 0x02;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, FILTER, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 1000);

		data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 1000);
	}
}


void MPU6050_Read_Accel (){
	HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_A, 6);
	flags.bit2.Acc_Ask_MPU = TRUE;
}


void MPU6050_Read_Gyro(){
    HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_G, 6);
    flags.bit2.Gyr_Ask_MPU = TRUE;
}


void MPU6050_Read_Temp(){
	HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data_T, 2);
    flags.bit2.Temp_Ask_MPU = TRUE;
}


void MPU6050_SMA_G(){

	if(mode == 1){
		if(abs(G_Z - Gz_offset) >= 0.15){
			Gz_dps = (G_Z - Gz_offset);
			angle += Gz_dps/20;

			if (angle >= 360)
				angle -= 360;
			if (angle < 0)
				angle += 360;
		}
	}
	if(mode == 2){
		if(abs(G_Z - Gz_offset) >= 0.08){
			Gz_dps = (G_Z - Gz_offset);
			angle += Gz_dps/20;

			if (angle >= 360)
				angle -= 360;
			if (angle < 0)
				angle += 360;
		}
	}
}


void MPU6050_Gyro_Setup(){
	i_G++;
	Gz_offset += G_Z;
	if (i_G >= 20) {
		Gz_offset = Gz_offset / i_G;
		i_G = 0;
		flags.bit2.Gyr_First_Offset = FALSE;
	}
}





void SSD1306_Display(){

	switch (screenState) {

		case 0:
			SSD1306_Fill(SSD1306_COLOR_BLACK);

			SSD1306_GotoXY(1, 0);
			SSD1306_Puts("MENU", &Font_11x18, SSD1306_COLOR_WHITE);

			char *modos[] = {
			    "MANUAL",
			    "AUTO",
			    "TRACK"
			};

			for (uint8_t i = 0; i < 3; i++) {
			    uint8_t y = 26 + i * 12; // Posiciones seguras: 26, 38, 50
			    SSD1306_GotoXY(1, y);
			    if (i == optionState) {
			        SSD1306_Puts("> ", &Font_7x10, SSD1306_COLOR_WHITE);
			    } else {
			        SSD1306_Puts("  ", &Font_7x10, SSD1306_COLOR_WHITE);
			    }
			    SSD1306_Puts(modos[i], &Font_7x10, SSD1306_COLOR_WHITE);
			}

			switch (mode) {
			    case 0:
			        SSD1306_DrawLine(16, 36, 58, 36, SSD1306_COLOR_WHITE);
			        break;
			    case 1:
			    	SSD1306_DrawLine(16, 47, 72, 47, SSD1306_COLOR_WHITE);
			        break;
			    case 2:
			    	SSD1306_DrawLine(16, 59, 72, 59, SSD1306_COLOR_WHITE);
			        break;
			}
			break;
		case 1:
			SSD1306_Fill(SSD1306_COLOR_BLACK);

			SSD1306_GotoXY(1,0);
			SSD1306_Puts("GENERAL", &Font_11x18, SSD1306_COLOR_WHITE);

			switch(optionState){
				case 0:
					if(mode == 0){
						SSD1306_GotoXY(1, 22);
						sprintf(str, "Mode:%u", 0);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					if(mode == 1){
						SSD1306_GotoXY(1, 22);
						sprintf(str, "Mid:%u", automatic.mid_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 42);
						sprintf(str, "fol:%u", automatic.follow_state);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					if(mode == 2){
						SSD1306_GotoXY(1, 22);
						sprintf(str, "Mid:%u", tracker.mid_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 42);
						sprintf(str, "fol:%u", tracker.follow_state);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					SSD1306_GotoXY(1, 32);
					sprintf(str, "Err:%ld", error_t);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(1, 52);
					sprintf(str, "Line:%u", flags.bit2.on_line);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(64, 22);
					sprintf(str, "L.line:%u", last_line);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(64, 32);
					sprintf(str, "R:%u", turn_R);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(64, 42);
					sprintf(str, "L:%u", turn_L);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					break;
				case 1:
					SSD1306_GotoXY(1,22);
					SSD1306_Puts("L", &Font_7x10, SSD1306_COLOR_WHITE);

					if(mode == 1){
					    SSD1306_GotoXY(1, 32);
					    sprintf(str, "kp:%ld", automatic.kp_L);
					    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					    SSD1306_GotoXY(1, 42);
					    sprintf(str, "ki:%ld", automatic.ki_L);
					    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					    SSD1306_GotoXY(1, 52);
					    sprintf(str, "kd:%ld", automatic.kd_L);
					    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					if(mode == 2){
					    SSD1306_GotoXY(1, 32);
					    sprintf(str, "kp:%ld", tracker.kp_L);
					    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					    SSD1306_GotoXY(1, 42);
					    sprintf(str, "ki:%ld", tracker.ki_L);
					    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					    SSD1306_GotoXY(1, 52);
					    sprintf(str, "kd:%ld", tracker.kd_L);
					    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

				    SSD1306_GotoXY(48,22);
					SSD1306_Puts("R", &Font_7x10, SSD1306_COLOR_WHITE);

					if(mode == 1){
						SSD1306_GotoXY(48, 32);
						sprintf(str, "kp:%ld", automatic.kp_R);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(48, 42);
						sprintf(str, "ki:%ld", automatic.ki_R);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(48, 52);
						sprintf(str, "kd:%ld", automatic.kd_R);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					if(mode == 2){
						SSD1306_GotoXY(48, 32);
						sprintf(str, "kp:%ld", tracker.kp_R);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(48, 42);
						sprintf(str, "ki:%ld", tracker.ki_R);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(48, 52);
						sprintf(str, "kd:%ld", tracker.kd_R);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

				    SSD1306_GotoXY(94,22);
					SSD1306_Puts("PID", &Font_7x10, SSD1306_COLOR_WHITE);

				    SSD1306_GotoXY(94, 32);
				    sprintf(str, "P:%ld", P);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

				    SSD1306_GotoXY(94, 42);
				    sprintf(str, "I:%ld", I);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

				    SSD1306_GotoXY(94, 52);
				    sprintf(str, "D:%ld", D);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					break;
			}

			break;
		case 2:
			SSD1306_Fill(SSD1306_COLOR_BLACK);

			SSD1306_GotoXY(1,0);
			SSD1306_Puts("CONNECT", &Font_11x18, SSD1306_COLOR_WHITE);

			char *nets[] = {
			    "Lab",
			    "Dpto",
			};

			for (uint8_t i = 0; i < 2; i++) {
			    uint8_t y = 22 + i * 12; // Posiciones seguras: 26, 38, 50
			    SSD1306_GotoXY(1, y);
			    if (i == optionState) {
			        SSD1306_Puts("> ", &Font_7x10, SSD1306_COLOR_WHITE);
			    } else {
			        SSD1306_Puts("  ", &Font_7x10, SSD1306_COLOR_WHITE);
			    }
			    SSD1306_Puts(nets[i], &Font_7x10, SSD1306_COLOR_WHITE);
			}

			switch (network) {
			    case 0:
			        SSD1306_DrawLine(16, 31, 40, 31, SSD1306_COLOR_WHITE);
			        break;
			    case 1:
			    	SSD1306_DrawLine(16, 43, 40, 43, SSD1306_COLOR_WHITE);
			        break;
			}

			SSD1306_GotoXY(48, 22);
			switch (stateESP01) {
				case 0:
					SSD1306_Puts("[WiFi:---]", &Font_7x10, SSD1306_COLOR_WHITE);
					break;
				case 1:
					SSD1306_Puts("[SSID OK]", &Font_7x10, SSD1306_COLOR_WHITE);
					break;
				case 2:
					SSD1306_Puts("[SSID OK]", &Font_7x10, SSD1306_COLOR_WHITE);
					break;
				case 3:
					SSD1306_Puts("[PASS OK]", &Font_7x10, SSD1306_COLOR_WHITE);
					break;
				case 4:
					SSD1306_Puts("[IP OK]", &Font_7x10, SSD1306_COLOR_WHITE);
					stateESP01 = 5;
					break;
				case 5:
					SSD1306_Puts("[WiFi:OK]", &Font_7x10, SSD1306_COLOR_WHITE);
					break;
				default:
					break;
			}

			if(mode == 1){
				SSD1306_GotoXY(48, 42);
				sprintf(str, "Send:%lu", automatic.send_time);
				SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
			}

			if(mode == 2){
				SSD1306_GotoXY(48, 42);
				sprintf(str, "Send:%lu", tracker.send_time);
				SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
			}

			SSD1306_GotoXY(1, 52);
			SSD1306_Puts("rIP:", &Font_7x10, SSD1306_COLOR_WHITE);
			SSD1306_Puts(currentConfig.remoteIP, &Font_7x10, SSD1306_COLOR_WHITE);

			break;
		case 3:
			SSD1306_Fill(SSD1306_COLOR_BLACK);

			SSD1306_GotoXY(1,0);
			SSD1306_Puts("MOTORS", &Font_11x18, SSD1306_COLOR_WHITE);

			switch (optionState) {
				case 0:
					if(mode == 1){
						SSD1306_GotoXY(1, 22);
						sprintf(str, "mid:%u", automatic.mid_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 32);
						sprintf(str, "Asp:%u", automatic.acc_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 42);
						sprintf(str, "Bsp:%u", automatic.back_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 52);
						sprintf(str, "Lsp:%u", automatic.lost_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(64, 22);
						sprintf(str, "Aen:%u", automatic.accEntry);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(64, 32);
						sprintf(str, "Azo:%u", automatic.accZone);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					}
					if(mode == 2){
						SSD1306_GotoXY(1, 22);
						sprintf(str, "mid:%u", tracker.mid_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 32);
						sprintf(str, "Asp:%u", tracker.acc_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 42);
						sprintf(str, "Bsp:%u", tracker.back_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(1, 52);
						sprintf(str, "Lsp:%u", tracker.lost_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(64, 22);
						sprintf(str, "Aen:%u", tracker.accEntry);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

						SSD1306_GotoXY(64, 32);
						sprintf(str, "Azo:%u", tracker.accZone);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					break;
				case 1:

					SSD1306_GotoXY(1, 22);
					sprintf(str, "izq:%u", left_Speed);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(70, 22);
					sprintf(str, "der:%u", right_Speed);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					if(mode == 1){
						SSD1306_GotoXY(48, 32);
						sprintf(str, "mid:%u", automatic.mid_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					if(mode == 2){
						SSD1306_GotoXY(48, 32);
						sprintf(str, "mid:%u", tracker.mid_Speed);
						SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
					}

					break;
				default:
					break;
			}

			break;
		case 4:
			SSD1306_Fill(SSD1306_COLOR_BLACK);

			SSD1306_GotoXY(1,0);
			SSD1306_Puts("LINE", &Font_11x18, SSD1306_COLOR_WHITE);

			switch (optionState) {
				case 0:
					uint8_t start_x = 1;

					for(uint8_t i = 0; i < NUM_SENSORS; i++) {
						uint8_t x = GRAPH_LEFT_X + start_x + (i * (BAR_WIDTH + SPACING));

						uint8_t height = (adc_clean[i] * GRAPH_HEIGHT + 2047) / 4095;
						if (height == 0 && adc_clean[i] > 0) height = 1;

						uint8_t y = GRAPH_BOTTOM_Y - height;

						SSD1306_DrawFilledRectangle(x, y, BAR_WIDTH, height, SSD1306_COLOR_WHITE);
					}

					// Configuración de la linea
					const uint8_t bar_start_x = 10;
					const uint8_t bar_end_x = 118;
					const uint8_t bar_width = bar_end_x - bar_start_x;
					const uint8_t center_x = bar_start_x + bar_width / 2;

					uint8_t indicator_x = center_x + ((error_t/10) * bar_width / 70);

					uint8_t tri_size = 4;

					SSD1306_DrawLine(bar_start_x - 1, 56, bar_width + 2, 56, SSD1306_COLOR_WHITE);

				    SSD1306_DrawTriangle(
				    		indicator_x, 54 - tri_size,
				    		indicator_x - tri_size/2, 54,
							indicator_x + tri_size/2, 54,
							SSD1306_COLOR_WHITE
					);

					break;
				case 1:
					SSD1306_GotoXY(64, 42);
					sprintf(str, "Line:%u", line);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(1, 42);
					sprintf(str, "L.Line%u", last_line);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(1, 52);
					sprintf(str, "Err:%ld", error_t);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					break;
				default:
					break;
			}

			break;
		case 5:
			SSD1306_Fill(SSD1306_COLOR_BLACK);

			SSD1306_GotoXY(1, 0);
			SSD1306_Puts("MPU6050", &Font_11x18, SSD1306_COLOR_WHITE);

		    SSD1306_DrawCircle(CENTER_X, CENTER_Y, RADIUS, SSD1306_COLOR_WHITE);

		    int8_t x_offset = (int8_t)(((A_X/100.0) / ACC_RANGE) * RADIUS);
		    int8_t y_offset = (int8_t)(((A_Y/100.0) / ACC_RANGE) * RADIUS);

		    float dist = sqrtf(x_offset * x_offset + y_offset * y_offset);
		    if (dist > RADIUS - 2) {
		        float scale = (RADIUS - 2) / dist;
		        x_offset *= scale;
		        y_offset *= scale;
		    }

		    SSD1306_DrawFilledCircle(CENTER_X + x_offset, CENTER_Y + y_offset, 2, SSD1306_COLOR_WHITE);

		    switch (optionState) {
				case 0:
				    SSD1306_GotoXY(1, 28);
				    sprintf(str, "X:%d", A_X);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
				    SSD1306_GotoXY(1, 40);
				    sprintf(str, "Y:%d", A_Y);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
				    SSD1306_GotoXY(1, 52);
				    sprintf(str, "Z:%d", A_Z);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					break;
				case 1:
				    SSD1306_GotoXY(1, 28);
				    sprintf(str, "Gx:%d", G_X);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
				    SSD1306_GotoXY(1, 40);
				    sprintf(str, "Gy:%d",G_Y);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);
				    SSD1306_GotoXY(1, 52);
				    sprintf(str, "Gz:%d",G_Z);
				    SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					break;
				case 2:
					SSD1306_GotoXY(1, 22);
					sprintf(str, "ANG:%d",angle);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(1, 42);
					sprintf(str, "Go:%d",Gz_offset);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					SSD1306_GotoXY(1, 52);
					sprintf(str, "Gz:%d",G_Z);
					SSD1306_Puts(str, &Font_7x10, SSD1306_COLOR_WHITE);

					break;
				default:
					break;
			}

			break;
		default:
			break;
	}
	flags.bit1.screenState = TRUE;
}





void auto_flow(){
	switch (automatic.follow_state) {
		case LOST:
			if(last_line == 0){
				left_Speed = (automatic.mid_Speed*automatic.lost_Speed)/100;
				right_Speed = (automatic.mid_Speed*automatic.lost_Speed)/100;
				turn_L = TRUE;
			}

			if(last_line == 7){
				left_Speed = (automatic.mid_Speed*automatic.lost_Speed)/100;
				right_Speed = (automatic.mid_Speed*automatic.lost_Speed)/100;
				turn_R = TRUE;
			}

			accelCounter = 0;

			break;
		case FOLLOWING:
			turn_L = FALSE;
			turn_R = FALSE;

			if(abs(error_t) <= automatic.deadZone){
				accelCounter++;
				if(accelCounter >= automatic.accEntry){
					right_Speed = (automatic.mid_Speed*automatic.acc_Speed)/100;
					left_Speed  = (automatic.mid_Speed*automatic.acc_Speed)/100;
				}else{
					right_Speed = automatic.mid_Speed;
					left_Speed  = automatic.mid_Speed;
				}
			}else{
				right_Speed = automatic.mid_Speed - PID_R;
				left_Speed  = automatic.mid_Speed + PID_L;
				accelCounter = 0;
				if(right_Speed <= MOTOR_MIN_SPEED){
					right_Speed = 0;
				}
				if(left_Speed <= MOTOR_MIN_SPEED){
					left_Speed = 0;
				}
			}

			break;
		default:
			left_Speed = 0;
			right_Speed = 0;
			break;
	}
}


void tracker_flow(){
	switch (tracker.follow_state) {
		case LOST:
			if(last_line == 0){
				left_Speed = (tracker.mid_Speed*tracker.lost_Speed)/100;
				right_Speed = (tracker.mid_Speed*tracker.lost_Speed)/100;
				turn_L = TRUE;
			}

			if(last_line == 7){
				left_Speed = (tracker.mid_Speed*tracker.lost_Speed)/100;
				right_Speed = (tracker.mid_Speed*tracker.lost_Speed)/100;
				turn_R = TRUE;
			}

			break;
		case FOLLOWING:
			turn_L = FALSE;
			turn_R = FALSE;


			if(abs(error_t) <= tracker.deadZone){
				right_Speed = tracker.mid_Speed;
				left_Speed  = tracker.mid_Speed;
			}else{
				right_Speed = tracker.mid_Speed - PID_R;
				left_Speed  = tracker.mid_Speed + PID_L;
				if(right_Speed <= MOTOR_MIN_SPEED){
					right_Speed = 0;
				}
				if(left_Speed <= MOTOR_MIN_SPEED){
					left_Speed = 0;
				}
			}

			break;
		default:
			left_Speed = 0;
			right_Speed = 0;
			break;
	}
}


// Funciones de la HAL

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	ESP01_WriteRX(tempRxUART[0]);
	HAL_UART_Receive_IT(&huart1, tempRxUART, 1);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM2)
  {
	  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_raw, 8);
	  elapsed.oneSec++;
	  samples++;
  }
}


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C2) {
    	if(flags.bit2.Acc_Ask_MPU == TRUE){
    		flags.bit2.Acc_Ask_MPU = FALSE;
    		flags.bit2.Acc_Ready_MPU = TRUE;
    	}
    	if(flags.bit2.Gyr_Ask_MPU == TRUE){
    		flags.bit2.Gyr_Ask_MPU = FALSE;
    		flags.bit2.Gyr_Ready_MPU = TRUE;
    	}
    	if(flags.bit2.Temp_Ask_MPU == TRUE){
    		flags.bit2.Temp_Ask_MPU = FALSE;
    		flags.bit2.Temp_Ready_MPU = TRUE;
    	}
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  // Definición de estados y valores iniciales
  bufferESP01.Txiw = 0;
  bufferESP01.Txir = 0;

  mode = 0;

  tracker.follow_state = FOLLOWING;
  tracker.mid_Speed = 0;
  tracker.accEntry = 0;
  tracker.acc_Speed = 100;
  tracker.back_Speed = 40;
  tracker.lost_Speed = 80;
  tracker.deadZone = 70;
  tracker.returnZone = 300;
  tracker.accZone = 10;
  tracker.kp_L = 0;
  tracker.ki_L = 0;
  tracker.kd_L = 0;
  tracker.kp_R = 0;
  tracker.ki_R = 0;
  tracker.kd_R = 0;

  automatic.follow_state = FOLLOWING;
  automatic.mid_Speed = 0;
  automatic.accEntry = 10;
  automatic.acc_Speed = 80;
  automatic.back_Speed = 0;
  automatic.lost_Speed = 80;
  automatic.deadZone = 70;
  automatic.returnZone = 300;
  automatic.accZone = 10;
  automatic.kp_L = 0;
  automatic.ki_L = 0;
  automatic.kd_L = 0;
  automatic.kp_R = 0;
  automatic.ki_R = 0;
  automatic.kd_R = 0;

  currentESP01states = ESP01_FIRST_READY;
  sendTime = 200;

  screenState = 0;

  flags.bit1.updateScreen = FALSE;
  flags.bit1.motores = FALSE;
  flags.bit1.sendReadyUSB = FALSE;

  flags.bit2.Gyr_First_Offset = TRUE;

  flags.bit3.turn_L = FALSE;
  flags.bit3.turn_R = FALSE;

  _sESP01Handle handleESP01;
  handleESP01.DoCHPD = ESP01_CH_PD_Set;
  handleESP01.WriteUSARTByte = ESP01_UART_Send;
  handleESP01.WriteByteToBufRX = ESP01_UART_Receive;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  ESP01_Init(&handleESP01);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  // LED2 => PB4 = Verde
  // LED1 => PB3 = Amarillo

  MPU6050_Init();
  HAL_Delay(100);
  SSD1306_Init();

/*
* Pixeles muertos: Y = 1 , 3 , 5 , 7 , 9 , 11 , 13 , 15 , 17 , 19 , 21
* Y >= 22 para no tener pixeles rotos
*/

  CDC_Attach_RxFun(onUSBRx);

  ESP01_AttachChangeState(ESP01_ChangeState);

  ESP01_SetWIFI(currentConfig.ssid, currentConfig.password);
  ESP01_StartUDP(currentConfig.remoteIP, currentConfig.localPort, currentConfig.remotePort);

  HAL_UART_Receive_IT(&huart1, tempRxUART, 1);

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start_IT(&htim2);

// Izquierda
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // Atras
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // Adelante

// Derecha
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // Adelante
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);// Atras


/*
======================= ADC CONFIG =======================

Fórmula tiempo de conversión (Tconv):
   Tconv = SamplingTime + 12.5 ciclos

Configuración actual:
   ADC Clock = 12 MHz
   Frecuencia deseada = 4 kHz (Tconv necesario = 250 µs)

Caso SamplingTime = 1.5 ciclos:
   Tconv (1 canal) = (1.5 + 12.5) / 12 MHz = 1.166 µs
   Tconv (8 canales) = 8 * (1.5 + 12.5) / 12 MHz ≈ 9.33 µs
   -> Rápido, pero baja precisión.

Caso SamplingTime = 7.5 ciclos:
   Tconv (1 canal) = (7.5 + 12.5) / 12 MHz = 1.67 µs
   Tconv (8 canales) = 8 * (7.5 + 12.5) / 12 MHz ≈ 13.33 µs
   -> Mejor precisión, sigue siendo suficientemente rápido.

Trigger externo (TIM2):
   Se configura TIM2 para generar un evento cada 250 µs (4 kHz):
	   - Prescaler = 71  -> f_timer = 72 MHz / (71+1) = 1 MHz (tick = 1 µs)
	   - Period    = 249 -> Overflow cada 250 ticks = 250 µs
	   - Frecuencia final = 1 / 250 µs = 4 kHz

==========================================================
*/

/*
======================= PWM CONFIG =======================

Fórmula general:
	f_pwm = f_timer / ((PSC+1) * (ARR+1))

Configuración actual:
	Timer Clock = 72 MHz
	Prescaler (PSC) = 719
	Period (ARR)   = 999

Cálculos:
	f_timer = 72 MHz / (719+1) = 100 kHz  (tick = 10 µs)
	Período = (999+1) ticks = 1000 * 10 µs = 10 ms
	Frecuencia PWM = 1 / 10 ms = 100 Hz

Resultado:
	- TIM4 genera PWM de 100 Hz en 4 canales (CH1..CH4).
	- Duty Cycle = CCRx / (ARR+1)
	 (por ejemplo, CCR1 = 500 -> 50%)

 ============================================================
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

// ╔══════════════════════════════════════════════════════════════╗
// ║                 		   HEARTBEAT  			              ║
// ╚══════════════════════════════════════════════════════════════╝
	if(TIME_ELAPSED(elapsed.oneSec,elapsed.heartBeat,100)){
	  elapsed.heartBeat = elapsed.oneSec;

	  switch (mode) {
		 case 0:
			 switch (hbStep) {
				 case 0: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
				 case 1: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   break;
				 default: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
			 }
			 hbStep = (hbStep + 1) % 10;
			 break;
		 case 1:
			 switch (hbStep) {
				 case 0: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
				 case 1: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   break;
				 case 2: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
				 case 3: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   break;
				 default: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
			 }
			 hbStep = (hbStep + 1) % 10;
			 break;
		 case 2:
			 switch (hbStep) {
				 case 0: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
				 case 1: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   break;
				 case 2: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
				 case 3: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   break;
				 case 4: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
				 case 5: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);   break;
				 default: HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET); break;
			 }
			 hbStep = (hbStep + 1) % 10;
			 break;
		 default:
			 // Por si hay un modo no reconocido
			 HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			 hbStep = 0;
			 break;
	  }
	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                    		 ADC	    	    		      ║
// ╚══════════════════════════════════════════════════════════════╝
	if(samples >= 4000){		// Control de 4 ksamples
	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  samples = 0;
	}

	if(samples != count){
	  if(samples > count){
		  SMA_Update(&SMA_Filter);
	  }
	  count = samples;
	}

	if(TIME_ELAPSED(elapsed.oneSec,elapsed.PID,10)){
		elapsed.PID = elapsed.oneSec;
		position_IC();
		follow_State();

		// IZQUIERDA
		if(turn_L == TRUE){
			htim4.Instance->CCR1 = left_Speed; 	// ATRAS
			htim4.Instance->CCR2 = 0;			// ADELANTE
		}else{
			htim4.Instance->CCR1 = 0; 			// ATRAS
			htim4.Instance->CCR2 = left_Speed;	// ADELANTE
		}

		//DERECHA
		if(turn_R == TRUE){
			htim4.Instance->CCR4 = right_Speed;	// ATRAS
			htim4.Instance->CCR3 = 0;			// ADELANTE
		}else{
			htim4.Instance->CCR3 = right_Speed;	// ADELANTE
			htim4.Instance->CCR4 = 0;			// ATRAS
		}

		switch (mode) {
		case 0:
//			auto_flow();
			left_Speed = 0;
			right_Speed = 0;
			break;
		case 1:
			auto_flow();
			break;
		case 2:
//			tracker_flow();
			break;
		default:
			break;
		}
	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                    		 MOTORES    	    		      ║
// ╚══════════════════════════════════════════════════════════════╝

//	// IZQUIERDA
//	if(turn_L == TRUE){
//		htim4.Instance->CCR1 = left_Speed; 	// ATRAS
//		htim4.Instance->CCR2 = 0;			// ADELANTE
//	}else{
//		htim4.Instance->CCR1 = 0; 			// ATRAS
//		htim4.Instance->CCR2 = left_Speed;	// ADELANTE
//	}
//
//	//DERECHA
//	if(turn_R == TRUE){
//		htim4.Instance->CCR4 = right_Speed;	// ATRAS
//		htim4.Instance->CCR3 = 0;			// ADELANTE
//	}else{
//		htim4.Instance->CCR3 = right_Speed;	// ADELANTE
//		htim4.Instance->CCR4 = 0;			// ATRAS
//	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                     LÓGICA DEL VELOCISTA    		          ║
// ╚══════════════════════════════════════════════════════════════╝

//	switch (mode) {
//	case 0:
//		auto_flow();
//		left_Speed = 0;
//		right_Speed = 0;
//		break;
//	case 1:
//		auto_flow();
//		break;
//	case 2:
//		tracker_flow();
//		break;
//	default:
//		break;
//	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                  	 	BUTTON DEBOUNCE	    			      ║
// ╚══════════════════════════════════════════════════════════════╝

// BT1
	BT_Debounce(&buttons[0]);

	// Prolongada
	if(buttons[0].hold_detected){

	  screenState--;
	  if(screenState < 0)
		  screenState = 5;

	  optionState = 0;

	  buttons[0].hold_triggered = TRUE;
	  buttons[0].hold_release = TRUE;
	  buttons[0].hold_detected = 0;
	}

	// Instantánea
	if ((buttons[0].state == BTN_STATE_RISING) && (buttons[0].hold_release == FALSE)) {
	  screenState++;
	  if(screenState >= 6)
		  screenState = 0;
	  optionState = 0;
	}

// BT2
	BT_Debounce(&buttons[1]);

	// Prolongada
	if(buttons[1].hold_detected){

	  switch (screenState) {
		case 0:
			mode = optionState;
			if(mode == 0)
				sendTime = 200;
			if(mode == 1)
				sendTime = automatic.send_time;
			if(mode == 2)
				sendTime = tracker.send_time;
			currentESP01states = ESP01_MODE;
			break;
		case 1:

			break;
		case 2:
			network = optionState;
			switch(network){
				case 0:
					strcpy(currentConfig.ssid,"LabPrototip");
					strcpy(currentConfig.password,"labproto");
					strcpy(currentConfig.remoteIP,"192.168.2.112");
					currentESP01states = ESP01_RECONNECT;
					break;
				case 1:
					strcpy(currentConfig.ssid,"InternetPlus_ecf7f0");
					strcpy(currentConfig.password,"wlan13080f");
					strcpy(currentConfig.remoteIP,"192.168.1.13");
					currentESP01states = ESP01_RECONNECT;
					break;
			}
			break;
		case 3:

			break;
		case 4:

			break;
		default:
			break;
	}

	  buttons[1].hold_triggered = TRUE;
	  buttons[1].hold_release = TRUE;
	  buttons[1].hold_detected = 0;
	}

	// Instantanea
	if ((buttons[1].state == BTN_STATE_RISING) && (buttons[1].hold_release == FALSE)) {
	  optionState++;
	  switch (screenState) {
		case 0:					// MENU
			if(optionState >= 3)
				optionState = 0;
			break;
		case 1:					// GENERAL
			if(optionState >= 2)
				optionState = 0;
			break;
		case 2:					// CONNECT
			if(optionState >= 2)
				optionState = 0;
			break;
		case 3:					// MOTORS
			if(optionState >= 2)
				optionState = 0;
			break;
		case 4:					// LINE
			if(optionState >= 2)
				optionState = 0;
			break;
		case 5:					// MPU6050
			if(optionState >= 3)
				optionState = 0;
			break;
		default:
			break;
	  }
	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                           MPU6050                            ║
// ╚══════════════════════════════════════════════════════════════╝

	if(TIME_ELAPSED(elapsed.oneSec,elapsed.mpu,60)){
	  elapsed.mpu = elapsed.oneSec;
	  switch (data_ac) {
		case 0:
			MPU6050_Read_Accel();
			data_ac = 1;
			break;
		case 1:
			MPU6050_Read_Gyro();
			data_ac = 2;
			break;
		case 2:
			MPU6050_Read_Temp();
			data_ac = 0;
			break;
		default:
			break;
	  }
	}

	if(TIME_ELAPSED(elapsed.oneSec,elapsed.gyro_time,100)){
		elapsed.gyro_time = elapsed.oneSec;
		flags.bit2.Acc_Ready_MPU = FALSE;
		Accel_X_RAW = (int16_t)(Rec_Data_A[0] << 8 | Rec_Data_A [1]);
		Accel_Y_RAW = (int16_t)(Rec_Data_A[2] << 8 | Rec_Data_A [3]);
		Accel_Z_RAW = (int16_t)(Rec_Data_A[4] << 8 | Rec_Data_A [5]);

		A_X = Accel_X_RAW*981/16384.0;
		A_Y = Accel_Y_RAW*981/16384.0;
		A_Z = Accel_Z_RAW*981/16384.0;
	}

	if(flags.bit2.Gyr_Ready_MPU == TRUE){
		flags.bit2.Gyr_Ready_MPU = FALSE;
		Gyro_X_RAW = (int16_t)(Rec_Data_G[0] << 8 | Rec_Data_G[1]);
		Gyro_Y_RAW = (int16_t)(Rec_Data_G[2] << 8 | Rec_Data_G[3]);
		Gyro_Z_RAW = (int16_t)(Rec_Data_G[4] << 8 | Rec_Data_G[5]);

		G_X = Gyro_X_RAW / 131.0;
		G_Y = Gyro_Y_RAW / 131.0;
		G_Z = Gyro_Z_RAW / 131.0;

		if(flags.bit2.Gyr_First_Offset == TRUE){
			MPU6050_Gyro_Setup();
		}else{
			MPU6050_SMA_G();
		}
	}

	if(flags.bit2.Temp_Ready_MPU == TRUE){
		flags.bit2.Temp_Ready_MPU = FALSE;
		T_RAW = (int16_t)(Rec_Data_T[0] << 8 | Rec_Data_T[1]);
		T_MPU = (T_RAW / 340.0f) + 36.53f;
	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                          DISPLAY                             ║
// ╚══════════════════════════════════════════════════════════════╝

	  if(TIME_ELAPSED(elapsed.oneSec,elapsed.ssd1306,42)){ // 42 ms para 24 FPS
		  elapsed.ssd1306 = elapsed.oneSec;
		  SSD1306_Display();
	  }

	  if ((HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) && flags.bit1.screenState) {
		  flags.bit1.screenState = SSD1306_UpdateScreen();
	  }


// ╔══════════════════════════════════════════════════════════════╗
// ║                     COMUNICACIÓN USB                         ║
// ╚══════════════════════════════════════════════════════════════╝

	if (TIME_ELAPSED(elapsed.oneSec,elapsed.usbReceive,UART_TIMEOUT_MS) && (elapsed.usbReceive > 0)){
		decodeProtocol(&bufferUSB);
		decodeData(&bufferUSB);
		elapsed.usbReceive = 0;

		if(flags.bit1.sendReadyUSB == TRUE){
		  encodeProtocol(&bufferUSB,bufferUSB.payloadTxLength, estadoComandos);
		  CDC_Transmit_FS(bufferUSB.bufferTx + bufferUSB.Txir,ringBufferSize(&bufferUSB, 1));
		  bufferUSB.Txir = bufferUSB.Txiw;
		  flags.bit1.sendReadyUSB = FALSE;
		}
	}

// ╔══════════════════════════════════════════════════════════════╗
// ║                     COMUNICACIÓN WIFI                        ║
// ╚══════════════════════════════════════════════════════════════╝

	  ESP01_Task();

	  if(TIME_ELAPSED(elapsed.oneSec,elapsed.esp01Event,10)){
		  elapsed.esp01Event = elapsed.oneSec;
		  ESP01_Timeout10ms();
	  }

	  if (TIME_ELAPSED(elapsed.oneSec,elapsed.esp01Receive,UART_TIMEOUT_MS) && (elapsed.esp01Receive > 0)){
		  decodeProtocol(&bufferESP01);
		  decodeData(&bufferESP01);
		  elapsed.esp01Receive = 0;
	  }

	  switch (currentESP01states) {
		  case ESP01_FIRST_READY:
			  if (TIME_ELAPSED(elapsed.oneSec,elapsed.esp01Send,8000)) {
				  if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED) {
					  encodeProtocol(&bufferESP01,0, ALIVE);
					  ESP01_Send(bufferESP01.bufferTx, bufferESP01.Txir, ringBufferSize(&bufferESP01, 1), MAX_BUFFER_SIZE);
					  bufferESP01.Txir = bufferESP01.Txiw;
				  }
				  elapsed.esp01Send = elapsed.oneSec;
			  }
			  break;
		  case ESP01_RECONNECT:
			  ESP01_SetWIFI(currentConfig.ssid, currentConfig.password);
			  ESP01_StartUDP(currentConfig.remoteIP, currentConfig.localPort, currentConfig.remotePort);
			  currentESP01states = ESP01_FIRST_READY;
			  break;
		  case ESP01_MODE:

			  bufferESP01.payloadTxLength = 0;
			  send_index = 0;

			  bufferESP01.payloadTx[send_index++] = mode;

			  encodeProtocol(&bufferESP01,1, CMD_WORK_MODE);

			  ESP01_Send(bufferESP01.bufferTx, bufferESP01.Txir, ringBufferSize(&bufferESP01, 1), MAX_BUFFER_SIZE);
			  bufferESP01.Txir = bufferESP01.Txiw;

			  currentESP01states = ESP01_FIRST_READY;
			  break;
		  case ESP01_MPU:
			  if (TIME_ELAPSED(elapsed.oneSec,elapsed.esp01Send,sendTime)) {
				  if (ESP01_StateUDPTCP() == ESP01_UDPTCP_CONNECTED) {

					  bufferESP01.payloadTxLength = 0;
					  send_index = 0;

					  myWord.as_float = A_X;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_float = A_Y;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_float = A_Z;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_float = G_X;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_float = G_Y;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_float = G_Z;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.int16_parts.low_int16 = angle;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[0];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[1];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[2];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[3];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[4];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[5];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[6];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.uint16_parts.low_uint16 = adc_clean[7];
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;

					  myWord.as_int32  = error_t;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_int32  = P;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_int32  = I;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  myWord.as_int32  = D;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte0;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte1;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte2;
					  bufferESP01.payloadTx[send_index++] = myWord.bytes.byte3;

					  encodeProtocol(&bufferESP01,send_index, CMD_MPU);

					  ESP01_Send(bufferESP01.bufferTx, bufferESP01.Txir, ringBufferSize(&bufferESP01, 1), MAX_BUFFER_SIZE);
					  bufferESP01.Txir = bufferESP01.Txiw;
				  }
				  elapsed.esp01Send = elapsed.oneSec;
			  }

			  break;
		  default:
			  // Otros estados
			  break;
	  }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */
  hi2c2.hdmatx = &hdma_i2c2_tx;			//	===================> Mucho muy muy importante
  hi2c2.hdmarx = &hdma_i2c2_rx;
  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 719;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CH_EN_GPIO_Port, CH_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BT2_Pin BT1_Pin */
  GPIO_InitStruct.Pin = BT2_Pin|BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CH_EN_Pin */
  GPIO_InitStruct.Pin = CH_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CH_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
