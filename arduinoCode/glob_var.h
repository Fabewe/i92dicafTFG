#define RAWSAMPLES 512    //Numero de muestras
#define SAMPLERATE 46875  //Frecuencia de muestreo
#define MINSINTERVAL 15   //Minutos del periodo de recogida de datos


//Coeficientes para la decimación
int coefs[32] = {0, 0, 1, 1, 2, 3, 4, 5, 7, 9,   \
                11, 14, 16, 18, 18, 19, 19, 18,  \
                17, 16, 14, 11, 9, 7, 5, 4, 3, 2,\
                1, 1, 0, 0};


//ISR_Vibracion
volatile unsigned int timer0 = 0;
volatile unsigned int counter= 0;
volatile unsigned int timer1 = 0;


//Información TTN
String appEui = "X";
String appKey = "X";

//Objeto para manejar la librería LoRa
LoRaModem modem;          

//Índices de actividad
volatile int nSound = 0;
volatile int nVib = 0;

//Arrays para el sonido
uint32_t pdm_raw_samples[RAWSAMPLES];
double vReal[RAWSAMPLES];
double vImag[RAWSAMPLES];

//Timer para medir el periodo de envío
unsigned int time = 0;

//Objeto para manejar la librería FFT
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, RAWSAMPLES, SAMPLERATE); 
