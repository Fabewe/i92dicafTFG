#define RAWSAMPLES 512
#define SAMPLERATE 48000


// Crea una instancia de la pantalla OLED con I2C
// double coefs[32] = {0.002855073321371961,
//     0.004217810045954073,
//     0.006070979752995526,
//     0.008513978883235714,
//     0.011633445764465426,
//     0.015487673582478250,
//     0.020089356277603183,
//     0.025389133050128021,
//     0.031263078326193357,
//     0.037507457211444148,
//     0.043843530949484712,
//     0.049933894562348549,
//     0.055409900908815000,
//     0.059907517556544401,
//     0.063106964465950877,
//     0.064770205340986750,
//     0.064770205340986750,
//     0.063106964465950877,
//     0.059907517556544401,
//     0.055409900908815000,
//     0.049933894562348549,
//     0.043843530949484712,
//     0.037507457211444148,
//     0.031263078326193357,
//     0.025389133050128021,
//     0.020089356277603183,
//     0.015487673582478250,
//     0.011633445764465426,
//     0.008513978883235714,
//     0.006070979752995526,
//     0.004217810045954073,
//     0.002855073321371961};

int coefs[32] = {0, 0, 1, 1, 2, 3, 4, 5, 7, 9, 11, 14, 16, 18, 18, 19, 19, 18, 17, 16, 14, 11, 9, 7, 5, 4, 3, 2, 1, 1, 0, 0};

double simplified[RAWSAMPLES/4];

bool detectado = false;

const uint_fast16_t samples = RAWSAMPLES; //This value MUST ALWAYS be a power of 2
float samplingFrequency = SAMPLERATE;

double x = 0;


volatile bool pressedButton = false;

//ISR_Vibracion
volatile unsigned int timer0 = 0;
volatile unsigned int counter= 0;
volatile unsigned int timer1 = 0;


String appEui = "0000000000000000";
String appKey = "568B75EED37504C0A879B0DDB678ED8D";
LoRaModem modem;          //Objeto para manejar la librería LoRa

volatile int nSound = 0;
volatile int nVib = 0;

uint32_t pdm_raw_samples[samples];
double vReal[samples];
double vImag[samples];


ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency); //Objeto para manejar la librería FFT
