#include <I2S.h>
#include <arduinoFFT.h>
#include <MKRWAN.h>




//Valores por parte del usuario
#define V_SENS 10               //Calibrar sensibilidad vibración
#define MINSINTERVAL 1          //Minutos del periodo de recogida de datos





#define SAMPLERATE 37500        //Frecuencia de muestro I2S
#define BITSPERSAMPLE 32        //Tamaño de  muestra I2S
#define RAWSAMPLES  512         //Número de muestras recogidas en cada análisis


      

// 
double coefs[32] = {0.002855073321371961,
    0.004217810045954073,
    0.006070979752995526,
    0.008513978883235714,
    0.011633445764465426,
    0.015487673582478250,
    0.020089356277603183,
    0.025389133050128021,
    0.031263078326193357,
    0.037507457211444148,
    0.043843530949484712,
    0.049933894562348549,
    0.055409900908815000,
    0.059907517556544401,
    0.063106964465950877,
    0.064770205340986750,
    0.064770205340986750,
    0.063106964465950877,
    0.059907517556544401,
    0.055409900908815000,
    0.049933894562348549,
    0.043843530949484712,
    0.037507457211444148,
    0.031263078326193357,
    0.025389133050128021,
    0.020089356277603183,
    0.015487673582478250,
    0.011633445764465426,
    0.008513978883235714,
    0.006070979752995526,
    0.004217810045954073,
    0.002855073321371961};

const uint16_t samples = RAWSAMPLES; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = SAMPLERATE;
const uint8_t amplitude = 1;



uint32_t pdm_raw_samples[samples];
double vReal[samples];
double vImag[samples];



//
String appEui = "0000000000000000";
String appKey = "568B75EED37504C0A879B0DDB678ED8D";

unsigned long time = 0;
unsigned int data = 0;
int err;

const byte interruptPin = 8;
volatile int nSig = 0;
volatile int nVib = 0;
int nSound = 0;


//Objetos de librerías a usar

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, samplingFrequency); //Objeto para manejar la librería FFT
LoRaModem modem;          //Objeto para manejar la librería LoRa
void increaseVib() {

  nSig++;
  if(nSig % V_SENS == 0){
    nVib++;
    nSig = 0;
  }
  
}

void setup() {

  Serial.begin(115200);

  pinMode(8, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), increaseVib, RISING);


  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };



  int connected = modem.joinOTAA(appEui, appKey);
  if (!connected) {
    Serial.println("Something went wrong; are you indoor? Move near a window and retry");
    while (1) {}
  }


  


  time = millis();
  
  if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLERATE, BITSPERSAMPLE)) {
    Serial.println("Failed to initialize I2S!");
    while (1);
  }

  Serial.end();

}




void loop() {

  
  


  I2S.flush();
  for(int i = 0 ; i<RAWSAMPLES ; i++){
    while(I2S.available()<4){};
    pdm_raw_samples[i] = I2S.read();
  }



  for(int i = 0; i<RAWSAMPLES ;i++){
    double sum = 0;
    uint32_t x = pdm_raw_samples[i];
    
    for(int j = 0 ; j < 32 ; j++){
      if( x >> (31-j)& 0x01){
        sum += coefs[j];
      }
    }
    vReal[i] = sum;
    vImag[i] = 0;
  }

  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();
  //double x = FFT.majorPeak(vReal, samples >> 1, samplingFrequency);



//Detectar curva en FFT
 if((vReal[34] > vReal[44]) && (vReal[51]>vReal[44])){

  nSound++;

 }



  //Hacer cada X minutos
  if( (millis() - time) >= (MINSINTERVAL * 60 * 1000) ){

    modem.beginPacket();
    modem.write(nVib);
    modem.write(nSound);
    err = modem.endPacket(true);

    if (err > 0) {
      nVib = 0 ;
      nSound = 0;
    }

    time = millis();
  }
}
