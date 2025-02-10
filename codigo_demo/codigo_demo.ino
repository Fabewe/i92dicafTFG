#include <Arduino.h>
#include <Wire.h>
#include <arduinoFFT.h>
#include <MKRWAN.h>
#include "glob_var.h"



void ISR_Vibracion(){

  counter++;

  timer1 = millis();

  if((timer1 - timer0)> 4500) counter= 0;

  if((timer1 - timer0)> 3500){

    //Serial.println(counter);
    
    if(counter>10 && counter<250){
        Serial.println("VIBRACION - DETECTADA");
        nVib++;
    }counter  = 0;
    timer0 = timer1;
  }
}


void ISR_Boton(){

  pressedButton = true;
}


void enviarDatos(){
  int err;
  modem.beginPacket();
  modem.write(nVib);
  modem.write(nSound);
  err = modem.endPacket(true);

  if (err > 0) {
    nVib = 0 ;
    nSound = 0;
    Serial.println(" ==> DATOS ENVIADOS ==>");
  }
}





void i92_I2SBEGIIN(){


  /* Encender interfaz del GCLK */
  PM->APBAMASK.reg |= PM_APBAMASK_GCLK ;

  /* CONFIGURACION DIVISOR */
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) |  // Configurar el divisor de GCLK - 2
                     GCLK_GENDIV_DIV(16)  // Configurar el divisor /16 | 48/16MHz = 3 MHz
                     ; 

  /* CONFIGURACION GCLK02  */
  GCLK->GENCTRL.reg =
      GCLK_GENCTRL_ID(2) |                // Configurar el GCLK - 2
      GCLK_GENCTRL_SRC_DFLL48M |          // Seleccionar el oscilador de 48MHz como fuente
      GCLK_GENCTRL_IDC |                  // Mejor precisión
     // GCLK_GENCTRL_OE |                 // Output-Enable GCLK (GPIO PA14)
      GCLK_GENCTRL_GENEN                  // Enable GCLK
      ;

  while(GCLK->STATUS.bit.SYNCBUSY);       // SINCRONIZAR
 
  /* Encender interfaz del periférico I2S */
  PM->APBCMASK.reg |= PM_APBCMASK_I2S;


  /* Conectar GCLK02 CON EL I2S_GCLK_0 */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(I2S_GCLK_ID_0) |  // Seleccionar I2S_GCLK_0
                      GCLK_CLKCTRL_CLKEN |              // Enable 
                      GCLK_CLKCTRL_GEN(2);              // Enable GCLK02




   /* CONFIGURACION I2S_GCLK_0 */
   I2S->CLKCTRL[0].reg = I2S_CLKCTRL_MCKSEL_GCLK |       // Master Clock Select - GCLK_I2S_0 es usado como fuente de Master Clock 0
                         I2S_CLKCTRL_MCKEN |             // Master Clock Enable
                         I2S_CLKCTRL_MCKOUTDIV(1) |      // Master Clock Output Division Factor 
                         I2S_CLKCTRL_SCKSEL_MCKDIV |     // Serial Clock Select Divided  - Master Clock is used as Serial Clock 0 source
                         I2S_CLKCTRL_MCKDIV(1) |         // Master Clock Division Factor 
                         I2S_CLKCTRL_FSSEL_SCKDIV |      // Frame Sync Select - Divided Serial Clock n is used as Frame Sync n source
                         I2S_CLKCTRL_FSWIDTH_SLOT |      // Frame Sync Pulse is 1 Slot wide (default for I2S protocol)
                         I2S_CLKCTRL_NBSLOTS(1) |        // Número de slots
                         I2S_CLKCTRL_SLOTSIZE_32 |       // Tamaño slot - 32bits
                         I2S_CLKCTRL_BITDELAY_I2S;       // (I2S_CLKCTRL) I2S (1 Bit Delay)
  

  /* CONFIGURACION SERIALIZADOR */
  I2S->SERCTRL[0].reg = I2S_SERCTRL_SERMODE_RX | I2S_SERCTRL_MONO_MONO |
      I2S_SERCTRL_SLOTADJ_LEFT | I2S_SERCTRL_DATASIZE_32 | I2S_SERCTRL_CLKSEL_CLK0;

  I2S->CTRLA.reg = I2S_CTRLA_ENABLE | I2S_CTRLA_SEREN0 | I2S_CTRLA_CKEN0;
  while (I2S->SYNCBUSY.reg & I2S_SYNCBUSY_ENABLE);


  // Configurar el pin como salida GCLK
  PORT->Group[PORTA].PINCFG[10].reg = PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PMUX[10 / 2].reg |= PORT_PMUX_PMUXE_G; // PMUXE para el par bajo del pin;


  // Configurar el pin como entrada SDa
  PORT->Group[PORTA].PINCFG[7].reg = PORT_PINCFG_PMUXEN;
  PORT->Group[PORTA].PMUX[7 / 2].reg |= PORT_PMUX_PMUXO_G; // PMUXO para el par bajo del pin;


}


void computarAudio(){

  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double avg0 = 0;
  double avg1 = 0;
  double avg3 = 0;

  for(int i = 0 ; i < 32 ; i++){
    
    avg0 += vReal[49+i];
    
   }

   for(int i = 0 ; i < 21 ; i++){
    
    avg1 += vReal[1+i];
    
   }

   for(int i = 0 ; i < 28 ; i++){
    
    avg2 += vReal[22+i];
    
   }

  avg0 = avg0 / 32;
  avg1 = avg1 / 21;
  avg2 = avg2 / 28;


  if(avg0 > (1.8 * avg1 ) && (avg0 > avg2)){
    nSound++;
  }
  
}



void setup() {
    Serial.begin(9600);


    //Inicialización Micrófono
    i92_I2SBEGIIN();

    //Inicialización Sensor Vibraciones
    timer0 = millis();   
    attachInterrupt(digitalPinToInterrupt(6), ISR_Vibracion, FALLING);

    //Inicialización conexion lora
    if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
    }

    int connected = modem.joinOTAA(appEui, appKey);
    if (!connected) {
      Serial.println("Something went wrong; are you indoor? Move near a window and retry");
      while (1) {}
    }

    //CODIGO EXCLUSIVO DEMO

    attachInterrupt(digitalPinToInterrupt(5), ISR_Boton, FALLING);

}

void loop() {

    noInterrupts();
    int y = 0;
    while( y < RAWSAMPLES){
      while(!I2S->INTFLAG.bit.RXRDY0){}
      pdm_raw_samples[y]= I2S->DATA[0].reg;
      y++;
      
    }
    interrupts();

      for(int n = 0; n < RAWSAMPLES ; n++){
        uint32_t sample = pdm_raw_samples[n];
        double value = 0;
        for(int j = 0 ; j < 32 ; j++){
          if( sample >> (31-j)& 0x01){
            value += coefs[j];
          }
        }

        vReal[n] = value;
        vImag[n] = 0;
      }

    computarAudio();


    if(pressedButton){
      enviarDatos();
      pressedButton = false;
    }





}
