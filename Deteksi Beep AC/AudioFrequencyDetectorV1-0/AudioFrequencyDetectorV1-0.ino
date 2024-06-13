#include "arduinoFFT.h"
 
#define SAMPLES 128            
#define SAMPLING_FREQUENCY 9000

arduinoFFT FFT = arduinoFFT();
 
unsigned int samplingPeriod;
unsigned long microSeconds;
 
double vReal[SAMPLES]; 
double vImag[SAMPLES]; 
 
void setup() 
{
    Serial.begin(115200); 
    samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); 
}
 
void loop() 
{ 
    for(int i=0; i<SAMPLES; i++)
    {
        microSeconds = micros();    
     
        vReal[i] = analogRead(0); 
        vImag[i] = 0; 
 
        while(micros() < (microSeconds + samplingPeriod))
        {

        }
    }
 
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    Serial.println(peak);     
 
    delay (200); 
}
