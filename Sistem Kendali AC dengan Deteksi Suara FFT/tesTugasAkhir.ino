#define BLYNK_TEMPLATE_ID "TMPL6dukzpSr9"
#define BLYNK_TEMPLATE_NAME "PanasonicACiot"
#define BLYNK_AUTH_TOKEN "t36j0W9c60vaZo3wEyC0jw-ergCpjcyk"

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <ir_Coolix.h> // Library AC Protokol Coolix
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include "arduinoFFT.h"
#include <DHT.h>

#define SAMPLES 128            // SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 9000 // Ts = Based on Nyquist, must be 2 times the highest expected frequency.
#define DHTPIN 2     // Pin data sensor DHT11
#define DHTTYPE DHT11 // Tipe sensor DHT11

char ssid[] = "KOST BANYU ALFA";  // Ganti dengan nama SSID WiFi Anda
char pass[] = "Banyualfakost";  // Ganti dengan password WiFi Anda

const uint16_t kIrLed = 4; 

int pushMode = 0;
int pushFan = 0;
int pushSwing = 0;

int togglePower = 0;
int toggleMode = 0;
int toggleFan = 0;
int toggleSwing = 0;
int temp = 17;
int notifMode, notifFan, notifSwing;

IRCoolixAC ac(kIrLed); // Inisialisasi Coolix AC 
IRsend irsend(kIrLed);
DHT dht(DHTPIN, DHTTYPE); // Inisialisasi sensor DHT
WidgetLCD lcd(V6);

arduinoFFT FFT = arduinoFFT();
unsigned int samplingPeriod;
unsigned long microSeconds;

double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

unsigned long detectionStartTime = 0; // Menyimpan waktu mulai deteksi
bool isDetectingFrequency = false; // Menyimpan status deteksi
unsigned long lastTemperatureCheck = 0; // Menyimpan waktu terakhir kali suhu diukur

void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  ac.begin();
  irsend.begin();
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Blynk.virtualWrite(V7, temp);
  lcdState();
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  dht.begin();
  setButtonState(false); // Disable buttons at start
}

void loop() {
  Blynk.run();

  if (isDetectingFrequency) {
    if (millis() - detectionStartTime < 2500) {
      detectFrequency();
    } else {
      isDetectingFrequency = false;
    }
  }

  if (millis() - lastTemperatureCheck >= 10000) {
    detectTemperature();
    lastTemperatureCheck = millis();
  }
}

void startFrequencyDetection() {
  detectionStartTime = millis();
  isDetectingFrequency = true;
}

void detectFrequency() {
  for(int i=0; i<SAMPLES; i++)
    {
        microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
     
        vReal[i] = analogRead(0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
        vImag[i] = 0; //Makes imaginary term 0 always

        while(micros() < (microSeconds + samplingPeriod))
        {
        }
    }
 
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

    if (peak >= 3000 && peak <= 4500) {
      lcd.print(6, 1, "|RESPONSE");
      Blynk.virtualWrite(V6, "|RESPONSE");
      Serial.println("AC MERESPON");
      delay(400);
    } else {
      lcd.print(6, 1, "|--------");
      Blynk.virtualWrite(V6, "|--------");
    }
}

void detectTemperature() {
  float temperatureC = dht.readTemperature();

  if (isnan(temperatureC)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("Temperature: ");
  Serial.print(temperatureC);
  Serial.println("°C");
}

BLYNK_WRITE(V0) {
  togglePower = param.asInt();
  if (togglePower == 1) { 
    ac.on();
    ac.setTemp(temp);
    Blynk.virtualWrite(V7, temp);
    Serial.println("AC ON");
    setButtonState(true); // Enable buttons
    ac.send();
    startFrequencyDetection();
    
  } else {
    ac.off();
    Serial.println("AC OFF");
    setButtonState(false); // Disable buttons 
    ac.send();
    startFrequencyDetection(); 
  }
}

BLYNK_WRITE(V1) {
  if (togglePower == 1) { // Only process if AC is ON
    toggleMode = param.asInt();
    if (toggleMode == 1) {
      pushMode++;
      if (pushMode > 4) {
        pushMode = 0;
      }
      if (pushMode == 1) {
        notifMode = 2; // Mode Auto
        Serial.print("MODE: AUTO,");
        Serial.println("FAN: AUTO");
        lcd.print(0, 0, "M:AUTO");
        Blynk.setProperty(V2, "isDisabled", true);
        notifFan = 5; // Auto
        lcd.print(7, 0, "F:AUTO");
        ac.setFan(notifFan);
      } else if (pushMode == 2) {
        Blynk.setProperty(V2, "isDisabled", false);
        notifMode = 0; // Mode Cool
        Serial.println("MODE: COOL");
        lcd.print(0, 0, "M:COOL");
      } else if (pushMode == 3) {
        notifMode = 1; // Mode Dry
        Serial.print("MODE: DRY,");
        Serial.println("FAN: AUTO");
        lcd.print(0, 0, "M:DRY ");
        Blynk.setProperty(V2, "isDisabled", true);
        notifFan = 5; // Auto
        lcd.print(7, 0, "F:AUTO");
        ac.setFan(notifFan);
      } else if (pushMode == 4) {
        Blynk.setProperty(V2, "isDisabled", false);
        notifMode = 3; // Mode Heat
        Serial.println("MODE: HEAT");
        lcd.print(0, 0, "M:HEAT");
      } else {
        Blynk.setProperty(V2, "isDisabled", false);
        notifMode = 4; // Mode Fan
        Serial.println("MODE: FAN");
        lcd.print(0, 0, "M:FAN ");
      }
      ac.setMode(notifMode);
      ac.send();
      startFrequencyDetection(); 
    }
  }
}

BLYNK_WRITE(V2) {
  if (togglePower == 1) { // Only process if AC is ON
    toggleFan = param.asInt();
    if (toggleFan == 1) {
      pushFan++;
      if (pushFan > 3) {
        pushFan = 0;
      }
      if (pushFan == 1) {
        notifFan = 4; // Min
        Serial.println("FAN: -");
        lcd.print(7, 0, "F:-   ");
      } else if (pushFan == 2) {
        notifFan = 2; // Med
        Serial.println("FAN: --");
        lcd.print(7, 0, "F:--  ");
      } else if (pushFan == 3) {
        notifFan = 1; // Max
        Serial.println("FAN: ---");
        lcd.print(7, 0, "F:--- ");
      } else {
        notifFan = 5; // Auto
        Serial.println("FAN: AUTO");
        lcd.print(7, 0, "F:AUTO");
      }
      ac.setFan(notifFan);
      ac.send();
      startFrequencyDetection(); 
    }
  }
}

BLYNK_WRITE(V3) {
  if (togglePower == 1) { // Only process if AC is ON
    toggleSwing = param.asInt();
    if (toggleSwing == 1) {
      pushSwing++;
      if (pushSwing > 2) {
        pushSwing = 1; // Set kembali ke 1 jika melebihi 2
      }
      if (pushSwing == 1) {
        notifSwing = 1; // Aktifkan fitur Swing
        Serial.println("SWING: ON");
        lcd.print(0, 1, "S:ON ");
      } else if (pushSwing == 2) {
        notifSwing = 0; // Nonaktifkan fitur Swing
        Serial.println("SWING: OFF");
        lcd.print(0, 1, "S:OFF");
      }
      ac.setSwing();
      ac.send();
      startFrequencyDetection(); 
    }
  }
}

BLYNK_WRITE(V4) {
  if (togglePower == 1) { // Only process if AC is ON
    int tempUp = param.asInt();
    if (tempUp == 1) { 
      temp++;
      if (temp > 30) {
        temp = 30;
      }
      Blynk.virtualWrite(V7, temp);
      Serial.println("Menaikkan Suhu AC");
      ac.setTemp(temp);
      ac.send();
      startFrequencyDetection();
    }
  }
}

BLYNK_WRITE(V5) {
  if (togglePower == 1) { // Only process if AC is ON
    int tempDown = param.asInt();
    if (tempDown == 1) { 
      temp--;
      if (temp < 17) {
        temp = 17;
      }
      Blynk.virtualWrite(V7, temp);
      Serial.println("Menurunkan Suhu AC");
      ac.setTemp(temp);
      ac.send();
      startFrequencyDetection();
    }
  }
}

void lcdState() {
  lcd.clear();
  lcd.print(0, 0, "M");
  lcd.print(1, 0, ":AUTO ");
  lcd.print(7, 0, "F");
  lcd.print(8, 0, ":AUTO ");
  lcd.print(0, 1, "S");
  lcd.print(1, 1, ":ON ");
  lcd.print(6, 1, "|--------");
}

void setButtonState(bool state) {
  Blynk.setProperty(V1, "isDisabled", !state);
  Blynk.setProperty(V2, "isDisabled", !state);
  Blynk.setProperty(V3, "isDisabled", !state);
  Blynk.setProperty(V4, "isDisabled", !state);
  Blynk.setProperty(V5, "isDisabled", !state);
}
