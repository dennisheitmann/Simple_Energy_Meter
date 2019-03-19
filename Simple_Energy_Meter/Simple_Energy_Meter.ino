/*  2019 Dennis Heitmann based on work of 2015 Tisham Dhar
    licensed under GNU GPL

    NodeMCU 1.0
    160 MHz
    4M (1M SPIFFS)
*/
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <PubSubClient.h>

//const char mqtt_server[]  = "";
//const int  mqtt_port      = 1883;
//const char mqtt_name[]    = "";
//const char mqtt_user[]    = "";
//const char mqtt_pass[]    = "";
//
//const char mqtt_topic_1[] = "Vrms";
//const char mqtt_topic_2[] = "Irms";
//const char mqtt_topic_3[] = "VA";

#include "mqttpw.h"

void callback(char* topic, byte* payload, unsigned int length) {
  // nothing
}

WiFiClient espClient;
PubSubClient client(mqtt_server, mqtt_port, callback, espClient);

boolean reconnect() {
  // Loop until we're reconnected
  int trycount = 0;
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqtt_name, mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      trycount = 0;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      unsigned long lastMillis = millis();
      while (lastMillis + 3000 > millis()) {
        yield();
      }
      trycount++;
      if (trycount > 600) {
        ESP.reset();
      }
    }
  }
  return client.connected();
}

Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */
//Maximum value of ADS
#define ADC_COUNTS 32768
//MOBIL
#define VCAL 1.231
#define ICAL 0.053
//DO7DH
//#define VCAL 1.539
//#define ICAL 0.053

double filteredI; //Filtered_ is the raw analog value minus the DC offset
double filteredV; //Filtered_ is the raw analog value minus the DC offset
int sampleV;      //sample_ holds the raw analog read value
int sampleI;      //sample_ holds the raw analog read value

double offsetV;   //Low-pass filter output
double offsetI;   //Low-pass filter output

double apparentPower, Vrms, Irms;
int startV;       //Instantaneous voltage at start of sample window.
double sqV, sumV, sqI, sumI, sumP; //sq = squared, sum = Sum, inst = instantaneous
boolean lastVCross, checkVCross; //Used to measure number of times threshold is crossed.

void calcVI(unsigned int crossings, unsigned int timeout)
{

  unsigned long crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned long numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st = false;                                //an indicator to exit the while loop

  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while (st == false)                                //the while loop...
  {
    startV = ads.readADC_Differential_2_3();                    //using the voltage waveform
    if ((abs(startV) < (ADC_COUNTS * 0.55)) && (abs(startV) > (ADC_COUNTS * 0.45))) st = true; //check its within range
    if ((millis() - start) > timeout) {
      Serial.println("Timeout!");
      st = true;
    }
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis() - start) < timeout))
  {
    numberOfSamples++;                       //Count number of times looped.

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleI = ads.readADC_Differential_0_1();                 //Read in raw voltage signal
    sampleV = ads.readADC_Differential_2_3();                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV - offsetV) / 1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI - offsetI) / 1024);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV = filteredV * filteredV;                //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
    else checkVCross = false;
    if (numberOfSamples == 1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.
  // float multiplier
  float multiplier = 0.015625F;
  double V_RATIO = VCAL * multiplier;
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL * multiplier;
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  //Calculation power values
  apparentPower = Vrms * Irms;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
  //--------------------------------------------------------------------------------------
}

void sendMQTT() {
  if (! client.connected()) {
    reconnect();
    yield();
  }
  char Vrms_str[8];
  dtostrf(Vrms, 5, 1, Vrms_str);
  if (! client.publish(mqtt_topic_1, Vrms_str)) {
    Serial.println(F("Vrms publish failed"));
  } else {
    // Serial.println(F("Vrms publish OK!"));
  }
  char Irms_str[8];
  dtostrf(Irms, 5, 2, Irms_str);
  if (! client.publish(mqtt_topic_2, Irms_str)) {
    Serial.println(F("Irms publish failed"));
  } else {
    // Serial.println(F("Irms publish OK!"));
  }
  char apparentPower_str[8];
  dtostrf(apparentPower, 4, 0, apparentPower_str);
  if (! client.publish(mqtt_topic_3, apparentPower_str)) {
    Serial.println(F("apparentPower publish failed"));
  } else {
    // Serial.println(F("apparentPower publish OK!"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(10);

  WiFiManager wifiManager;
  wifiManager.setTimeout(900);
  if (!(wifiManager.autoConnect())) {
    delay(100);
    ESP.reset();
    delay(100);
  }

  ads.setGain(GAIN_EIGHT);
  ads.begin();

  Serial.println("Equilibrating");
  calcVI(200, 15000);
  Serial.println("...");
  calcVI(200, 15000);
  Serial.println("Done!");

}

void loop() {

  client.disconnect();
  delay(250);
  // 1 Measurement
  calcVI(100, 10000);
  double apparentPowerX = apparentPower;
  double IrmsX = Irms;
  double VrmsX = Vrms;
  delay(250);
  // 2 Measurement
  calcVI(100, 10000);
  apparentPowerX += apparentPower;
  IrmsX += Irms;
  VrmsX += Vrms;
  delay(250);
  // 3 Measurement
  calcVI(100, 10000);
  apparentPowerX += apparentPower;
  IrmsX += Irms;
  VrmsX += Vrms;
  delay(250);
  // 4 Measurement
  calcVI(100, 10000);
  apparentPowerX += apparentPower;
  IrmsX += Irms;
  VrmsX += Vrms;
  delay(250);
  // 5 Measurement
  calcVI(100, 10000);
  apparentPowerX += apparentPower;
  IrmsX += Irms;
  VrmsX += Vrms;
  // Calibration
  apparentPower = apparentPowerX / 5;
  Irms = IrmsX / 5;
  Vrms = VrmsX / 5;
  Serial.println("--------------------");
  Serial.print("VA: ");
  Serial.println(apparentPower);
  Serial.print("Irms: ");
  Serial.println(Irms);
  Serial.print("Vrms: ");
  Serial.println(Vrms);
  sendMQTT();
  delay(250);

}
