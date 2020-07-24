#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "algorithm_by_RF.h"
#include "BluetoothSerial.h"


#define DEBUG // Uncomment for debug output to the Serial stream
//#define USE_ADALOGGER // Comment out if you don't have ADALOGGER itself but your MCU still can handle this code
#define TEST_MAXIM_ALGORITHM // Uncomment if you want to include results returned by the original MAXIM algorithm
//#define DEBUG_FLEXIPLOT
//#define SAVE_RAW_DATA // Uncomment if you want raw data coming out of the sensor saved to SD card. Red signal first, IR second.
//#define USE_MAX_30105

#ifdef USE_MAX_30105
#include "MAX30105.h" //sparkfun MAX3010X library
MAX30105 particleSensor;
#else
#include "max30102.h"
#endif

#ifdef TEST_MAXIM_ALGORITHM
  #include "algorithm.h" 
#endif

// Interrupt pin
const byte oxiInt = 10; // pin connected to MAX30102 INT

// ADALOGGER pins
#ifdef USE_ADALOGGER
  File dataFile;
  const byte chipSelect = 4;
  const byte cardDetect = 7;
  const byte batteryPin = 9;
  const byte ledPin = 13; // Red LED on ADALOGGER
  const byte sdIndicatorPin = 8; // Green LED on ADALOGGER
  bool cardOK;
#endif

uint32_t elapsedTime,timeStart;

uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;

BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test");
  Serial.println("Device name: ESP32test");
  
  pinMode(oxiInt, INPUT);  //pin D10 connects to the interrupt output pin of the MAX30102

#ifdef USE_MAX_30105
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    while (1);
  }

  //Setup MAX30102
  byte ledBrightness = 0x1F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 8; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 400; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  // Set up the wanted parameters
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
#else
  Wire.begin();
  maxim_max30102_reset(); //resets the MAX30102
  delay(1000);

  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();  //initialize the MAX30102
#endif

  old_n_spo2=0.0;

  while(Serial.available()==0)  //wait until user presses a key
  {
    Serial.println(F("Press any key to start conversion"));
    delay(1000);
  }
  uch_dummy=Serial.read();
#ifdef TEST_MAXIM_ALGORITHM
  Serial.print(F("Time[s]\tSpO2\tHR\tSpO2_MX\tHR_MX\tClock\tRatio\tCorr"));
  SerialBT.print(F("Time[s]\tSpO2\tHR\tSpO2_MX\tHR_MX\tClock\tRatio\tCorr"));
#else // TEST_MAXIM_ALGORITHM
  Serial.print(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr"));
  SerialBT.print(F("Time[s]\tSpO2\tHR\tClock\tRatio\tCorr"));
#endif // TEST_MAXIM_ALGORITHM
#ifdef SAVE_RAW_DATA
  int32_t i;
  // These are headers for the red signal
  for(i=0;i<BUFFER_SIZE;++i) {
    Serial.print("\t");
    Serial.print(i);
  }
  // These are headers for the infrared signal
  for(i=0;i<BUFFER_SIZE;++i) {
    Serial.print("\t");
    Serial.print(i);
  }
#endif // SAVE_RAW_DATA
  Serial.println("");
  
  timeStart=millis();
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];

#ifdef USE_MAX_30105
  particleSensor.check(); //Check the sensor, read up to 3 samples
  while (1) {//do we have new data
    aun_red_buffer[i] = particleSensor.getRed();  //Sparkfun's MAX30105
    aun_ir_buffer[i] = particleSensor.getIR();  //Sparkfun's MAX30105
/*
    #ifdef DEBUG
    Serial.print(i, DEC);
    Serial.print(F("\t"));
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print(F("\t"));
    Serial.print(aun_ir_buffer[i], DEC);    
    Serial.println("");
    #endif // DEBUG
*/
    #ifdef DEBUG_FLEXIPLOT
    Serial.print("{P0|RED|255,0,0|");
    Serial.print(aun_red_buffer[i]);
    Serial.print("|IR|0,0,255|");
    Serial.print(aun_ir_buffer[i]);
    Serial.println("}");
    #endif
    i++;
    particleSensor.nextSample();
    if ((i % BUFFER_SIZE) == 0) { //every Num samples 
        i = 0;
        break;
    }
  }
#else     
  //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
  //read BUFFER_SIZE samples, and determine the signal range
  for(i=0;i<BUFFER_SIZE;i++)
  {
    //while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
    maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
#ifdef DEBUG
    Serial.print(i, DEC);
    Serial.print(F("\t"));
    Serial.print(aun_red_buffer[i], DEC);
    Serial.print(F("\t"));
    Serial.print(aun_ir_buffer[i], DEC);    
    Serial.println("");

    SerialBT.print(i, DEC);
    SerialBT.print(F("\t"));
    SerialBT.print(aun_red_buffer[i], DEC);
    SerialBT.print(F("\t"));
    SerialBT.print(aun_ir_buffer[i], DEC);    
    SerialBT.println("");
#endif // DEBUG
  }
#endif

  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
  rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
  elapsedTime=millis()-timeStart;
  millis_to_hours(elapsedTime,hr_str); // Time in hh:mm:ss format
  elapsedTime/=1000; // Time in seconds

#ifdef DEBUG
  Serial.println("--RF--");
  Serial.print(elapsedTime);
  Serial.print("\t");
  Serial.print(n_spo2);
  Serial.print("\t");
  Serial.print(n_heart_rate, DEC);
  Serial.print("\t");
  Serial.println(hr_str);
  Serial.println("------");
  
  SerialBT.println("--RF--");
  SerialBT.print(elapsedTime);
  SerialBT.print("\t");
  SerialBT.print(n_spo2);
  SerialBT.print("\t");
  SerialBT.print(n_heart_rate, DEC);
  SerialBT.print("\t");
  SerialBT.println(hr_str);
  SerialBT.println("------");
#endif // DEBUG

#ifdef TEST_MAXIM_ALGORITHM
  //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using MAXIM's method
  float n_spo2_maxim;  //SPO2 value
  int8_t ch_spo2_valid_maxim;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate_maxim; //heart rate value
  int8_t  ch_hr_valid_maxim;  //indicator to show if the heart rate calculation is valid
  maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2_maxim, &ch_spo2_valid_maxim, &n_heart_rate_maxim, &ch_hr_valid_maxim); 
#ifdef DEBUG
  Serial.println("--MX--");
  Serial.print(elapsedTime);
  Serial.print("\t");
  Serial.print(n_spo2_maxim);
  Serial.print("\t");
  Serial.print(n_heart_rate_maxim, DEC);
  Serial.print("\t");
  Serial.println(hr_str);
  Serial.println("------");
  
  SerialBT.println("--MX--");
  SerialBT.print(elapsedTime);
  SerialBT.print("\t");
  SerialBT.print(n_spo2_maxim);
  SerialBT.print("\t");
  SerialBT.print(n_heart_rate_maxim, DEC);
  SerialBT.print("\t");
  SerialBT.println(hr_str);
  SerialBT.println("------");
#endif // DEBUG
#endif // TEST_MAXIM_ALGORITHM

  //save samples and calculation result to SD card
#ifdef TEST_MAXIM_ALGORITHM
  if(ch_hr_valid && ch_spo2_valid || ch_hr_valid_maxim && ch_spo2_valid_maxim) {
#else   // TEST_MAXIM_ALGORITHM
  if(ch_hr_valid && ch_spo2_valid) { 
#endif // TEST_MAXIM_ALGORITHM

    Serial.print(elapsedTime);
    Serial.print("\t");
    Serial.print(n_spo2);
    Serial.print("\t");
    Serial.print(n_heart_rate, DEC);
    Serial.print("\t");
    
    SerialBT.print(elapsedTime);
    SerialBT.print("\t");
    SerialBT.print(n_spo2);
    SerialBT.print("\t");
    SerialBT.print(n_heart_rate, DEC);
    SerialBT.print("\t");
#ifdef TEST_MAXIM_ALGORITHM
    Serial.print(n_spo2_maxim);
    Serial.print("\t");
    Serial.print(n_heart_rate_maxim, DEC);
    Serial.print("\t");

    SerialBT.print(n_spo2_maxim);
    SerialBT.print("\t");
    SerialBT.print(n_heart_rate_maxim, DEC);
    SerialBT.print("\t");
#endif //TEST_MAXIM_ALGORITHM
    Serial.print(hr_str);
    Serial.print("\t");
    Serial.print(ratio);
    Serial.print("\t");
    Serial.print(correl);
    
    SerialBT.print(hr_str);
    SerialBT.print("\t");
    SerialBT.print(ratio);
    SerialBT.print("\t");
    SerialBT.print(correl);
#ifdef SAVE_RAW_DATA
    // Save raw data for unusual O2 levels
    for(i=0;i<BUFFER_SIZE;++i)
    {
      Serial.print(F("\t"));
      Serial.print(aun_red_buffer[i], DEC);
    }
    for(i=0;i<BUFFER_SIZE;++i)
    {
      Serial.print(F("\t"));
      Serial.print(aun_ir_buffer[i], DEC);    
    }
#endif // SAVE_RAW_DATA
    Serial.println("");
    SerialBT.println("");
    old_n_spo2=n_spo2;
  }
  delay(500);
}

void millis_to_hours(uint32_t ms, char* hr_str)
{
  char istr[6];
  uint32_t secs,mins,hrs;
  secs=ms/1000; // time in seconds
  mins=secs/60; // time in minutes
  secs-=60*mins; // leftover seconds
  hrs=mins/60; // time in hours
  mins-=60*hrs; // leftover minutes
  itoa(hrs,hr_str,10);
  strcat(hr_str,":");
  itoa(mins,istr,10);
  strcat(hr_str,istr);
  strcat(hr_str,":");
  itoa(secs,istr,10);
  strcat(hr_str,istr);
}
