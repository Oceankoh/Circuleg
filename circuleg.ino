#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "BluetoothSerial.h"

#include "max30102.h"

// Interrupt pin
const byte oxiInt = 10;  // pin connected to MAX30102 INT

uint32_t elapsedTime, timeStart;

//uint32_t aun_ir_buffer[BUFFER_SIZE];   // infrared LED sensor data
//uint32_t aun_red_buffer[BUFFER_SIZE];  // red LED sensor data
float old_n_spo2;                      // Previous SPO2 value
uint8_t uch_dummy, k;

BluetoothSerial SerialBT;
#define buzz 19

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32test");
  Serial.println("Device name: ESP32test");
  pinMode(oxiInt, INPUT);  // pin D10 connects to the interrupt output pin of the MAX30102
  pinMode(buzz, OUTPUT);
  Wire.begin();
  maxim_max30102_reset();  // resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(
      REG_INTR_STATUS_1,
      &uch_dummy);        // Reads/clears the interrupt status register
  maxim_max30102_init();  // initialize the MAX30102

  old_n_spo2 = 0.0;
  uch_dummy = Serial.read();

  Serial.println("");
  timeStart = millis();
}



void loop(){
  uint32_t red_val, ir_val;
  uint32_t *red_ptr, *ir_ptr;
  red_ptr = &red_val;
  ir_ptr = &ir_val;
  uint32_t total = 0;
  for(int i=0;i<10;){
    maxim_max30102_read_fifo(red_ptr, ir_ptr);
    Serial.println(*ir_ptr);
//    SerialBT.println(*ir_ptr);
    if (*ir_ptr>7000){
      Serial.print(total);
      Serial.print(" ");
      Serial.println(*ir_ptr);
      total += *ir_ptr; 
      i++;
    }
    delay(500);
  }
  Serial.print("Average: ");
  Serial.println(total/10);
  SerialBT.print(total/10);
  if(total/10<8000){ //TODO Configure
    Serial.println("buzz");
//    SerialBT.println("buzz");
    digitalWrite(buzz, HIGH);
  }else{
    Serial.println("stop buzzing");
//    SerialBT.println("stop buzzing");
    digitalWrite(buzz, LOW);
  }
}

void millis_to_hours(uint32_t ms, char *hr_str) {
  char istr[6];
  uint32_t secs, mins, hrs;
  secs = ms / 1000;   // time in seconds
  mins = secs / 60;   // time in minutes
  secs -= 60 * mins;  // leftover seconds
  hrs = mins / 60;    // time in hours
  mins -= 60 * hrs;   // leftover minutes
  itoa(hrs, hr_str, 10);
  strcat(hr_str, ":");
  itoa(mins, istr, 10);
  strcat(hr_str, istr);
  strcat(hr_str, ":");
  itoa(secs, istr, 10);
  strcat(hr_str, istr);
}
