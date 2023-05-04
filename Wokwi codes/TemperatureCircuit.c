/*
Code rundown:
Reads temperature values from a thermistor connected to an analog input pin,
calculates the temperature in Celsius and Fahrenheit, and performs recalibration 
of the thermistor values using the highest and lowest temperature readings over a 
period of time.

-M, G y S
*/
#include <stdio.h>

int calibration_time = 0;
bool recalibrated = false;
float recalibratedVoltage = 0;
float Rt = 0;
double voltage = 0;
double tempC = 0;
float temperatureQuotient = 0;
float values_for_recalibration [5];

float recalibrate(float values[5]);
float calculateHighestTemperature(float values[5]);
float calculateLowestTemperature(float values[5]);

void setup() {
  Serial.begin(9600);
  // analogReadResolution(10);
  // pinMode(15,INPUT);
}

void loop() {
  //Temperature Sensor Initialization
  float adcValue = analogRead(15);
  float voltage = (float)adcValue / 4095.0 * 3.3;                  //read ADC pin and convert to voltage
  float Rt = 10 * voltage / (3.3 - voltage);                       //calculate resistance value of thermistor
  double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);  //calculate temperature (Kelvin)
  tempC = tempK - 273.15;                                   //calculate temperature (Celsius)
  double tempF = (tempC*9/5)+32;
  
  if (temperatureQuotient != 0){
  tempC = tempC*temperatureQuotient;
  }

  if(calibration_time < 5 && recalibrated == false){
    Serial.println("Recalibrating...");
    values_for_recalibration[calibration_time] = tempC;
    calibration_time++;
  }

  else if(calibration_time >= 5 ){
    temperatureQuotient = recalibrate(values_for_recalibration);
    recalibrated = false;
    calibration_time = 0;

    for (int i = 0; i < 5; i++){  //reset array
      values_for_recalibration[i] = 0;
    }
    delay(500);
  }
  Serial.printf("Calibration Time: %d,\tVoltage : %.2fV, \tTemperature in C: %.2fC,  \tTemperature in F: %.2fF\n", calibration_time, voltage, tempC, tempF);
  delay(1000);
}

float recalibrate(float values[]){
  for (unsigned int i = 0; i < 5; i++){
    Serial.printf("Value %d : %.2f\n", i, values[i]);
  }
  float lowestTemperature = calculateLowestTemperature(values);
  float highestTemperature = calculateHighestTemperature(values);
  float averageTemperature = (lowestTemperature + highestTemperature) / 2;
  
  //Linear interpolation to find the temperature of the thermistor
  float interTemp = lowestTemperature + ((5/2)-1) * (highestTemperature - lowestTemperature) / (5-1);
  Serial.printf("Interpolated Temperature : %.2f\n", interTemp);
  float temperatureQuotient = interTemp / averageTemperature;

  return temperatureQuotient;
 
  }

float calculateLowestTemperature(float values[]){
  float lowestTemperature = values[0];
  for (unsigned int i = 0; i < 5; i++){
    if (values[i] < lowestTemperature){
      lowestTemperature = values[i];
    }
  }
  return lowestTemperature;
}

float calculateHighestTemperature(float values[]){
  float highestTemperature = values[0];
  for (unsigned int i = 0; i < 5; i++){
    if (values[i] > highestTemperature){
      highestTemperature = values[i];
    }
  }
  return highestTemperature;
}
