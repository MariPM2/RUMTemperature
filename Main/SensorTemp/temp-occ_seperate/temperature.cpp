#include <Arduino.h>
#include <wifi.h>
#include <PubSubClient.h>

#define PIN_ANALOG_IN_TEMPERATURE 36
#define PIN_ANALOG_IN_INTERRUPTOR 32

//WIFI
#define WIFI_NAME "Uncle Boobs" //The wifi name and password must be changed to match the compatible networks
#define WIFI_PASSWORD "gabyromcor"
#define WIFI_TIMEOUT 10000

//MQTT BROKER
const char *mqttServer = "15.156.41.54";
const char *topic = "Room204/temperature"; //Must be changed to corresponding Room that each ESP will be located and info must match 
const int mqttPort = 1883;


WiFiClient espClient;
PubSubClient client(espClient);

//Temperature global variables
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

void connectToWifi();

void connectToWifi(){

  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

  unsigned long startAttempTime = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempTime < WIFI_TIMEOUT)
  {
    Serial.print(".");
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Failed to connect to WiFi");
    return;
  }
  else {
    Serial.println("Connected to WiFi");
  }
}

void connectToBroker(){
client.setServer(mqttServer, mqttPort);
 
while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client")) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
}
}

void setup() {
  Serial.begin(9600);
  connectToWifi();
  connectToBroker();
}

void loop() {
  //Temperature Sensor Initialization
  float adcValue = analogRead(PIN_ANALOG_IN_TEMPERATURE);
  float voltage = (float)adcValue / 4095.0 * 3.3;                  //read ADC pin and convert to voltage
  float Rt = 10 * voltage / (3.3 - voltage);                       //calculate resistance value of thermistor
  double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);  //calculate temperature (Kelvin)
  tempC = tempK - 273.15;                                   //calculate temperature (Celsius)
  double button = analogRead(PIN_ANALOG_IN_INTERRUPTOR);
  
  if (temperatureQuotient != 0){
  tempC = tempC*temperatureQuotient;
  }

  if( button == 0 && calibration_time < 5 && recalibrated == false){
    Serial.println("Recalibrating...");
    values_for_recalibration[calibration_time] = tempC;
    calibration_time++;
  }

  else if( button != 0 && calibration_time >= 5 ){
    temperatureQuotient = recalibrate(values_for_recalibration);
    recalibrated = false;
    calibration_time = 0;

    for (int i = 0; i < 5; i++){  //reset array
      values_for_recalibration[i] = 0;
    }
    delay(500);
  }
  Serial.printf("Calibration Time: %d,\tVoltage : %.2fV, \tTemperature : %.2fC\n", calibration_time, voltage, tempC);
  delay(1000);

  printf("Button: %.2f\n", button);

  if (!client.connected())
  {
    connectToBroker();
  }
  client.publish(topic, String(tempC).c_str());
  client.loop();
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