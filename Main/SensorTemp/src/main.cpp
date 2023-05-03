#include <Arduino.h>
#include <wifi.h>
#include <PubSubClient.h>
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////TEMPERATURE SENSOR//////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Comment the codes accordingly depending on which functionality you want to use (TEMPERATURE SENSOR or PEOPLE INSIDE ROOM SENSORS)

// #define PIN_ANALOG_IN_TEMPERATURE 36
// #define PIN_ANALOG_IN_INTERRUPTOR 32

// //WIFI
// #define WIFI_NAME "Uncle Boobs" //The wifi name and password must be changed to match the compatible networks
// #define WIFI_PASSWORD "gabyromcor"
// #define WIFI_TIMEOUT 10000

// //MQTT BROKER
// const char *mqttServer = "15.156.41.54";
// const char *TempC = "Room204/tempC";
// const char *TempF = "Room204/tempF";
// const int mqttPort = 1883;


// WiFiClient espClient;
// PubSubClient client(espClient);

// //Temperature global variables
// int calibration_time = 0;
// bool recalibrated = false;
// float recalibratedVoltage = 0;
// float Rt = 0;
// double voltage = 0;
// double tempC = 0;
// float temperatureQuotient = 0;
// float values_for_recalibration [5];

// float recalibrate(float values[5]);
// float calculateHighestTemperature(float values[5]);
// float calculateLowestTemperature(float values[5]);

// void connectToWifi();

// void connectToWifi(){

//   Serial.println("Connecting to WiFi");
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(WIFI_NAME, WIFI_PASSWORD);

//   unsigned long startAttempTime = millis();

//   while (WiFi.status() != WL_CONNECTED && millis() - startAttempTime < WIFI_TIMEOUT)
//   {
//     Serial.print(".");
//     delay(100);
//   }

//   if (WiFi.status() != WL_CONNECTED)
//   {
//     Serial.println("Failed to connect to WiFi");
//     return;
//   }
//   else {
//     Serial.println("Connected to WiFi");
//   }
// }

// void connectToBroker(){
// client.setServer(mqttServer, mqttPort);
 
// while (!client.connected()) {
//     Serial.println("Connecting to MQTT...");

//     if (client.connect("ESP32Client")) {
//       Serial.println("connected");
//     } else {
//       Serial.print("failed with state ");
//       Serial.print(client.state());
//       delay(2000);
//     }
// }
// }

// void setup() {
//   Serial.begin(9600);
//   connectToWifi();
//   connectToBroker();
// }

// void loop() {
//   //Temperature Sensor Initialization
//   float adcValue = analogRead(PIN_ANALOG_IN_TEMPERATURE);
//   float voltage = (float)adcValue / 4095.0 * 3.3;                  //read ADC pin and convert to voltage
//   float Rt = 10 * voltage / (3.3 - voltage);                       //calculate resistance value of thermistor
//   double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) / 3950.0);  //calculate temperature (Kelvin)
//   tempC = tempK - 273.15;                                   //calculate temperature (Celsius)
//   double tempF = (tempC * 9 / 5) + 32;                         //Calculate temperature (Fahrenheit)
//   double button = analogRead(PIN_ANALOG_IN_INTERRUPTOR);
  
//   if (temperatureQuotient != 0){
//   tempC = tempC*temperatureQuotient;
//   }

//   if( button == 0 && calibration_time < 5 && recalibrated == false){
//     Serial.println("Recalibrating...");
//     values_for_recalibration[calibration_time] = tempC;
//     calibration_time++;
//   }

//   else if( button != 0 && calibration_time >= 5 ){
//     temperatureQuotient = recalibrate(values_for_recalibration);
//     recalibrated = false;
//     calibration_time = 0;

//     for (int i = 0; i < 5; i++){  //reset array
//       values_for_recalibration[i] = 0;
//     }
//     delay(500);
//   }
//   Serial.printf("Calibration Time: %d,\tVoltage : %.2fV, \tTemperature in C : %.2fC, \tTemperature in F : %.2fF\n", calibration_time, voltage, tempC, tempF);
//   delay(1000);

//   printf("Button: %.2f\n", button);

//   if (!client.connected())
//   {
//     connectToBroker();
//   }
//   client.publish(TempC, String(tempC).c_str());
//   client.publish(TempF, String(tempF).c_str());
//   client.loop();
// }

// float recalibrate(float values[]){
//   for (unsigned int i = 0; i < 5; i++){
//     Serial.printf("Value %d : %.2f\n", i, values[i]);
//   }
//   float lowestTemperature = calculateLowestTemperature(values);
//   float highestTemperature = calculateHighestTemperature(values);
//   float averageTemperature = (lowestTemperature + highestTemperature) / 2;
  
//   //Linear interpolation to find the temperature of the thermistor
//   float interTemp = lowestTemperature + ((5/2)-1) * (highestTemperature - lowestTemperature) / (5-1);
//   Serial.printf("Interpolated Temperature : %.2f\n", interTemp);
//   float temperatureQuotient = interTemp / averageTemperature;


//   return temperatureQuotient;
 
//   }


// float calculateLowestTemperature(float values[]){
//   float lowestTemperature = values[0];
//   for (unsigned int i = 0; i < 5; i++){
//     if (values[i] < lowestTemperature){
//       lowestTemperature = values[i];
//     }
//   }
//   return lowestTemperature;
// }

// float calculateHighestTemperature(float values[]){
//   float highestTemperature = values[0];
//   for (unsigned int i = 0; i < 5; i++){
//     if (values[i] > highestTemperature){
//       highestTemperature = values[i];
//     }
//   }
//   return highestTemperature;
// }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////PEOPLE INSIDE ROOM SENSORS////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <NewPing.h>

#define SONAR_TRIG_PIN_IN 26
#define SONAR_ECHO_PIN_IN 25
#define SONAR_TRIG_PIN_OUT 18
#define SONAR_ECHO_PIN_OUT 19
#define LED_WAIT_PIN 4
#define LED_ENTER_PIN 13

#define MAX_DISTANCE 200
#define MIN_DISTANCE 15
#define DEFAULT_DISTANCE 50

#define ITERATIONS 10
#define DELAY_BETWEEN_PINGS 40
#define DELAY_AFTER_CALIBRATION 1000

//WIFI
#define WIFI_NAME "Uncle Boobs" //The wifi name and password must be changed to match the compatible networks
#define WIFI_PASSWORD "gabyromcor"
#define WIFI_TIMEOUT 10000

//MQTT BROKER
const char *mqttServer = "15.156.41.54";
const char *topic = "Room205/peopleCount"; //Must be changed to corresponding Room that each ESP will be located and info must match 
const int mqttPort = 1883;


WiFiClient espClient;
PubSubClient client(espClient);

NewPing sonarIn(SONAR_TRIG_PIN_IN, SONAR_ECHO_PIN_IN, MAX_DISTANCE);
NewPing sonarOut(SONAR_TRIG_PIN_OUT, SONAR_ECHO_PIN_OUT, MAX_DISTANCE);

int people_count = 0;
int limit = 10;

int calibrateIn = 0;
int calibrateOut = 0;

bool prevInBlocked = false;
bool prevOutBlocked = false;

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
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
  connectToWifi();
  connectToBroker();

  pinMode(LED_WAIT_PIN, OUTPUT);
  pinMode(LED_ENTER_PIN, OUTPUT);

  digitalWrite(LED_WAIT_PIN, HIGH);
  digitalWrite(LED_ENTER_PIN, HIGH); // Both LEDs are lit to alert user to ongoing calibration.

  Serial.println("Calibrating...");
  delay(DELAY_AFTER_CALIBRATION);

  for (int a = 0; a < ITERATIONS; a++) {
    delay(50);
    calibrateIn += sonarIn.ping_cm();
    delay(50);
    calibrateOut += sonarOut.ping_cm();
    delay(200);
  }

  calibrateIn = 0.75 * calibrateIn / ITERATIONS; // The threshold is set at 75% of the average of these readings. This should prevent the system counting people if it is knocked.
  calibrateOut = 0.75 * calibrateOut / ITERATIONS;

  if (calibrateIn > MAX_DISTANCE || calibrateIn < MIN_DISTANCE) { // If the calibration gave a reading outside of sensible bounds, then the default is used
    calibrateIn = DEFAULT_DISTANCE;
  }
  if (calibrateOut > MAX_DISTANCE || calibrateOut < MIN_DISTANCE) {
    calibrateOut = DEFAULT_DISTANCE;
  }

  Serial.print("Entry threshold set to: ");
  Serial.println(calibrateIn);
  Serial.print("Exit threshold set to: ");
  Serial.println(calibrateOut);

  digitalWrite(LED_WAIT_PIN, LOW);
  digitalWrite(LED_ENTER_PIN, LOW); // Both LEDs are off to alert user that calibration has finished.

  delay(DELAY_AFTER_CALIBRATION);
}

void loop() {
  int distanceIn = sonarIn.ping_cm();
  delay(DELAY_BETWEEN_PINGS); // Wait 40 milliseconds between pings.
  int distanceOut = sonarOut.ping_cm();
  delay(DELAY_BETWEEN_PINGS);

  if (distanceIn < calibrateIn && distanceIn > 0) {
  // If closer than wall/calibrated object (person is present) && throw out zero readings
  if (!prevInBlocked) {
      people_count++;
    if (people_count < limit) {
      Serial.print("People inside room: ");
      Serial.println(people_count);
    } else if (people_count == limit) {
      Serial.print("Maximum capacity reached! ");
      Serial.print(people_count);
      Serial.println(" people inside.");
    } else if (people_count > limit) {
      Serial.print("Warning: Overcapacity! ");
      Serial.print(people_count);
      Serial.println(" people inside. ");
    }
  }
  prevInBlocked = true;
} else {
  prevInBlocked = false;
}
  if (distanceOut < calibrateOut && distanceOut > 0) {
    if (!prevOutBlocked && people_count > 0) {
      people_count--;
    if (people_count < limit) {
      Serial.print("People inside room: ");
      Serial.println(people_count);
    } else if (people_count == limit) {
      Serial.print("Maximum capacity reached! ");
      Serial.print(people_count);
      Serial.println(" people inside.");
    } else if (people_count > limit) {
      Serial.print("Warning: Overcapacity! ");
      Serial.print(people_count);
      Serial.println(" people inside. ");
    }
  }
    prevOutBlocked = true;
  } else {
    prevOutBlocked = false;
  }
  if (people_count > 0 && people_count < limit) { // If there are people present, then turn on the LED
    digitalWrite(LED_ENTER_PIN, HIGH);
  } else {
    digitalWrite(LED_ENTER_PIN, LOW);
  }
  if (people_count >= limit) { // If the number of people is greater than the limit, then turn on the LED
    digitalWrite(LED_WAIT_PIN, HIGH);
  } else {
    digitalWrite(LED_WAIT_PIN, LOW);
  }
  if (!client.connected())
  {
    connectToBroker();
  }
  client.publish(topic, String(people_count).c_str());
  client.loop();
}
