#include <Arduino.h>
#include <wifi.h>
#include <PubSubClient.h>
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
const char *topic = "Room204/peopleCount"; //Must be changed to corresponding Room that each ESP will be located and info must match 
const int mqttPort = 1883;


WiFiClient espClient;
PubSubClient client(espClient);

NewPing sonarIn(SONAR_TRIG_PIN_IN, SONAR_ECHO_PIN_IN, MAX_DISTANCE);
NewPing sonarOut(SONAR_TRIG_PIN_OUT, SONAR_ECHO_PIN_OUT, MAX_DISTANCE);

int people_count = 0;
int limit = 30;

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
