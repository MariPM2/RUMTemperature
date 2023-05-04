#RUM Temp
This code is a firmware for an ESP32 device that has both ultrasonic sensors and temperature sensors, as well as MQTT communication capabilities. It can count the number of people entering and exiting a room with a capacity limit of ten people while also measuring the temperature in Celsius and Fahrenheit and publishing the values to an MQTT broker.

**Components connected to the ESP32:**
- Ultrasonic Distance Sensors x2
- Thermistor
- Recalibration Button
- Resistors x3
- LEDs x2

**Methods and Functions for Temperature Sensor**
- `setup()` function: This function is executed only once when the device is powered on or reset. It initializes the serial communication, connects to Wi-Fi and MQTT Broker.
- `loop()` function: This function is executed continuously as long as the device is powered on. It reads the temperature and button sensor values, recalibrates the temperature sensor if the button is pressed for more than five times, publishes the temperature values to MQTT broker, and updates the calibration time, voltage, and temperature in Celsius and Fahrenheit in the serial monitor.
- `connectToWifi()` function: This function connects to the Wi-Fi network with the given name and password.
- `connectToBroker()` function: This function connects to the MQTT broker using the IP address and port number.
- `recalibrate(float values[5])` function: This function recalibrates the temperature sensor using the array of five temperature values obtained by pressing the button. It finds the lowest and highest temperatures, interpolates the temperature of the thermistor, and calculates the temperature quotient used for temperature correction.
- `calculateLowestTemperature(float values[5])` function: This function calculates the lowest temperature value from the given array of five temperature values.
- `calculateHighestTemperature(float values[5])` function: This function calculates the highest temperature value from the given array of five temperature values.

**Methods and Functions for Ultrasonic Distance Sensors**
- The `#include <NewPing.h>` directive is used to include the NewPing library, which provides functions for ultrasonic sensors.
- `#define` is used to define constants for the pins used by the ultrasonic sensors and LEDs, as well as the maximum, minimum, and default distances that will be used in calibration, the number of iterations and delays between pings that will be used in the main loop, the WiFi name and password, the timeout for connecting to WiFi, the MQTT broker's IP address, port, and topic.
- `WiFiClient espClient` creates a WiFi client that can be used to connect to a WiFi network.
- `PubSubClient client(espClient)` creates a PubSub client that can be used to connect to an MQTT broker.
- `NewPing sonarIn(SONAR_TRIG_PIN_IN, SONAR_ECHO_PIN_IN, MAX_DISTANCE)` creates a NewPing object for the ultrasonic sensor used to detect people entering the room.
- `NewPing sonarOut(SONAR_TRIG_PIN_OUT, SONAR_ECHO_PIN_OUT, MAX_DISTANCE)` creates a NewPing object for the ultrasonic sensor used to detect people leaving the room.
- `int people_count = 0` initializes the count of people in the room to 0.
- `int limit = 10` sets the maximum capacity of the room to 10 people.
- `int calibrateIn = 0` and `int calibrateOut = 0` initialize the variables used to store the calibration values for the ultrasonic sensors.
- `bool prevInBlocked = false` and `bool prevOutBlocked = false` initialize the variables used to track whether the ultrasonic sensors are currently detecting an object.
- `void connectToWifi()` defines a function that connects to a WiFi network.
- `void connectToBroker()` defines a function that connects to an MQTT broker.
- `void setup()` initializes the program by setting up the serial monitor, connecting to WiFi and the MQTT broker, setting the LED pins as outputs, calibrating the ultrasonic sensors, and turning off the calibration LEDs.
- `void loop()` is the main program loop that pings the ultrasonic sensors, counts people entering and leaving the room, and sends a message to the MQTT broker indicating the current number of people in the room.

To set up the device, the user must configure the WIFI name and password and MQTT broker address and topic by changing the corresponding constants in the code. The calibration distances and recalibration process can also be adjusted to suit the specific use case.

The code first sets up the pins for the sensors and LEDs, as well as the WiFi and MQTT configurations, in the `setup()` function. The `loop()` function is responsible for measuring the temperature, recalibrating the readings if the button is pressed, and counting people entering and exiting the room.

The temperature is measured by reading the analog value from the temperature sensor connected to the `PIN_ANALOG_IN_TEMPERATURE` pin. The analog value is then converted to voltage and used to calculate the resistance value of the thermistor. From the resistance value, the temperature in Celsius and Fahrenheit is calculated.

If the button connected to the `PIN_ANALOG_IN_INTERRUPTOR` pin is pressed, the device can recalibrate the temperature readings. The recalibration process takes five temperature readings and calculates the lowest and highest values. The firmware then linearly interpolates between the two values to find the temperature of the thermistor. The new temperature quotient is then calculated by dividing the interpolated temperature by the average temperature.

The main program logic for counting people entering and exiting the room happens in the `loop()` function as well. The distances measured by the two sensors are compared to pre-set calibration distances (`calibrateIn` and `calibrateOut`) that were determined during a calibration phase at the beginning of the program. If the distance measured by the sensor is less than the corresponding calibration distance, the code considers that as a person entering or exiting the room depending on the sensor being used and updates the `people_count` variable accordingly. If the `people_count` is less than the `limit` of ten, then the code publishes the `people_count` value to the corresponding MQTT topic.

The code also uses LEDs to indicate the calibration and alert the user if the room reaches its maximum capacity. The temperature readings in Celsius and Fahrenheit are published to the MQTT broker using the `PubSubClient` library. If the device loses the connection to the broker, the firmware will automatically try to reconnect.

The code includes comments explaining each part of the firmware and how it works, making it easy to modify and customize for different use cases.
