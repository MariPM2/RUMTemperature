# Is this A/C on?

Focused on monitoring the temperature and occupancy of several classrooms across the University of Puerto Rico - Mayaguez Campus. The solution relies on ESP32 microcontrollers equipped with temperature sensors and ultrasonic distance sensors, which continuously send measurements to a Node-RED server via the MQTT protocol. The server is hosted on an AWS Lightsail Cloud Server, and users can access the data either through Apple's Siri or through a web interface.

To set up the system, users must first download the PlatformIO extension for Visual Studio Code. They should then create a new project and specify that the board is an Espressif ESP-Wrover Kit. To communicate with the MQTT server, users must download the following libraries: 

- Wrover Kit library
- PubSub library
- NewPing library

Once the project is built, users can download it to their ESP32 microcontrollers.

To use Siri, users must install the Shortcuts app on their Apple device and use the following shortcut: https://www.icloud.com/shortcuts/e62923f2b6aa4a5686fe005c311e17ee. They can then activate the Siri command by saying "Siri, Room Status". Siri will ask the user for the specific room, in which case the user should say "Room XYZ" (where XYZ is the room number).

Alternatively, users can access the temperature and occupancy data by connecting to the following website: http://rumtemp.space:1880/ui (assuming the server is running). 

To implement this solution, we used the following tools and technologies: 

- VS Code IDE
- PlatformIO extension
- Espressif Wrover library
- Arduino library
- WiFi library
- PubSub Client library
- NewPing library
- AWS LightSail Cloud Server
- Node-RED
- Whois domain webservice

This IoT solution provides an efficient way to monitor the temperature and occupancy of several rooms in the University of Mayaguez Campus. The data is easily accessible via Siri or a web interface, which allows users to quickly identify rooms that need adjustment in their temperature settings and maximum capacity.
