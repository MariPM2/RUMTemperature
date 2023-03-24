#include <LiquidCrystal.h>
int pirSensor1 = 2; // PIR Sensor 1 connected to Digital Pin 2
int pirSensor2 = 3; // PIR Sensor 2 connected to Digital Pin 3
int count = 0;// Variable to store the count of people in the room
int sensor = A0;
float temp;
float tempf;
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  pinMode(pirSensor1, INPUT); // Set PIR Sensor 1 as input
  pinMode(pirSensor2, INPUT); // Set PIR Sensor 2 as input
  lcd.begin(16, 2); // Initialize the LCD display
}

void loop() {
  if (digitalRead(pirSensor1) == HIGH) { // Check if someone enters the room
    count++; // Increment the count by 1
    delay(1000); // Wait for 1 second to avoid multiple counts
  }
  if (digitalRead(pirSensor2) == HIGH) { // Check if someone exits the room
    count--; // Decrement the count by 1
    delay(1000); // Wait for 1 second to avoid multiple counts
  }
  	temp=analogRead(sensor);
  	tempf=(tempc*1.8)+32;
	lcd.setCursor(0,0);
	lcd.print("Temp in F = ");
	lcd.println(tempf);
  	lcd.setCursor(0, 1);
  	lcd.print("People in Room: ");
  	lcd.print(count);
  	delay(1000); // Update the count once per second
}
