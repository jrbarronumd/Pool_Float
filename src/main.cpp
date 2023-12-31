// Resources: 
// MQTT https://github.com/plapointe6/EspMQTTClient
// ADS1115: https://github.com/adafruit/Adafruit_ADS1X15
// DS18B20: https://randomnerdtutorials.com/esp32-ds18b20-temperature-arduino-ide/
// millis() Rollover: https://www.norwegiancreations.com/2018/10/arduino-tutorial-avoiding-the-overflow-issue-when-using-millis-and-micros/
// pH temp compensation: https://www.hach.com/asset-get.download.jsa?id=17525673904
// deep sleep????  RTC pull-up resistor for charge pump to shut down in deep sleep!!!
// Need to add a software switch to keep the ESP out of sleep mode in case of desired OTA software update.
// TODO:
	// For PCB:
		// WHY does the TP4056 not work???
		// Add 3.3V, 5V, and GND header pins to the J-tag pins (or somewhere else on the top face)
		// ESP antenna should not be surrounded by interference...
		// Change pinout of orp/pH connector so each 3-pin connector can be directly plugged in

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "EspMQTTClient.h"
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h> // Not having this caused build to fail - dependency of ADS1X15 library?

#define poolTempPin 23
#define chgPumpPin 17 // Shutdown pin on the 5V charge pump
#define phPin 18 // To power the sensor board for readings
#define orpPin 19 // To power the sensor board for readings

EspMQTTClient client(
	"Barron_NoT", // Wifi SSD
	"qwer asdf",  // Wifi PW
	"10.0.0.5",  // MQTT Broker server ip
	"mqttuser",   // Can be omitted if not needed
	"Q044Uc0OWy",   // Can be omitted if not needed
	"Pool_Float",     // Client name that uniquely identify your device
	1883              // The MQTT port, default to 1883. this line can be omitted
);

// Constants based on calibration:
float phSlope = -0.00107411; // in pH/bit - WILL NEED TO CHANGE IF ADS GAIN CHANGES!!
float phIntercept = 15.76451632; // WILL NEED TO CHANGE IF ADS GAIN CHANGES!!
float calTemp = 22.56; // pH calibration temp in deg C (temp of pH buffer used to generate phSlope)

// Variables
float orp = 0.0; // Oxidation reduction potential - used to estimate chlorine concentration
float pH = 0.0; // pH level
int phBits = 0.0; // Initial pH reading used in calcs
float tempF = 999.0; // Temp in deg F
float tempC = 555.0; // Temp in deg C
unsigned long previousMillis = 0; // Used to handle timers and millis roll-over
unsigned long interval = 10000; // Interval for checking sensors

Adafruit_ADS1115 ads;
OneWire oneWire(poolTempPin);
DallasTemperature sensors(&oneWire);

void onConnectionEstablished() {
	// This function is called once everything is connected (Wifi and MQTT)
	// WARNING : YOU MUST IMPLEMENT IT IF YOU USE EspMQTTClient
	client.publish("poolFloat/status", "online", true);
}

void getPH() {
	// digitalWrite(phPin, HIGH); // This will power the pH probe amp board.
	// delay(2000);
// ********************Need While loop here to make sure values aren't changing.***************************
	phBits = ads.readADC_SingleEnded(1);
	float phSlopeComp = phSlope * (tempC + 273.15) / (calTemp + 273.15); // Temp compensation (Source listed above)
	pH = phSlopeComp * phBits + phIntercept;
	// digitalWrite(phPin, LOW);
	// delay(500);
}

void getORP() {
	// digitalWrite(orpPin, HIGH); // This will power the ORP probe amp board
	// delay(2000);
// ********************Need While loop here to make sure values aren't changing.***************************
	orp = ads.readADC_SingleEnded(0); // USE COMPARISON TO GROUND INSTEAD?
	orp = orp * 0.1875; // converting bits to mV
	// digitalWrite(orpPin, LOW);
	// delay(500);
}

void setup() {
	Serial.begin(115200);
	pinMode(phPin, OUTPUT);
	pinMode(orpPin, OUTPUT);
	pinMode(chgPumpPin, OUTPUT);
	digitalWrite(phPin, HIGH);
	digitalWrite(orpPin, HIGH);
	digitalWrite(chgPumpPin, LOW);
	Serial.println("ADC Range: +/- 6.144V (1 bit = 0.1875mV)");
	ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 0.1875mV (default)
	// ads.setGain(GAIN_ONE); // 1x gain   +/- 4.096V  1 bit = 0.125mV (ADS1115)
	ads.begin(); // Start ADC (ADS1115)
	sensors.begin(); // Start DS18B20 sensor

	// ************Optional functionalities of EspMQTTClient: 
	client.enableHTTPWebUpdater(); // Enable the web updater. User and password default to values of MQTTUsername and MQTTPassword. These can be overwritten with enableHTTPWebUpdater("user", "password").
	client.enableLastWillMessage("poolFloat/status", "offline", true);  // You can activate the retain flag by setting the third parameter to true
	client.setKeepAlive(600); // (in seconds) Default is 15 seconds.
	client.enableDebuggingMessages(); // Enable debugging messages sent to serial output
}


void loop() {
	client.loop();
	// Check sensors on interval (will handle millis rollover)
	if ((unsigned long)(millis() - previousMillis) > interval) {
		previousMillis = millis();
		sensors.requestTemperatures();
		tempF = sensors.getTempFByIndex(0);
		tempC = sensors.getTempCByIndex(0);
		getPH();
		getORP();
		String msg = "";
		msg = String(msg + tempF + " F -- " + tempC + " C -- " + phBits + " bits -- pH = " + pH + " --  ORP = " + orp + " mV");
		Serial.println(msg);
		client.publish("poolFloat/data", msg, true);
	}
}