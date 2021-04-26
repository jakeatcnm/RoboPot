/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/jake/Documents/IoT/RoboPot/RoboPot/src/RoboPot.ino"
/*
 * Project RoboPot
 * Description:  IoT Flower Pot
 * Author: Jake Millington
 * Date: 20-APR-2021
 */


#include <JsonParserGeneratorRK.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_BME280.h>

//#include <JsonParserGeneratorRK.h>

#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
 

#include <Grove_Air_quality_Sensor.h>
#include "credentials.h"

void setup();
void loop();
void MQTT_connect();
float convertToFarenheit(float celsius);
float convertToInHg( float pascals);
#line 25 "c:/Users/jake/Documents/IoT/RoboPot/RoboPot/src/RoboPot.ino"
float tempC = 0;
float tempF = 0;
float pressPA = 0;
float pressIH = 0;
float humidRH = 0;

unsigned long duration = 0;
unsigned long starttime = 0;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float dustConcentration = 0;
int airQuality = -1;
float moisture = 0;



String DateTime, TimeOnly;
char currentTime[9];

const int SCREEN_WIDTH = 128; // OLED display width, in pixels
const int SCREEN_HEIGHT = 64; // OLED display height, in pixels

const int PUMPPIN = D7;
const int MOISTUREPIN = A0;
const int DUSTPIN = A3;
const int AIRPIN = A2;
AirQualitySensor airQualitySensor(AIRPIN);

#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3C 
Adafruit_SSD1306 display(OLED_RESET);

Adafruit_BME280 bme;
int lastTime = 0;
/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe buttonSub = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/WaterButton"); 
Adafruit_MQTT_Publish humidityPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish dustConcentrationPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DustConcentration");
Adafruit_MQTT_Publish soilMoisturePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/SoilMoisture");
Adafruit_MQTT_Publish pressurePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure");
Adafruit_MQTT_Publish airQualityPub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");
Adafruit_MQTT_Publish temperaturePub = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
unsigned long last;
int value;
int publishTimer = 0;

int i;
int count = 0;
// Start in SEMI_AUTOMATIC Mode
SYSTEM_MODE(SEMI_AUTOMATIC);
int waterTimer = 0;
void setup() {
  Serial.begin(9600);

  pinMode(PUMPPIN, OUTPUT);
  pinMode(MOISTUREPIN, INPUT);
  pinMode(DUSTPIN, INPUT);
  pinMode(AIRPIN, INPUT);
  //setup I2C
  Wire.begin();
  for (i = 0; i <= 127; i++)
  {
    Wire.beginTransmission (i);
    //Serial.printf("Wire transmission end returned: %i \n",Wire.endTransmission());
    if (Wire.endTransmission () == 0)
      {
      Serial.printf("Found address: %03i (0x%02X) \n",i,i);  
      count++;
      delay (1);
      } 
  }
  Serial.printf("Done: Found %i device(s). \n",count);

  //setup Display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();
  display.setRotation(0);

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);
  display.println("Robopot\nPart Container\nPart Computer\nAll Pot");
  display.display();
  // Draw a single pixel in white

  delay(2000);
  Time.zone(-6);
  Particle.syncTime();
  delay(100); //wait for Serial Monitor to startup

  //pinMode(A0, INPUT_PULLDOWN);
 

  Serial.printf("Connecting to Internet \n");
  display.clearDisplay();
  display.setCursor(0,0);
  display.printf("Connecting to Infonet\n");
  display.display();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
    delay(100);
  }
  Serial.printf("\n Connected!!!!!! \n");
  display.printf("Connected!");
  display.display();
  // Setup MQTT subscription for onoff feed.
  //mqtt.subscribe(&TempF);
  mqtt.subscribe(&buttonSub);
  //airQualitySensor.init();
  bme.begin(0x76);

}


// loop() runs over and over again, as quickly as it can execute.
void loop() {
  MQTT_connect();

  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &buttonSub) {
      value = atoi((char *)buttonSub.lastread);
      Serial.printf("WaterButton value: %i\n",value);
    }
  }
  digitalWrite(PUMPPIN, value);

  moisture = analogRead(MOISTUREPIN);
  Serial.println(moisture);
  moisture = (moisture - 3400) * -1;
  Serial.println(moisture);
  int moisturePercent = moisture / 17;
  Serial.println(moisturePercent);

  if (moisturePercent < 20){
    digitalWrite(PUMPPIN, HIGH);
    delay(500);
    digitalWrite(PUMPPIN, LOW);
  }
    
  Serial.printf("Moisture: %i\n", moisturePercent);
  tempC = bme.readTemperature();
  tempF = convertToFarenheit(tempC);
  pressPA = bme.readPressure();
  pressIH = convertToInHg(pressPA);
  humidRH = bme.readHumidity();

  duration = pulseIn(DUSTPIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy+duration;
 
  if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
  {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
    Serial.printf("Ratio: %i", ratio);
    dustConcentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
    Serial.print(lowpulseoccupancy);
    Serial.print(",");
    Serial.print(ratio);
    Serial.print(",");
    Serial.println(dustConcentration);
    lowpulseoccupancy = 0;
    starttime = millis();
  }



  airQuality=airQualitySensor.slope();
  if (airQuality >= 0)// if a valid data returned.
  {
    if (airQuality==0)
      Serial.println("High pollution! Force signal active");
    else if (airQuality==1)
      Serial.println("High pollution!");
    else if (airQuality==2)
      Serial.println("Low pollution!");
    else if (airQuality ==3)
      Serial.println("Fresh air");   
  }
  display.clearDisplay();

  display.setRotation(0);

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setCursor(0,0);
  display.printf("Soil moisture: %i\n", moisturePercent);
  display.printf("Temperature: %.3f\n", tempF);
  display.printf("Humidity: %.3f\n", humidRH);
  display.printf("Pressure: %.3f\n", pressIH);
  display.printf("Air Quality: %i\n", airQuality);
  display.printf("Dust: %f", dustConcentration);
  display.display();

  if (millis() > publishTimer){
    if(mqtt.Update()) {
      humidityPub.publish(humidRH);
      dustConcentrationPub.publish(dustConcentration);
      soilMoisturePub.publish(moisturePercent);
      pressurePub.publish(pressIH);
      airQualityPub.publish(airQuality);
      temperaturePub.publish(tempF);
      publishTimer = millis() + 30000;
    } 
  }

}


// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

float convertToFarenheit(float celsius){
  float farenheit = (celsius * 1.8) + 32;
  return farenheit;
}

float convertToInHg( float pascals){
  float mercury = pascals / 3386.389;
  return mercury;
}