#include "RF.h"

Eloquent::ML::Port::RandomForest clf;

// Pin Config ARDUINO//
// SD card              (CS=10,SCK=13,Mosi=11,Miso=12)
// RTC     0x50 && 0x68 (SDA=A4,SCL=A5,Others=0)
// GY91    0x76         (SDA=A4,SCL=A5,Others=0)

// Pin Config ESP32-VSPI//clai
// SD card              (CS=G2,SCK=G18,Mosi=G23,Miso=G19)
// RTC     0x50 && 0x68 (SDA=G21,SCL=G22,Others=0)
// GY91    0x76         (SDA=G21,SCL=G22,Others=0)

// Libraries for Use MQTT
#include <WiFi.h>
#include <ESPmDNS.h>
#include <EEPROM.h>
#include <PubSubClient.h>

// Libraries for sliding window
#include <RunningAverage.h>

// Libraries for math
#include <math.h>

// WiFi var
const char* HOSTNAME = "EspDuino32";
const char *ssid = "ANTARES";
const char *password = "xxx";

// MQTT var
//const char* MQTT_SERVER = "test.mosquitto.org"; //diganti https://test.mosquitto.org/ 
const char* MQTT_SERVER = "broker.emqx.io";
const char* topic = "ANTARES/DataTest";

// RTC
#include <Wire.h>
#include "RTClib.h"
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
RTC_DATA_ATTR int readingID = 0;

String dataMessage1; String dataMessage0;
String formattedDate; String dayStamp; String timeStamp; // Date and time variables

WiFiClient client;
PubSubClient mqtt(client);

// MPU9250 && BMP
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bme; // I2C
float altitude, displacement;
MPU9250_asukiaaa mySensor;
float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
float calibrates, factor, temperature, pressure;
float aZ_present, aZ_average, aZ_previous;
float vZ_present, vZ_average, vZ_previous;
float sZ_present, sZ_all, dZ, vZ;
float aX_present, aX_average, aX_previous;
float vX_present, vX_average, vX_previous;
float sX_present, sX_all, dX, vX;
float aY_present, aY_average, aY_previous;
float vY_present, vY_average, vY_previous;
float sY_present, sY_all, dY, vY;

// variable for Labelling
//float label; 
float sensorData[25]; // Sensor data
int result; // result
float interval=0.5;

// Timeout var
unsigned long timeout = 0;  
unsigned long wifi_timeout = 0;
unsigned long prevMillis = 0;
unsigned long prevReconnMillis = 0;

// running average var
RunningAverage aXRA(5);
RunningAverage aYRA(5);
RunningAverage aZRA(5);
RunningAverage aSRA(5);
RunningAverage gXRA(5);
RunningAverage gYRA(5);
RunningAverage gZRA(5);
RunningAverage mDRA(5);
RunningAverage mXRA(5);
RunningAverage mYRA(5);
RunningAverage mZRA(5);

RunningAverage aXRS(5);
RunningAverage aYRS(5);
RunningAverage aZRS(5);
RunningAverage aSRS(5);
RunningAverage gXRS(5);
RunningAverage gYRS(5);
RunningAverage gZRS(5);
RunningAverage mDRS(5);
RunningAverage mXRS(5);
RunningAverage mYRS(5);
RunningAverage mZRS(5);

RunningAverage label(5);

// shift var
int count_shift = 0;

void setup() {
 // Start serial communication for debugging purposes
  Serial.begin(115200);
  bme.begin(); //bmp initialization
  //MPU9250 Acceleration
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  delay(3000); // wait for console opening
 
  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
       // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    //rtc.adjust(DateTime(2021, 1, 13, 16, 1, 1));
  }

  connectToWiFi();
  mqtt.setServer(MQTT_SERVER, 1883);

  //running average
  aXRA.clear(); // explicitly start clean
  aYRA.clear();
  aZRA.clear();
  aSRA.clear();
  gXRA.clear();
  gYRA.clear();
  gZRA.clear();
  mXRA.clear();
  mYRA.clear();
  mZRA.clear();
  mDRA.clear();
  aXRS.clear(); 
  aYRS.clear();
  aZRS.clear();
  aSRS.clear();
  gXRS.clear();
  gYRS.clear();
  gZRS.clear();
  mXRS.clear();
  mYRS.clear();
  mZRS.clear();
  mDRS.clear();
  label.clear();
}


void loop() {
  unsigned long currentMillis = millis();
  
  if(!mqtt.connected())
  {
      // Reconnect WiFi Connection
      if((WiFi.status() != WL_CONNECTED) && (currentMillis - wifi_timeout >= 3000))
      {
        wifi_timeout = currentMillis;
        Serial.println("Connection Lost! Reconnecting...");
        WiFi.disconnect();
        WiFi.reconnect();  
      }
      else{
        reconnect();
      } 
  }
  else{
    mqtt.loop();
  }

  //digitalWrite(trigPin, LOW);
  delayMicroseconds(5);

  // Trigger the sensor by setting the trigPin high for 10 microseconds:
  //digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  //digitalWrite(trigPin, LOW);
  
// delay(500); // wait for console opening
    if (mySensor.accelUpdate() == 0) {
    aX = (mySensor.accelX()-15.93);
    aY = (mySensor.accelY()-0.23);
    aZ = (mySensor.accelZ())+0.03;
    aSqrt = mySensor.accelSqrt();
    Serial.println("accelX: " + String(aX));
    Serial.println("accelY: " + String(aY));
    Serial.println("accelZ: " + String(aZ));
    Serial.println("taccelSqrt\t: " + String(aSqrt)); 

    //running average
    aXRA.addValue(aX);
    aYRA.addValue(aY);
    aZRA.addValue(aZ);
    aSRA.addValue(aSqrt);

    aXRS.addValue(aX);
    aYRS.addValue(aY);
    aZRS.addValue(aZ);
    aSRS.addValue(aSqrt);
  }

  if (mySensor.gyroUpdate() == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    Serial.println("gyroX: " + String(gX));
    Serial.println("gyroY: " + String(gY));
    Serial.println("gyroZ: " + String(gZ));

    //running average
    gXRA.addValue(gX);
    gYRA.addValue(gY);
    gZRA.addValue(gZ);

    gXRS.addValue(gX);
    gYRS.addValue(gY);
    gZRS.addValue(gZ);
  }  

   if (mySensor.magUpdate() == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    Serial.println("magX: " + String(mX));
    Serial.println("maxY: " + String(mY));
    Serial.println("magZ: " + String(mZ));
    Serial.println("horizontalDirection: " + String(mDirection));

    //running average
    mXRA.addValue(mX);
    mYRA.addValue(mY);
    mZRA.addValue(mZ);
    mDRA.addValue(mDirection);

    mXRS.addValue(mX);
    mYRS.addValue(mY);
    mZRS.addValue(mZ);
    mDRS.addValue(mDirection);
  }
  
  
  delayMicroseconds(5);

  if (count_shift == 3) {
    // SVM Logic
    sensorData[0] = aXRA.getAverage();
    sensorData[1] = aSRA.getAverage();//resultan
    sensorData[2] = gXRA.getAverage();
    sensorData[3] = gYRA.getAverage();  
    sensorData[4] = gZRA.getAverage();
    sensorData[5] = aZRS.getStandardDeviation();//aZ
    sensorData[6] = aSRS.getStandardDeviation();//resultan
    sensorData[7] = gXRS.getStandardDeviation();
    sensorData[8] = gYRS.getStandardDeviation();  
    sensorData[9] = gZRS.getStandardDeviation();
    
    result = clf.predict(sensorData);

    // Labelling
    if (result == 1) {
      Serial.print("Label: ");
      Serial.println("1");
    }
    else {
      Serial.print("Label: ");
      Serial.println("0");
    };

    //running average label
    label.addValue(result);
  
    logSDCard();

    count_shift = 0;
  };

  //increment shift count
  count_shift++;
  
  // Increment readingID on every new reading
  readingID++;
  
  delay(500);
}

void connectToWiFi()
{
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while(WiFi.waitForConnectResult() != WL_CONNECTED) 
    {
      if(wifi_timeout >= 5) 
      {
        ESP.restart();
      }
      else wifi_timeout++;
      
      Serial.println("Connection Failed! Reconnecting...");
      WiFi.disconnect();
      WiFi.reconnect();
      delay(3000);
    }

    wifi_timeout = 0;

    if (!MDNS.begin(HOSTNAME)) 
    {
      Serial.println("Error setting up MDNS responder!");
      
      while (1) delay(1000);
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect(){
  while (!mqtt.connected()) {
    Serial.println("Connecting to MQTT...");
 
    if (mqtt.connect("ESP32Client", HOSTNAME, "")) 
    {
      Serial.println("connected");  
    } 
    else 
    {
      Serial.print("failed with state ");
      Serial.print(mqtt.state());
      delay(2000);
    }
}
}

///    EDIT    ///
void logSDCard() {
  DateTime now = rtc.now();
  dataMessage1 = String(readingID) + "," + String(daysOfTheWeek[now.dayOfTheWeek()]) + "," + String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + "," + 
                String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "," + 
                String(sensorData[0], 4) + "," + String(sensorData[1], 4) + "," + String(sensorData[2], 4) + "," + String(sensorData[3], 4) + "," + String(sensorData[4], 4) + "," + String(sensorData[5], 4) + "," + String(sensorData[6], 4) + "," + String(sensorData[7], 4) + "," + String(sensorData[8], 4) + "," + String(sensorData[9], 4) + "," + String(sensorData[10], 4) + "," +
                String (result) + "," + "1" +   "\r\n";
  dataMessage0 = String(readingID) + "," + String(daysOfTheWeek[now.dayOfTheWeek()]) + "," + String(now.year()) + "/" + String(now.month()) + "/" + String(now.day()) + "," + 
                String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second()) + "," + 
                String(sensorData[0], 4) + "," + String(sensorData[1], 4) + "," + String(sensorData[2], 4) + "," + String(sensorData[3], 4) + "," + String(sensorData[4], 4) + "," + String(sensorData[5], 4) + "," + String(sensorData[6], 4) + "," + String(sensorData[7], 4) + "," + String(sensorData[8], 4) + "," + String(sensorData[9], 4) + "," + String(sensorData[10], 4) + "," +
                String(result) + "," + "0" +   "\r\n";

                
  if (label.getAverage() > 0.3) {
    //Serial.print("Save data: ");
    Serial.println(dataMessage1);
    //appendFile(SD, "/prob.txt", dataMessage1.c_str());
    int len = dataMessage1.length()+1;
    char _data1[len];
    dataMessage1.toCharArray(_data1, len);
    mqtt.publish(topic, _data1);
  }
  else 
  {
    //Serial.print("Save data: ");
    Serial.println(dataMessage0);
    //appendFile(SD, "/prob.txt", dataMessage0.c_str());
    int len = dataMessage0.length()+1;
    char _data0[len];
    dataMessage0.toCharArray(_data0, len);
    mqtt.publish(topic, _data0);
  };
  //Serial.print("Save data: ");
  //Serial.println(dataMessage);
  //appendFile(SD, "/prob.txt", dataMessage.c_str());
}
