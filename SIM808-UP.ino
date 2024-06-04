#include <SoftwareSerial.h>
#include <DFRobot_SIM808.h>
#define TINY_GSM_MODEM_SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Constants
const char* BROKER = "0.tcp.eu.ngrok.io";
const int MQTT_PORT = 14965;
const char* TOPIC_GPS = "proj2.0/gps";
const char* AUTHORIZED_NUMBER = "0556110363";

// Objects
SoftwareSerial sim808Serial(10, 11);
DFRobot_SIM808 sim808(&sim808Serial);
TinyGsm modem(sim808Serial);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Variables
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 20000; // 20 seconds
const int maxGPSData = 10;
String gpsDataArray[maxGPSData];
int gpsDataCount = 0;

// Function prototypes
void setupSIM808();
void setupInternet();
boolean mqttConnect();
void publishGPSData();
void checkIncomingCall();
void sendATCommand(const char* cmd);
String GPSData();


//setup
void setup() {
  //init Serial communication
  Serial.begin(115200);
  sim808Serial.begin(9600);
  Serial.println("Initializing...");
  setupSIM808(); //init sim808 and attach gps
  setupInternet(); //configure gprs and setup connection
}


//loop
void loop() {
  // Publish GPS data at the specified interval
  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= publishInterval) {
    lastPublishTime = currentMillis;
    if (sim808.getGPS()) {
      if (gpsDataCount >= maxGPSData) { // If the list is full, remove the oldest data (at index 0)
        for (int i = 0; i < gpsDataCount - 1; i++) {
          gpsDataArray[i] = gpsDataArray[i + 1];
        }
        gpsDataCount--;
      }

      // Add new GPS data to the array
      gpsDataArray[gpsDataCount] = GPSData();
      gpsDataCount++;
    }
  
    //publish the data in the array if the connection established
    if (gpsDataCount > 0) {
      publishGPSData();
    }
  }
    
  // Check for incoming calls
  delay(2000);
  checkIncomingCall();
}

void setupSIM808() {
  //check if sim808 is on
  if (!sim808.init()) {
    Serial.println("SIM808 initialization failed");
    while (true); 
  }
  Serial.println("SIM808 initialized successfully");
  //check if sim808 attached gps
  if (!sim808.attachGPS()) {
    Serial.println("Failed to enable GPS");
    while (true); 
  }
  Serial.println("GPS enabled");
}

void setupInternet() {
  Serial.println("Configuring internet access...");

  sendATCommand("AT");
  sendATCommand("AT+CGATT=1"); //enable the GPRS functionality
  sendATCommand("AT+CSTT=\"internet\""); //set the apn ("internet" is ooredoo apn)
  sendATCommand("AT+CIICR");//establish the wireless link layer of the GPRS connection after setting the APN
  sendATCommand("AT+CIFSR");//request the ip address

  //test the connection
  Serial.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    Serial.println(" fail");
    while (true);
  }
  Serial.println(" OK");

  delay(3000);

  //specify the mqtt broker and port
  mqtt.setServer(BROKER, MQTT_PORT);
}


//check if connection to broker is established
boolean mqttConnect() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.print(BROKER);
  if (!mqtt.connect("client")) {
    Serial.println(" Connection failed");
    return false;
  }
  Serial.println("Connected to MQTT broker");
  return mqtt.connected();
}



String GPSData() {
  //Get the 3D data and speed
  float lat = sim808.GPSdata.lat;
  float lon = sim808.GPSdata.lon;
  float speed = sim808.GPSdata.speed_kph;
  float heading =sim808.GPSdata.heading;
  String timestamp = String(sim808.GPSdata.day) + "/" +
                     String(sim808.GPSdata.month) + "/" +
                     String(sim808.GPSdata.year) + " " +
                     String(sim808.GPSdata.hour) + ":" +
                     String(sim808.GPSdata.minute) + ":" +
                     String(sim808.GPSdata.second);


  //print to serial monitor to check 
  Serial.print("Timestamp: ");
  Serial.println(timestamp);
  Serial.print("Latitude: ");
  Serial.println(lat, 6);
  Serial.print("Longitude: ");
  Serial.println(lon, 6);
  Serial.print("Speed (kph): ");
  Serial.println(speed);
  Serial.print("Heading: ");
  Serial.println(heading);

  return String(lat, 6) + "," + String(lon, 6) + " " + timestamp + " " + String(speed, 6) + " " + String(heading, 6);
}

void publishGPSData() {
  if (!mqtt.connected()) {
    //if not connection lost then reconnect
    if (mqttConnect()) {
      if (gpsDataCount > 0) {
        //if there is data to publis or we are out of interval
        String message = gpsDataArray[0];
        mqtt.publish(TOPIC_GPS, message.c_str());
        for (int i = 0; i < gpsDataCount - 1; i++) {
          gpsDataArray[i] = gpsDataArray[i + 1];
        }
        gpsDataCount--;
      }
    } else {
      Serial.println("MQTT connection failed!");
    }
  } else {
    //device still connected to mqtt broker
    if (gpsDataCount > 0) {
      String message = gpsDataArray[0];
      mqtt.publish(TOPIC_GPS, message.c_str());
      for (int i = 0; i < gpsDataCount - 1; i++) {
        gpsDataArray[i] = gpsDataArray[i + 1];
      }
      gpsDataCount--;
    }
  }
}

  //spying
void checkIncomingCall() {
  sim808Serial.println("AT+CLCC");
  delay(1000);
  while (sim808Serial.available()) {
    String response = sim808Serial.readStringUntil('\n');
    Serial.println(response);
    if (response.indexOf(AUTHORIZED_NUMBER) != -1) {
      sim808.answer();
    }
  }
}

void sendATCommand(const char* cmd) {
  sim808Serial.println(cmd);
  delay(1000);
  while (sim808Serial.available()) {
    Serial.println(sim808Serial.readStringUntil('\n'));
  }
}
