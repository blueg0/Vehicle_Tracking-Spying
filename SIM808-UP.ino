#include <SoftwareSerial.h>
#include <DFRobot_SIM808.h>
#define TINY_GSM_MODEM_SIM808
#include <TinyGsmClient.h>
#include <PubSubClient.h>

// Constants
const char BROKER[] = "5.tcp.eu.ngrok.io";
const int MQTT_PORT = 18430;
const char TOPIC_GPS[] = "proj2.0/gps";
const char AUTHORIZED_NUMBER[] = "XXXX";
const int maxGPSData = 10; // Define maximum GPS data storage

// Objects
SoftwareSerial sim808Serial(10, 11);
DFRobot_SIM808 sim808(&sim808Serial);
TinyGsm modem(sim808Serial);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

// Variables
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 20000; // 20 seconds
String gpsDataArray[maxGPSData];
int gpsDataCount = 0;

// Function prototypes
void setupSIM808();
void setupInternet();
boolean mqttConnect();
void storeGPSData();
void publishGPSData();
void checkIncomingCall();
void sendATCommand(const char* cmd);

//setup
void setup() {
  //init Serial communication
  Serial.begin(115200);
  sim808Serial.begin(9600);
  Serial.println("Initializing...");
  setupSIM808();    //init sim808 and attach gps
  setupInternet();  //configure gprs and setup connection
}


//loop
void loop() {

  //Publish GPS data at the specified interval
  unsigned long currentMillis = millis();


  if (currentMillis - lastPublishTime >= publishInterval) {
    lastPublishTime = currentMillis;
    Serial.println(F("Checking GPS data..."));
    while (true) {
      if (sim808.getGPS()) {
        break;
      }
    }


    Serial.println(F("GPS data obtained"));
    storeGPSData();
  }
    //publish the data in the array if the connection established
  if (gpsDataCount > 0) {
    publishGPSData();
  }

  delay(1500);
  // Check for incoming calls
  checkIncomingCall();  
  }



void setupSIM808() {
  if (!sim808.init()) {
    Serial.println(F("SIM808 initialization failed"));
    while (true); // Stop further execution
  }
  Serial.println(F("SIM808 initialized successfully"));

  if (!sim808.attachGPS()) {
    Serial.println(F("Failed to enable GPS"));
    while (true); // Stop further execution
  }
  Serial.println(F("GPS enabled"));
}

void setupInternet() {  
  Serial.println(F("Configuring internet access..."));
  sendATCommand("AT");
  sendATCommand("AT+CGATT=1");            //enable the GPRS functionality
  sendATCommand("AT+CSTT=\"internet\"");  //set the apn ("internet" is ooredoo apn)
  sendATCommand("AT+CIICR");              //establish the wireless link layer of the GPRS connection after setting the APN
  sendATCommand("AT+CIFSR");              //request the ip address

  //test the connection
   Serial.print(F("Waiting for network..."));
  if (!modem.waitForNetwork()) {
    Serial.println(F(" fail"));
    while (true);
  }
  Serial.println(F(" OK"));

  delay(3000);

  //specify the mqtt broker and port
  mqtt.setServer(BROKER, MQTT_PORT);
}


//check if connection to broker is established
boolean mqttConnect() {
  Serial.print(F("Connecting to MQTT broker: "));
  Serial.print(BROKER);

  if (!mqtt.connect("client")) {
    Serial.println(F(" Connection failed"));
    return false;
  }

  Serial.println(F("Connected to MQTT broker"));
  return mqtt.connected();
}



void storeGPSData() {
  if (gpsDataCount >= maxGPSData) {
    for (int i = 0; i < maxGPSData - 1; i++) {
      gpsDataArray[i] = gpsDataArray[i + 1]; // Shift data to remove the oldest
    }
    gpsDataCount--;
  }

  float lat = sim808.GPSdata.lat;
  float lon = -sim808.GPSdata.lon;
  float speed = sim808.GPSdata.speed_kph;
  float heading = sim808.GPSdata.heading;
  char timestamp[25];
  snprintf(timestamp, sizeof(timestamp), "%02d/%02d/%04d %02d:%02d:%02d", 
           sim808.GPSdata.day, sim808.GPSdata.month, sim808.GPSdata.year, 
           sim808.GPSdata.hour, sim808.GPSdata.minute, sim808.GPSdata.second);

  Serial.print(F("Timestamp: ")); Serial.println(timestamp);
  Serial.print(F("Latitude: ")); Serial.println(lat, 6);
  Serial.print(F("Longitude: ")); Serial.println(lon, 6);
  Serial.print(F("Speed (kph): ")); Serial.println(speed);
  Serial.print(F("Heading: ")); Serial.println(heading);

  char message[100];
  snprintf(message, sizeof(message), "%.6f,%.6f %s %.6f %.6f", lat, lon, timestamp, speed, heading);

  gpsDataArray[gpsDataCount] = message;
  gpsDataCount++;
}



void publishGPSData() {
  if (!mqtt.connected() && !mqttConnect()) {
    Serial.println("MQTT connection failed!");
    return;
  }else {
      mqtt.publish(TOPIC_GPS, gpsDataArray[0].c_str());
    for (int i = 0; i < gpsDataCount - 1; i++) {
      gpsDataArray[i] = gpsDataArray[i + 1];
    }
    gpsDataCount--;
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
