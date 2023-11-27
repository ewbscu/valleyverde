#include "SparkFun_Si7021_Breakout_Library.h"
#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"

#define MINTEMP 70
#define MAXTEMP 80
#define SECOND 1000
#define MINUTE 60000

// Pin assignments
const int power = A3;
const int GND = A2;
const int relay = 2;

// Variable for readings
float humidity = 0;
float tempf = 0;

// Heater state
bool heat = false;

// Create Instance SI7021 sensor
Weather sensor;


char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
char server[] = HOST_URL;    // name address for Google (using DNS)

unsigned int localPort = 2390;      // local port to listen for UDP packets
IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[ NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets
// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
WiFiSSLClient client;


void setup()
{
    Serial.begin(9600);   // open serial over USB at 9600 baud

    pinMode(power, OUTPUT);
    pinMode(GND, OUTPUT);
    pinMode(relay, OUTPUT);

    digitalWrite(power, HIGH);
    digitalWrite(GND, LOW);
    //Initialize serial and wait for port to open:
    Serial.begin(9600);

    //Initialize the I2C sensors and ping them
    sensor.begin();
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }
    setupWifi();

}

int i=0;

void loop()
{
    // Read temp and humidity
    getWeather();
    printInfo();

    // Heating logic
    if (tempf < MINTEMP) {
      heat = true;
      digitalWrite(relay, HIGH);
    } else if (tempf > MAXTEMP) {
      heat = false;
      digitalWrite(relay, LOW);
    }

    // Send info to website
    // *Code Here*

    // Delay one second while heater is on, else one minute
    if (heat) {
      if(i%10 == 0) {
        Serial.println("UPDATING DATA");
        sendData(tempf, humidity, heat);
      }
      i++;
      delay(SECOND);
    } else {
      sendData(tempf, humidity, heat);
      delay(MINUTE);
    }
}
//---------------------------------------------------------------
void getWeather()
{
  // Measure relative humidity
  humidity = sensor.getRH();

  // Measure temperature
  tempf = sensor.getTempF();
  // Temperature is measured every time RH is requested.
  // It is faster, therefore, to read it from previous RH
  // measurement with getTemp() instead with readTemp()
}
//---------------------------------------------------------------
void printInfo()
{
//This function prints the weather data out to the default Serial Port

  Serial.print("Temp:");
  Serial.print(tempf);
  Serial.print("F, ");

  Serial.print("Humidity:");
  Serial.print(humidity);
  Serial.println("%");
}


void setupWifi() {
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
    Serial.println(WIFI_FIRMWARE_LATEST_VERSION);
  }
  status = 0;
  int tries = 0;
  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED && tries < 5) {
//    Serial.print("Attempting to connect to SSID: ");
//    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(3000);
    Serial.print(".");
    tries++;
  }
  Serial.println("");
  Serial.println("Connected to wifi");
  printWiFiStatus();
  Udp.begin(localPort);
}

void sendData(float temp, float humidity, bool heater) {
  Serial.print("WIFI STATUS: ");
   Serial.println(WiFi.status());
  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting...");
    setupWifi();
  }
  sendNTPpacket(timeServer);
  delay(1000);
  if (Udp.parsePacket()) {
    Serial.println("packet received");
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    Serial.print("Seconds since Jan 1 1900 = ");
    Serial.println(secsSince1900);
    // now convert NTP time into everyday time:
    Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    Serial.println(epoch);
    Serial.println("\nStarting connection to server...");
    // if you get a connection, report back via serial:
    if (client.connect(server, 443)) {
      Serial.println("connected to server");
      String payload = "{ \"heater\": " + String(heater) + ", \"humidity\":" + String(humidity) + ",\"temp\": " + String(temp) + ", \"time\": " + String(epoch) + " }";
      Serial.println("Request:");
      // Make a HTTP request:
      String request = "PUT /.json HTTP/1.1\r\n"
                   "Host: " + String(server) + "\r\n"
                   "Content-Type: application/json\r\n"
                   "Content-Length: " + payload.length() + "\r\n"
                   "Connection: close\r\n"
                   "\r\n" + payload;
      Serial.println(request);
      client.println(request);
  
    }
    delay(2000);
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
    }
    // if the server's disconnected, stop the client:
    if (!client.connected()) {
      Serial.println();
      Serial.println("disconnecting from server.");
      client.stop();
    }
  }
}

// send an NTP request to the time server at the given address
unsigned long sendNTPpacket(IPAddress& address) {
  Serial.println("Getting time");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Serial.println("Got Time");
  Udp.endPacket();
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
