#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

/*
#define mqtt_server "192.168.3.2"
#define mqtt_user "iotdevice"
#define mqtt_password "ransalcharu"

#define door_topic "doors/garage-door"
#define relay_topic "doors/relay"

const char* ssid = "Not your WiFi";
const char* password = "ransalcharu";
WiFiClient espClient;
PubSubClient client(espClient);

*/

void callback(char* topic, byte* payload, unsigned int length)
{
  payload[length] = '\0';
  String strTopic = String((char*) topic);
  Serial.println("Callback Triggered");
  if (strTopic == relay_topic)
  {
    String switch1 = String((char*) payload);
    if (switch1 == "on")
    {
      Serial.println("TOGGLED");
      digitalWrite(4, LOW);
      delay(600);
      digitalWrite(4, HIGH);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("esp8266-garage");

  // No authentication by default
  ArduinoOTA.setPassword((const char *)"ransalcharu");

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Ready 1");

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  pinMode(5, INPUT_PULLUP);
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH);
  //pinMode(6, OUTPUT);
}

int prevState = -1;
int currState = -1;
long lastChangeTime = 0;


void reconnect() {
  while (!client.connected())
  {
    if (client.connect("Garage-Sensor", mqtt_user, mqtt_password))
    {
      Serial.println("Connected to MQTT Server");
      client.subscribe("doors/relay");
    }
    else
    {
      Serial.print("Connection to MQTT Server failed. Reason: ");
      Serial.print(client.state());
      delay(5000);
    }
  }
}

void checkPin()
{
  // Invert state, since button is "Active LOW"
  int state = !digitalRead(5);
  //Serial.println(state);

  // Debounce mechanism
  long t = millis();
  if (state != prevState && millis() - lastChangeTime > 50) {
    lastChangeTime = t;
    Serial.println(state);
    if (!state)
        client.publish(door_topic, "Opened", true);
      else client.publish(door_topic, "Closed", true);
  }
  
  prevState = state;

}




void loop()
{
  ArduinoOTA.handle();
  if (!client.connected())
  {
    Serial.println("Reconnecting");
    reconnect();
  }
  client.loop();
  checkPin();
}

