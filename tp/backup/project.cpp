#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <MQTT.h>
#include <FS.h>

// Remplacez ces informations par les vôtres
const char* ssid = "Hesias";
const char* password = "bienvenuechezHesias";
const char* mqtt_server = "172.20.63.156"; 

WiFiClient net;
MQTTClient client;

Adafruit_AHTX0 aht;

void callback(String &topic, String &payload) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(payload);

  if (topic == "esp32c3/scd40/co2") {
    Serial.print("Message from topic esp32c3/scd40/co2: ");
    Serial.println(payload);
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void connect() {
  Serial.print("Checking WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("\nConnecting to MQTT...");
  while (!client.connect("ESP32Client")) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nConnected to MQTT!");

  client.subscribe("esp32c3/scd40/co2");
  client.subscribe("jacquot/temperature");
}

void setup() {
  Serial.begin(115200);
  setup_wifi();
  
  client.begin(mqtt_server, 1883, net);
  client.onMessage(callback);

  Wire.begin(3, 2);

  if (!aht.begin(&Wire)) {
    Serial.println("Failed to find AHT sensor!");
    while (1) delay(10);
  }
  Serial.println("AHT10 or AHT20 found");

  connect();
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void loop() {
  client.loop();

  if (!client.connected()) {
    connect();
  }

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp); // Lisez les valeurs du capteur

  char tempStr[8];
  char humidityStr[8];
  dtostrf(temp.temperature, 1, 2, tempStr); // Convertir la température en chaîne de caractères
  dtostrf(humidity.relative_humidity, 1, 2, humidityStr); // Convertir l'humidité en chaîne de caractères
  Serial.print("Temperature: ");
  Serial.println(tempStr);
  Serial.print("humidity: ");
  Serial.println(humidityStr);
  client.publish("calixte/temperature", tempStr); 
  client.publish("calixte/humidity", humidityStr); 
  delay(2000);
}

