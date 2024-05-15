#include <Arduino.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <ArduinoMqttClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <ArduinoMqttClient.h>
#if defined(ARDUINO_SAMD_MKRWIFI1010) || defined(ARDUINO_SAMD_NANO_33_IOT) || defined(ARDUINO_AVR_UNO_WIFI_REV2)
#include <WiFiNINA.h>
#elif defined(ARDUINO_SAMD_MKR1000)
#include <WiFi101.h>
#elif defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_PORTENTA_H7_M7) || defined(ARDUINO_NICLA_VISION) || defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_GIGA) || defined(ARDUINO_OPTA)
#include <WiFi.h>
#elif defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif defined(ARDUINO_UNOR4_WIFI)
#include <WiFiS3.h>
#endif

#define PIN 8
#define NUMPIXELS      1 // Nombre de LEDs dans le bandeau
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_AHTX0 aht;

#define NUMFLAKES 10 // Number of snowflakes in the animation example

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16
static const unsigned char PROGMEM logo_bmp[] =
    {0b00000000, 0b11000000,
     0b00000001, 0b11000000,
     0b00000001, 0b11000000,
     0b00000011, 0b11100000,
     0b11110011, 0b11100000,
     0b11111110, 0b11111000,
     0b01111110, 0b11111111,
     0b00110011, 0b10011111,
     0b00011111, 0b11111100,
     0b00001101, 0b01110000,
     0b00011011, 0b10100000,
     0b00111111, 0b11100000,
     0b00111111, 0b11110000,
     0b01111100, 0b11110000,
     0b01110000, 0b01110000,
     0b00000000, 0b00110000};

const char *ssid = "Hesias";
const char *password = "bienvenuechezHesias";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

AsyncWebServer server(80);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const char broker[] = "172.20.63.156";
int port = 1883;
const char topic[] = "calixte/temperature";
const char topic2[] = "calixte/humidity";
const char topic_subs[] = "loris/temperature";
const char topic_subs2[] = "loris/co2";

const long interval = 1000;
unsigned long previousMillis = 0;

int count = 0;

void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show(); // Mise à jour de la couleur de la LED
}

void connectWifi() {
  WiFi.mode(WIFI_STA); // Optional
    WiFi.begin(ssid, password);
    Serial.println("\nConnecting");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(100);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

void connectMqqtBroker() {
    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);

    if (!mqttClient.connect(broker, port))
    {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        while (1)
            ;
    }

    Serial.println("You're connected to the MQTT broker!");
    Serial.println();

    Serial.print("Subscribing to topic: ");
    Serial.println(topic);
    Serial.println();

    // subscribe to a topic
    mqttClient.subscribe(topic_subs);
    mqttClient.subscribe(topic_subs2);

    // topics can be unsubscribed using:
    // mqttClient.unsubscribe(topic);

    Serial.print("Waiting for messages on topic: ");
    Serial.println(topic_subs);
    Serial.println(topic_subs2);
    Serial.println();

}

void printTemperatureToDisplay()
{
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    // print temperature to display
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.cp437(true);
    display.write("Temperature: ");
    display.print(temp.temperature);
    display.println(" C");
    // print humidity to display
    display.setCursor(0, 10);
    display.cp437(true);
    display.write("Humidity: ");
    display.print(humidity.relative_humidity);
    display.println(" rH");
    display.display();
    delay(1000);
}

void publishTemperatureToMqtt()
{
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    mqttClient.poll();

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis;

        // Envoi de la température
        Serial.print("Temperature: ");
        Serial.println(temp.temperature);
        mqttClient.beginMessage(topic);
        mqttClient.print(temp.temperature);
        mqttClient.endMessage();

        // Envoi de l'humidité
        Serial.print("Humidity: ");
        Serial.println(humidity.relative_humidity);
        Serial.println("");
        mqttClient.beginMessage(topic2);
        mqttClient.print(humidity.relative_humidity);
        mqttClient.endMessage();
    }
}

void receiveFromTopic() {
    int messageSize = mqttClient.parseMessage();
    if (messageSize)
    {
        Serial.print("Received a message with topic '");
        Serial.print(mqttClient.messageTopic());
        Serial.print("', length ");
        Serial.print(messageSize);
        Serial.println(" bytes:");

        char message[messageSize + 1];
        int index = 0;

        while (mqttClient.available())
        {
            message[index++] = (char)mqttClient.read();
        }
        message[index] = '\0';

        if (strcmp(mqttClient.messageTopic().c_str(), topic_subs) == 0) {
            Serial.print("Loris temperature ");
            Serial.print(message);
            Serial.println();
            setColor(255, 0, 0);
            delay(500);
            setColor(0, 0, 0);
        } else if (strcmp(mqttClient.messageTopic().c_str(), topic_subs2) == 0) {
            Serial.print("Loris co2 ");
            Serial.print(message);
            Serial.println();
            setColor(0, 255, 0);
            delay(500);
            setColor(0, 0, 0);
        }
    }
}


void launchWebServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        sensors_event_t humidity, temp;
        aht.getEvent(&humidity, &temp);
          String html = "<!DOCTYPE html><html><head><title>ESP32 Weather Station</title>"
              "<style>"
              "body { background-color: #DD491A; font-family: 'Arial', sans-serif; margin: 0; padding: 20px; color: #FFFFFF; }"
              ".container { background-color: #FFFFFF; padding: 20px; border-radius: 10px; color: #004080; margin: auto; width: 50%; box-shadow: 0 4px 8px 0 rgba(0,0,0,0.2); }"
              "h1 { color: #FFA500; font-size: 24px; }"
              "p { color: #004080; font-size: 18px; }"
              "p:hover { color: #FFA500; cursor: pointer; }"
              "@media (max-width: 600px) { .container { width: 90%; } }"
              "</style>"
              "</head><body>"
              "<div class='container'>"
              "<h1>Weather Station</h1>"
              "<p><b>Temperature:</b> " + String(temp.temperature) + " C</p>"
              "<p><b>Humidity: </b>" + String(humidity.relative_humidity) + " %</p>"
              "</div>"
              "</body></html>";
    request->send(200, "text/html", html);
    });

    server.begin();
}

void setup()
{
    pixels.begin(); 
    Serial.println("Adafruit AHT10/AHT20 demo!");

    Wire.setPins(3, 2);

    if (!aht.begin())
    {
        Serial.println("Could not find AHT? Check wiring");
        while (1)
            delay(10);
    }
    Serial.println("AHT10 found");

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

    display.display();
    delay(2000);

    Serial.begin(115200);
    delay(1000);

    connectWifi();
    connectMqqtBroker();
    launchWebServer();
    
}

void loop()
{
    printTemperatureToDisplay();
    publishTemperatureToMqtt();
    receiveFromTopic();

    
}

