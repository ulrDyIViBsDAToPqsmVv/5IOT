/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <Arduino.h>
#include "WiFi.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define PIN 8
#define NUMPIXELS      1 // Nombre de LEDs dans le bandeau
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

#define NUMFLAKES 10 // Number of snowflakes in the animation example

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

//Variable for MAC address of receiving messages
char formattedMAC[18];

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


// AHTX0 sensor
Adafruit_AHTX0 aht;

// MAC Adress of my receiver
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure example to send data
// Must match the receiver structure
// Structure for incoming message
typedef struct struct_message {
    float temperature;
    float humidity;
} struct_message;

// Create a struct_message called myData
// myDate = Incoming message
struct_message myData;

// Structure example to send data
// Must match the receiver structure
// Structure for outgoing message
typedef struct struct2_message {
    float temperature_out; 
    float humidity_out;

} structi2_message;

// Create a struct_message called myData_out
// myData_out = outgoing message
structi2_message myData_out;

typedef struct struct3_message {
    float uint8_t red, 
    float uint8_t green,
    float uint8_t blue,

} struct3_message;

// Create a struct_message called myData_out
// myData_led = LED outgoing message
struct3_message myData_led;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
Serial.println(" ");
}

void setColor(uint8_t red, uint8_t green, uint8_t blue) {
  pixels.setPixelColor(0, pixels.Color(red, green, blue));
  pixels.show(); // Mise à jour de la couleur de la LED
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.print(len);
  // Display MAC Address
  Serial.println("MAC : ");
  for (int i = 0; i < 6; i++) {
  if (mac[i] < 0x10) {
    Serial.print("0");
  }
  Serial.print(mac[i], HEX);
  if (i < 5) {
    Serial.print(":");
   }
  }
  Serial.println(" ");

 // MAC address formatting 
  int index = 0;
  for (int i = 0; i < 6; i++) {
    if (mac[i] < 0x10) {
      formattedMAC[index++] = '0';
    }
    sprintf((char*)&formattedMAC[index], "%X", mac[i]);
    index += 2;
    if (i < 5) {
      formattedMAC[index++] = ':';
    }
  }
  formattedMAC[index] = '\0'; // Ajouter le caractère de fin de chaîne


  Serial.print("temperature: ");
  Serial.println(myData.temperature);
  Serial.print("humidity : ");
  Serial.println(myData.humidity);
  Serial.println();
}

void setup() {
  pixels.begin()
  // Init Serial Monitor
  Serial.begin(115200);
  // Get MAC Address  
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  // Wire setup
  Wire.setPins(3, 2);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Check if AHT10 sensor is found
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
   }
  Serial.println("AHT10 or AHT20 found"); 

  // Check if Screen SSD1306 is okay
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;)
            ;
    }

  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
}
 // Fonction a tester
void printTemperatureToDisplay()
{
    //Check sender MAC ADDRESS
    WiFi.mode(WIFI_MODE_STA);
    // print MAC to display
    display.clearDisplay();
    display.setCursor(0, 0);
    display.cp437(true);
    display.write("MAC:");
    display.print(WiFi.macAddress());
    display.display();
    display.setTextSize(1);
    // print MAC to display
    display.setCursor(0, 10);
    display.cp437(true);
    display.write("F:");
    display.print(formattedMAC);
    display.setTextSize(1);
    // print temperature to display
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 25);
    display.cp437(true);
    display.write("T:");
    display.print(myData.temperature);
    display.println(" C");
    // print humidity to display
    display.setCursor(64, 25);
    display.cp437(true);
    display.write("H:");
    display.print(myData.humidity);
    display.println(" rH");
    display.display();
}
//

void message_sender(){
delay(2000);
  // Update temperature & humidity value
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);

 // Assignation des valeurs aux variables de la structure myData_out
  myData_out.temperature_out = temp.temperature;
  myData_out.humidity_out = humidity.relative_humidity;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData_out, sizeof(myData_out));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  Serial.print("my_temperature: ");
  Serial.println(myData_out.temperature_out);
  Serial.print("my_humidity : ");
  Serial.println(myData_out.humidity_out);
  Serial.println();
  }
  else {
    Serial.println("Error sending the data");
  }
}


void loop() {
    message_sender();
    printTemperatureToDisplay();
}
