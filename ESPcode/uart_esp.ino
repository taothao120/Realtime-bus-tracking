#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
#include "driver/gpio.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
// Provide the token generation process info.
#include "addons/TokenHelper.h"
// Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

#define TXD_PIN (GPIO_NUM_17)
#define RXD_PIN (GPIO_NUM_16)

// Insert your network credentials
#define WIFI_SSID "redmi1234"
#define WIFI_PASSWORD "123456789"

// Insert Firebase project API Key
#define API_KEY "AIzaSyBnUEYApFonmRwA6us3YVOLiO7dZsOSMhA"

// Insert RTDB URL
#define DATABASE_URL "https://gpsdata-9819f-default-rtdb.asia-southeast1.firebasedatabase.app/"

// Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

float longitude = 0.0, latitude = 0.0;
unsigned long sendDataPrevMillis = 0;
String timeg = "GP";
unsigned long t1 = 0;
int count = 0;
bool signupOK = false;

void setup() {
  Serial.begin(115200);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Assign the api key (required)
  config.api_key = API_KEY;

  // Assign the RTDB URL (required)
  config.database_url = DATABASE_URL;

  // Sign up
  if (Firebase.signUp(&config, &auth, "", "")) {
    Serial.println("Sign-up OK");
    signupOK = true;
  } else {
    Serial.printf("Sign-up Error: %s\n", config.signer.signupError.message.c_str());
  }

  // Assign the callback function for the long running token generation task
  config.token_status_callback = tokenStatusCallback; // see addons/TokenHelper.h

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

  const uart_port_t uart_num = UART_NUM_2;
  uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, // Disabled flow control if not needed
    .rx_flow_ctrl_thresh = 122
  };

  // Configure UART parameters
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)); // Set flow control pins to no change if not used
  const int uart_buffer_size = (1024 * 2);
  QueueHandle_t uart_queue;
  // Install UART driver using an event queue here
  ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 20, &uart_queue, 0));
}

void loop() {
  // Read data from UART
  const uart_port_t uart_num = UART_NUM_2;
  uint8_t data[128];
  int length = 0;

  ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
  length = uart_read_bytes(uart_num, data, length, 100);

  if (length > 0) {
    if (length < sizeof(data)) {
      data[length] = '\0'; // Null-terminate the data to make it a valid string
    } else {
      data[sizeof(data) - 1] = '\0'; // Ensure null-termination if buffer is full
    }

    char *test = (char*)data;

    // Use sscanf to parse the string
    if (sscanf(test, "%f %f", &latitude, &longitude) == 2) {
      Serial.print("Latitude: ");
      Serial.println(latitude);
      Serial.print("Longitude: ");
      Serial.println(longitude);
    } else {
      Serial.println("Error parsing string");
    }

    if (signupOK) {
      // Send data to Firebase
      if (!Firebase.RTDB.setFloat(&fbdo, "/GPS_Data/Latitude", latitude)) {
        Serial.print("Error sending latitude data: ");
        Serial.println(fbdo.errorReason());
      }

      if (!Firebase.RTDB.setFloat(&fbdo, "/GPS_Data/Longitude", longitude)) {
        Serial.print("Error sending longitude data: ");
        Serial.println(fbdo.errorReason());
      }
    }
  }

  // Add a short delay to prevent the loop from running too quickly
  delay(500);
}
