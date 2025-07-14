// ESP8266 Relay Controller – No Authentication Version

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESPAsyncTCP.h>
#define WEBSERVER_H
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

#define NUM_RELAYS 4
const int relayPins[NUM_RELAYS] = {5, 4, 14, 12};    // D1, D2, D5, D6 - All safe GPIO pins for outputs
const int buttonPins[NUM_RELAYS] = {13, 16, 0, 2};   // D7, D0, D3, D4 - Safe for inputs with pullups
const int resetPin = 10;                             // SD3 - Reset button (pull low to reset)
#define LED_PIN 15                                   // D8 - Safe pin for NeoPixel LED strip
#define NUM_LEDS 3

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char* CONFIG_FILE = "/config.json";
const char* RELAY_FILE = "/relay_state.json";
bool relayState[NUM_RELAYS];

void setLEDStatus(bool boot, bool wifi, bool op) {
  strip.setPixelColor(0, boot ? strip.Color(255,165,0) : 0);   // Orange
  strip.setPixelColor(1, wifi ? strip.Color(0,255,0) : 0);     // Green
  strip.setPixelColor(2, op ? strip.Color(0,0,255) : strip.Color(255,0,0)); // Blue/Red
  strip.show();
}

void saveRelayState() {
  StaticJsonDocument<256> doc;
  for(int i = 0; i < NUM_RELAYS; i++) doc["relay" + String(i)] = relayState[i];
  File f = LittleFS.open(RELAY_FILE, "w");
  if (f) { serializeJson(doc, f); f.close(); }
}

void loadRelayState() {
  if (!LittleFS.exists(RELAY_FILE)) return;
  File f = LittleFS.open(RELAY_FILE, "r");
  StaticJsonDocument<256> doc;
  if (!deserializeJson(doc, f)) {
    for(int i = 0; i < NUM_RELAYS; i++) {
      relayState[i] = doc["relay" + String(i)] | false;
      digitalWrite(relayPins[i], relayState[i]);
    }
  }
  f.close();
}

void setRelay(int i, bool state) {
  relayState[i] = state;
  digitalWrite(relayPins[i], state);
  saveRelayState();
}

void notifyClients() {
  StaticJsonDocument<256> doc;
  for(int i = 0; i < NUM_RELAYS; i++) doc["relay" + String(i)] = relayState[i];
  String msg; serializeJson(doc, msg);
  Serial.println("Sending WebSocket update: " + msg);
  ws.textAll(msg);
  Serial.println("Active WebSocket clients: " + String(ws.count()));
}

void handleButtons() {
  static unsigned long lastCheck = 0;
  static bool lastButtonState[NUM_RELAYS] = {HIGH, HIGH, HIGH, HIGH};
  static unsigned long lastDebounceTime[NUM_RELAYS] = {0, 0, 0, 0};
  static bool buttonPressed[NUM_RELAYS] = {false, false, false, false};
  const unsigned long debounceDelay = 50;  // 50ms debounce
  
  if (millis() - lastCheck > 10) {  // Check every 10ms for better responsiveness
    lastCheck = millis();
    
    for(int i = 0; i < NUM_RELAYS; i++) {
      bool currentState = digitalRead(buttonPins[i]);
      
      // Debug output for troubleshooting
      if (currentState != lastButtonState[i]) {
        Serial.printf("Button %d state changed: %s (pin %d)\n", 
                     i + 1, currentState ? "HIGH" : "LOW", buttonPins[i]);
      }
      
      // If button state changed, reset debounce timer
      if (currentState != lastButtonState[i]) {
        lastDebounceTime[i] = millis();
        buttonPressed[i] = false;  // Reset pressed flag on state change
      }
      
      // If button has been stable for debounce period
      if ((millis() - lastDebounceTime[i]) > debounceDelay) {
        // Button is pressed (LOW) and we haven't processed this press yet
        if (currentState == LOW && !buttonPressed[i]) {
          buttonPressed[i] = true;  // Mark as processed
          setRelay(i, !relayState[i]);
          Serial.printf("Button %d pressed - Relay %s (State: %s)\n", 
                       i + 1, relayState[i] ? "ON" : "OFF", 
                       relayState[i] ? "HIGH" : "LOW");
          notifyClients();
        }
        // Reset pressed flag when button is released
        else if (currentState == HIGH && buttonPressed[i]) {
          buttonPressed[i] = false;
        }
      }
      
      lastButtonState[i] = currentState;
    }
  }
}

void resetAll() {
  LittleFS.remove(RELAY_FILE);
  WiFiManager wm;
  wm.resetSettings();
  delay(500);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  pinMode(resetPin, INPUT_PULLUP);

  strip.begin(); strip.show();
  setLEDStatus(true, false, false);

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return; // Stop or handle error here
  }
  Serial.println("LittleFS mounted");
  for (int i = 0; i < NUM_RELAYS; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
    pinMode(buttonPins[i], INPUT_PULLUP);
    Serial.printf("Configured Relay %d: Pin %d (Output), Button %d: Pin %d (Input with pullup)\n", 
                  i + 1, relayPins[i], i + 1, buttonPins[i]);
    
    // Test initial button state
    bool initialState = digitalRead(buttonPins[i]);
    Serial.printf("Initial button %d state: %s\n", i + 1, initialState ? "HIGH" : "LOW");
  }

  if (digitalRead(resetPin) == LOW) resetAll();

  WiFiManager wm;
  wm.setTimeout(180);
  if (!wm.autoConnect("RelaySetup")) {
    Serial.println("Failed to connect, AP mode active");
  }

  loadRelayState();
  setLEDStatus(false, true, true);
  ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t *data, size_t len){
    switch(type) {
      case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        // Send current state to new client
        {
          StaticJsonDocument<256> doc;
          for(int i = 0; i < NUM_RELAYS; i++) doc["relay" + String(i)] = relayState[i];
          String msg; serializeJson(doc, msg);
          client->text(msg);
        }
        break;
      case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
      case WS_EVT_DATA:
        {
          String msg = String((char*)data).substring(0, len);
          Serial.println("WebSocket received: " + msg);
          if (msg.startsWith("toggle:")) {
            int idx = msg.substring(7).toInt();
            if (idx >= 0 && idx < NUM_RELAYS) {
              setRelay(idx, !relayState[idx]);
              Serial.printf("WebSocket toggle relay %d: %s\n", idx + 1, relayState[idx] ? "ON" : "OFF");
              notifyClients();
            }
          }
        }
        break;
      case WS_EVT_ERROR:
        Serial.printf("WebSocket client #%u error(%u): %s\n", client->id(), *((uint16_t*)arg), (char*)data);
        break;
    }
  });
  server.addHandler(&ws);
  server.on("/api/relay", HTTP_GET, [](AsyncWebServerRequest *req){
    StaticJsonDocument<256> doc;
    for(int i = 0; i < NUM_RELAYS; i++) doc["relay" + String(i)] = relayState[i];
    String res; serializeJson(doc, res);
    req->send(200, "application/json", res);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req){
    StaticJsonDocument<512> doc;
    
    // Device info
    doc["device"]["chipId"] = String(ESP.getChipId(), HEX);
    doc["device"]["freeHeap"] = ESP.getFreeHeap();
    doc["device"]["uptime"] = millis();
    doc["device"]["resetReason"] = ESP.getResetReason();
    
    // WiFi info
    doc["wifi"]["ssid"] = WiFi.SSID();
    doc["wifi"]["rssi"] = WiFi.RSSI();
    doc["wifi"]["ip"] = WiFi.localIP().toString();
    doc["wifi"]["mac"] = WiFi.macAddress();
    doc["wifi"]["connected"] = WiFi.status() == WL_CONNECTED;
    
    // WebSocket info
    doc["websocket"]["clients"] = ws.count();
    
    // Relay states
    for(int i = 0; i < NUM_RELAYS; i++) {
      doc["relays"]["relay" + String(i)] = relayState[i];
    }
    
    String res; 
    serializeJson(doc, res);
    req->send(200, "application/json", res);
  });

  server.on("/api/relay", HTTP_POST, [](AsyncWebServerRequest *req){
    if(req->hasParam("relay", true) && req->hasParam("state", true)) {
      int idx = req->getParam("relay", true)->value().toInt();
      bool st = req->getParam("state", true)->value() == "true";
      if (idx >= 0 && idx < NUM_RELAYS) setRelay(idx, st);
      notifyClients();
      req->send(200, "text/plain", "OK");
    } else req->send(400, "Bad Request");
  });

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *req){
    req->send(200, "text/plain", "Rebooting...");
    delay(200);
    resetAll();
  });

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
}

void loop() {
  static unsigned long lastLEDUpdate = 0;
  
  // Clean up disconnected WebSocket clients
  ws.cleanupClients();
  
  handleButtons();
  
  // Update LED status every 500ms instead of every loop
  if (millis() - lastLEDUpdate > 500) {
    lastLEDUpdate = millis();
    setLEDStatus(false, WiFi.status() == WL_CONNECTED, true);
  }
  
  delay(5);  // Reduced delay for better button responsiveness
}
