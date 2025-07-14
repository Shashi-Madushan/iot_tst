// ESP8266 Relay Controller – No Authentication Version

#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESPAsyncTCP.h>
#define WEBSERVER_H
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>  // Add MQTT support
#include <TimeLib.h>      // For scheduling

// File paths
const char* CONFIG_FILE = "/config.json";
const char* RELAY_FILE = "/relay_state.json";

// Time sync settings
const unsigned long TIME_UPDATE_INTERVAL = 3600000;  // Sync time every hour
const unsigned long SCHEDULE_CHECK_INTERVAL = 60000; // Check schedule every minute
const unsigned long LED_UPDATE_INTERVAL = 500;      // Update LED every 500ms
const unsigned long MQTT_RECONNECT_INTERVAL = 5000; // MQTT reconnect every 5s
const unsigned long BUTTON_CHECK_INTERVAL = 10;     // Check buttons every 10ms

// Global timing variables
unsigned long lastTimeUpdate = 0;
unsigned long lastScheduleCheck = 0;
unsigned long lastLEDUpdate = 0;
unsigned long lastMQTTReconnectAttempt = 0;
unsigned long lastButtonCheck = 0;

// Existing relay definitions
#define NUM_RELAYS 4
const int relayPins[NUM_RELAYS] = {5, 4, 14, 12};    // D1, D2, D5, D6
const int buttonPins[NUM_RELAYS] = {13, 16, 0, 2};   // D7, D0, D3, D4
const int resetPin = 10;                             // SD3
#define LED_PIN 15                                   // D8
#define NUM_LEDS 3

// Schedule structure
struct Schedule {
  uint8_t hour;
  uint8_t minute;
  bool daysOfWeek[7];  // Sun-Sat
  bool state;          // ON/OFF
  bool enabled;
};

// New structures for relay configuration
struct RelayConfig {
  String name;
  String description;
  bool scheduled;
  Schedule schedules[5];  // Up to 5 schedules per relay
  int numSchedules;
};

struct MQTTConfig {
  bool enabled;
  String server;
  int port;
  String username;
  String password;
  String topic_prefix;
};

// Improved button debounce structure
struct ButtonState {
    bool lastState;
    bool pressed;
    unsigned long lastDebounceTime;
};

// Global variables
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
WiFiClient espClient;
PubSubClient mqtt(espClient);

RelayConfig relayConfigs[NUM_RELAYS];
MQTTConfig mqttConfig;
bool relayState[NUM_RELAYS];

// Add button states array
ButtonState buttonStates[NUM_RELAYS];

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
  if (i < 0 || i >= NUM_RELAYS) return;
  
  relayState[i] = state;
  digitalWrite(relayPins[i], state);
  saveRelayState();
  
  // Publish to MQTT if enabled
  publishRelayState(i);
}

void notifyClients() {
    if (ws.count() == 0) return;  // Skip if no clients connected
    
    StaticJsonDocument<1024> doc;
    JsonObject relays = doc.createNestedObject("relays");
    for(int i = 0; i < NUM_RELAYS; i++) {
        JsonObject relay = relays.createNestedObject(String(i));
        relay["state"] = relayState[i];
        relay["name"] = relayConfigs[i].name;
        relay["description"] = relayConfigs[i].description;
        relay["scheduled"] = relayConfigs[i].scheduled;
        relay["numSchedules"] = relayConfigs[i].numSchedules;
        
        if (relayConfigs[i].scheduled && relayConfigs[i].numSchedules > 0) {
            JsonArray schedules = relay.createNestedArray("schedules");
            for (int j = 0; j < relayConfigs[i].numSchedules; j++) {
                JsonObject sched = schedules.createNestedObject();
                Schedule& s = relayConfigs[i].schedules[j];
                sched["hour"] = s.hour;
                sched["minute"] = s.minute;
                sched["state"] = s.state;
                sched["enabled"] = s.enabled;
                JsonArray days = sched.createNestedArray("days");
                for (int d = 0; d < 7; d++) {
                    days.add(s.daysOfWeek[d]);
                }
            }
        }
    }
    
    // Add MQTT status
    doc["mqtt_enabled"] = mqttConfig.enabled;
    doc["mqtt_connected"] = mqttConfig.enabled && mqtt.connected();
    if (mqttConfig.enabled) {
        doc["mqtt_server"] = mqttConfig.server;
        doc["mqtt_topic_prefix"] = mqttConfig.topic_prefix;
    }
    
    String msg;
    serializeJson(doc, msg);
    ws.textAll(msg);
}

void handleButtons() {
    if (millis() - lastButtonCheck < BUTTON_CHECK_INTERVAL) return;
    lastButtonCheck = millis();
    
    const unsigned long debounceDelay = 50;
    for(int i = 0; i < NUM_RELAYS; i++) {
        bool currentState = digitalRead(buttonPins[i]);
        
        if (currentState != buttonStates[i].lastState) {
            buttonStates[i].lastDebounceTime = millis();
            if (currentState != buttonStates[i].lastState) {
                Serial.printf("Button %d state changed: %s (pin %d)\n", 
                            i + 1, currentState ? "HIGH" : "LOW", buttonPins[i]);
            }
        }
        
        if ((millis() - buttonStates[i].lastDebounceTime) > debounceDelay) {
            if (currentState == LOW && !buttonStates[i].pressed) {
                buttonStates[i].pressed = true;
                setRelay(i, !relayState[i]);
                notifyClients();
            }
            else if (currentState == HIGH) {
                buttonStates[i].pressed = false;
            }
        }
        buttonStates[i].lastState = currentState;
    }
}

// Optimized schedule checking
void checkSchedules() {
    if (millis() - lastScheduleCheck < SCHEDULE_CHECK_INTERVAL) return;
    lastScheduleCheck = millis();
    
    if (!WiFi.status() == WL_CONNECTED) return;
    
    time_t now = time(nullptr);
    struct tm* timeinfo = localtime(&now);
    uint8_t currentHour = timeinfo->tm_hour;
    uint8_t currentMinute = timeinfo->tm_min;
    uint8_t currentDay = timeinfo->tm_wday;
    
    for (int i = 0; i < NUM_RELAYS; i++) {
        if (!relayConfigs[i].scheduled) continue;
        
        for (int j = 0; j < relayConfigs[i].numSchedules; j++) {
            Schedule& sched = relayConfigs[i].schedules[j];
            if (!sched.enabled) continue;
            
            if (sched.hour == currentHour && 
                sched.minute == currentMinute &&
                sched.daysOfWeek[currentDay]) {
                setRelay(i, sched.state);
            }
        }
    }
}

// Improved MQTT handling with status reporting
bool publishMQTTStatus() {
    if (!mqttConfig.enabled || !mqtt.connected()) return false;
    
    StaticJsonDocument<512> doc;
    doc["status"] = "online";
    doc["ip"] = WiFi.localIP().toString();
    doc["rssi"] = WiFi.RSSI();
    JsonObject relays = doc.createNestedObject("relays");
    
    for(int i = 0; i < NUM_RELAYS; i++) {
        JsonObject relay = relays.createNestedObject(String(i));
        relay["state"] = relayState[i];
        relay["name"] = relayConfigs[i].name;
    }
    
    String payload;
    serializeJson(doc, payload);
    return mqtt.publish((mqttConfig.topic_prefix + "status").c_str(), payload.c_str(), true);
}

void setupMQTT() {
    if (!mqttConfig.enabled || mqttConfig.server.length() == 0) {
        Serial.println("MQTT disabled or not configured");
        return;
    }
    
    mqtt.setServer(mqttConfig.server.c_str(), mqttConfig.port);
    mqtt.setCallback(mqttCallback);
    Serial.printf("MQTT configured: %s:%d\n", mqttConfig.server.c_str(), mqttConfig.port);
}

void reconnectMQTT() {
    if (!mqttConfig.enabled || mqtt.connected()) return;
    if (millis() - lastMQTTReconnectAttempt < MQTT_RECONNECT_INTERVAL) return;
    
    lastMQTTReconnectAttempt = millis();
    String clientId = "ESP8266Relay-" + String(ESP.getChipId(), HEX);
    
    Serial.print("Attempting MQTT connection...");
    bool connected = false;
      // Set up last will and testament
    String willTopic = mqttConfig.topic_prefix + "status";
    String willMessage = "{\"status\":\"offline\"}";

    if (mqttConfig.username.length() > 0) {
        connected = mqtt.connect(clientId.c_str(), 
                               mqttConfig.username.c_str(), 
                               mqttConfig.password.c_str(),
                               willTopic.c_str(),  // Last Will Topic
                               0,                  // Last Will QoS
                               true,              // Last Will Retain
                               willMessage.c_str()); // Last Will Message
    } else {
        connected = mqtt.connect(clientId.c_str(),
                               willTopic.c_str(),
                               0,
                               true,
                               willMessage.c_str());
    }
    
    if (connected) {
        Serial.println("connected");
        
        // Subscribe to all relay control topics
        String controlTopic = mqttConfig.topic_prefix + "+/set";
        mqtt.subscribe(controlTopic.c_str());
        
        // Subscribe to config topics
        String configTopic = mqttConfig.topic_prefix + "config/#";
        mqtt.subscribe(configTopic.c_str());
        
        // Publish initial status
        publishMQTTStatus();
        
        // Publish all relay states
        for (int i = 0; i < NUM_RELAYS; i++) {
            publishRelayState(i);
        }
    } else {
        Serial.printf("failed, rc=%d\n", mqtt.state());
    }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  String payloadStr = "";
  for (int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }
  
  // Extract relay number from topic
  String prefix = mqttConfig.topic_prefix;
  if (topicStr.startsWith(prefix)) {
    String relayPart = topicStr.substring(prefix.length());
    int relayNum = relayPart.toInt();
    
    if (relayNum >= 0 && relayNum < NUM_RELAYS) {
      bool newState = (payloadStr == "ON" || payloadStr == "1" || payloadStr == "true");
      setRelay(relayNum, newState);
      notifyClients();
    }
  }
}

void loadConfig() {
  if (!LittleFS.exists(CONFIG_FILE)) {
    Serial.println("Config file not found");
    return;
  }

  File configFile = LittleFS.open(CONFIG_FILE, "r");
  if (!configFile) {
    Serial.println("Failed to open config file");
    return;
  }

  StaticJsonDocument<2048> doc;  // Increased buffer size
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    Serial.printf("Failed to parse config file: %s\n", error.c_str());
    return;
  }

  // Load relay configurations
  JsonArray relays = doc["relays"];
  for(int i = 0; i < NUM_RELAYS && i < relays.size(); i++) {
    relayConfigs[i].name = relays[i]["name"].as<String>();
    relayConfigs[i].description = relays[i]["description"].as<String>();
    relayConfigs[i].scheduled = relays[i]["scheduled"] | false;
    relayConfigs[i].numSchedules = 0;

    if (relays[i].containsKey("schedules")) {
      JsonArray schedules = relays[i]["schedules"];
      for(JsonObject s : schedules) {
        if (relayConfigs[i].numSchedules >= 5) break;
        
        Schedule& sched = relayConfigs[i].schedules[relayConfigs[i].numSchedules];
        sched.hour = s["hour"] | 0;
        sched.minute = s["minute"] | 0;
        sched.enabled = s["enabled"] | false;
        sched.state = s["state"] | false;
        
        if (s.containsKey("days")) {
          JsonArray days = s["days"];
          for(int d = 0; d < 7 && d < days.size(); d++) {
            sched.daysOfWeek[d] = days[d] | false;
          }
        } else {
          for(int d = 0; d < 7; d++) sched.daysOfWeek[d] = false;
        }
        
        relayConfigs[i].numSchedules++;
        Serial.printf("Loaded schedule for relay %d: %02d:%02d, enabled=%d\n", 
                     i, sched.hour, sched.minute, sched.enabled);
      }
    }
  }

  // Load MQTT configuration
  mqttConfig.enabled = doc["mqtt"]["enabled"] | false;
  mqttConfig.server = doc["mqtt"]["server"] | "";
  mqttConfig.port = doc["mqtt"]["port"] | 1883;
  mqttConfig.username = doc["mqtt"]["username"] | "";
  mqttConfig.password = doc["mqtt"]["password"] | "";
  mqttConfig.topic_prefix = doc["mqtt"]["topic_prefix"] | "esp8266_relay/";

  if (mqttConfig.enabled) {
    setupMQTT();
  }
  
  Serial.println("Configuration loaded successfully");
}

void saveConfig() {
  // Save configuration to LittleFS
  StaticJsonDocument<512> doc;
  
  // Save relay configurations
  for(int i = 0; i < NUM_RELAYS; i++) {    doc["relays"]["relay" + String(i)]["name"] = relayConfigs[i].name;
    doc["relays"]["relay" + String(i)]["description"] = relayConfigs[i].description;
    doc["relays"]["relay" + String(i)]["scheduled"] = relayConfigs[i].scheduled;
    
    // Save schedules
    JsonArray scheduleArray = doc["relays"]["relay" + String(i)].createNestedArray("schedules");
    for (int j = 0; j < relayConfigs[i].numSchedules; j++) {
      JsonObject schedule = scheduleArray.createNestedObject();
      schedule["hour"] = relayConfigs[i].schedules[j].hour;
      schedule["minute"] = relayConfigs[i].schedules[j].minute;
      schedule["enabled"] = relayConfigs[i].schedules[j].enabled;
      schedule["state"] = relayConfigs[i].schedules[j].state;
      
      JsonArray days = schedule.createNestedArray("days");
      for (int d = 0; d < 7; d++) {
        days.add(relayConfigs[i].schedules[j].daysOfWeek[d]);
      }
    }
  }
  
  // Save MQTT configuration
  doc["mqtt"]["enabled"] = mqttConfig.enabled;
  doc["mqtt"]["server"] = mqttConfig.server;
  doc["mqtt"]["port"] = mqttConfig.port;
  doc["mqtt"]["username"] = mqttConfig.username;
  doc["mqtt"]["password"] = mqttConfig.password;
  doc["mqtt"]["topic_prefix"] = mqttConfig.topic_prefix;
  
  File f = LittleFS.open(CONFIG_FILE, "w");
  if (f) { serializeJson(doc, f); f.close(); }
}

void publishRelayState(int relayIndex) {
  if (!mqttConfig.enabled || !mqtt.connected()) return;
  
  String baseTopic = mqttConfig.topic_prefix + String(relayIndex);
  
  // Publish state
  String stateTopic = baseTopic + "/state";
  String statePayload = relayState[relayIndex] ? "ON" : "OFF";
  mqtt.publish(stateTopic.c_str(), statePayload.c_str(), true);
  
  // Publish name and description
  String nameTopic = baseTopic + "/name";
  mqtt.publish(nameTopic.c_str(), relayConfigs[relayIndex].name.c_str(), true);
  
  String descTopic = baseTopic + "/description";
  mqtt.publish(descTopic.c_str(), relayConfigs[relayIndex].description.c_str(), true);
  
  // Publish schedule status
  String schedTopic = baseTopic + "/scheduled";
  String schedPayload = relayConfigs[relayIndex].scheduled ? "true" : "false";
  mqtt.publish(schedTopic.c_str(), schedPayload.c_str(), true);
  
  // Publish active schedules count
  String countTopic = baseTopic + "/schedules_count";
  String countPayload = String(relayConfigs[relayIndex].numSchedules);
  mqtt.publish(countTopic.c_str(), countPayload.c_str(), true);
}

void syncTime() {
  const int timeZone = 0;  // Change this to your timezone offset in hours
  const int daylightOffset = 0;  // Set to 3600 if daylight savings is in effect
  configTime(timeZone * 3600, daylightOffset, "pool.ntp.org", "time.nist.gov");
  
  Serial.println("Waiting for time sync...");
  time_t now = time(nullptr);
  int retries = 0;
  while (now < 1000000000L && retries < 20) {  // Add retry limit
    delay(500);
    now = time(nullptr);
    retries++;
  }
  
  if (now > 1000000000L) {
    struct tm timeinfo;
    gmtime_r(&now, &timeinfo);
    Serial.printf("Time synced: %02d:%02d:%02d\n", 
                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  } else {
    Serial.println("Time sync failed");
  }
}

void resetAll() {
    Serial.println("Resetting device...");
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

  // Initialize button states
  for(int i = 0; i < NUM_RELAYS; i++) {
    buttonStates[i].lastState = HIGH;
    buttonStates[i].pressed = false;
    buttonStates[i].lastDebounceTime = 0;
  }

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS");
    return; // Stop or handle error here
  }
  Serial.println("LittleFS mounted");
  
  // Load configurations
  loadConfig();
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
  loadConfig();
  setLEDStatus(false, true, true);
  ws.onEvent([](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t *data, size_t len) {
    switch(type) {
      case WS_EVT_CONNECT:
        {
          Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
          
          // Send full state to new client
          StaticJsonDocument<1024> doc;
          
          // Relay states
          JsonObject relays = doc.createNestedObject("relays");
          for(int i = 0; i < NUM_RELAYS; i++) {
            JsonObject relay = relays.createNestedObject(String(i));
            relay["state"] = relayState[i];
            relay["name"] = relayConfigs[i].name;
            relay["description"] = relayConfigs[i].description;
            relay["scheduled"] = relayConfigs[i].scheduled;
          }
          
          // MQTT status
          doc["mqtt_connected"] = mqttConfig.enabled && mqtt.connected();
          
          // System time
          doc["time_synced"] = time(nullptr) > 1000000000L;
          
          String msg;
          serializeJson(doc, msg);
          client->text(msg);
        }
        break;
        
      case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
        
      case WS_EVT_DATA:
        {
          String msg = String((char*)data).substring(0, len);
          Serial.printf("WebSocket received from #%u: %s\n", client->id(), msg.c_str());
          
          StaticJsonDocument<256> doc;
          DeserializationError error = deserializeJson(doc, msg);
          
          if (error) {
            String errorMsg = "{\"error\":\"Invalid JSON format\"}";
            client->text(errorMsg);
            return;
          }
          
          // Handle different command types
          String cmd = doc["cmd"].as<String>();
          
          if (cmd == "toggle") {
            if (!doc.containsKey("relay")) {
              client->text("{\"error\":\"Missing relay parameter\"}");
              return;
            }
            
            int idx = doc["relay"].as<int>();
            if (idx < 0 || idx >= NUM_RELAYS) {
              client->text("{\"error\":\"Invalid relay index\"}");
              return;
            }
            
            setRelay(idx, !relayState[idx]);
            notifyClients();
          }
          else if (cmd == "set") {
            if (!doc.containsKey("relay") || !doc.containsKey("state")) {
              client->text("{\"error\":\"Missing relay or state parameter\"}");
              return;
            }
            
            int idx = doc["relay"].as<int>();
            bool state = doc["state"].as<bool>();
            
            if (idx < 0 || idx >= NUM_RELAYS) {
              client->text("{\"error\":\"Invalid relay index\"}");
              return;
            }
            
            setRelay(idx, state);
            notifyClients();
          }
          else if (cmd == "status") {
            StaticJsonDocument<512> statusDoc;
            statusDoc["mqtt_connected"] = mqttConfig.enabled && mqtt.connected();
            statusDoc["wifi_rssi"] = WiFi.RSSI();
            statusDoc["free_heap"] = ESP.getFreeHeap();
            statusDoc["uptime"] = millis();
            
            String statusMsg;
            serializeJson(statusDoc, statusMsg);
            client->text(statusMsg);
          }
          else {
            client->text("{\"error\":\"Unknown command\"}");
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
    StaticJsonDocument<512> doc;
    JsonArray relayArray = doc.createNestedArray("relays");
    
    for(int i = 0; i < NUM_RELAYS; i++) {
      JsonObject relay = relayArray.createNestedObject();
      relay["id"] = i;
      relay["state"] = relayState[i];
      relay["name"] = relayConfigs[i].name;
      relay["description"] = relayConfigs[i].description;
      relay["scheduled"] = relayConfigs[i].scheduled;
    }
    
    String res;
    serializeJson(doc, res);
    req->send(200, "application/json", res);
  });

  // Update relay endpoint
  server.on("/api/relay", HTTP_POST, [](AsyncWebServerRequest *req){
    req->send(400, "application/json", "{\"error\":\"Invalid request format\"}");
  },
  NULL,
  [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, (char*)data);
    
    if (error) {
      req->send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    
    if (!doc.containsKey("relay") || !doc.containsKey("state")) {
      req->send(400, "application/json", "{\"error\":\"Missing relay or state parameter\"}");
      return;
    }
    
    int relayIdx = doc["relay"].as<int>();
    bool newState = doc["state"].as<bool>();
    
    if (relayIdx < 0 || relayIdx >= NUM_RELAYS) {
      req->send(400, "application/json", "{\"error\":\"Invalid relay index\"}");
      return;
    }
    
    setRelay(relayIdx, newState);
    
    StaticJsonDocument<256> response;
    response["success"] = true;
    response["relay"] = relayIdx;
    response["state"] = relayState[relayIdx];
    
    String res;
    serializeJson(response, res);
    req->send(200, "application/json", res);
  });

  // Enhanced status endpoint with additional information
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *req){
    StaticJsonDocument<1024> doc;
    
    // Device info
    JsonObject device = doc.createNestedObject("device");
    device["chipId"] = String(ESP.getChipId(), HEX);
    device["freeHeap"] = ESP.getFreeHeap();
    device["uptime"] = millis();
    device["resetReason"] = ESP.getResetReason();
    device["flashSize"] = ESP.getFlashChipSize();
    device["cpuFreq"] = ESP.getCpuFreqMHz();
    
    // WiFi info
    JsonObject wifi = doc.createNestedObject("wifi");
    wifi["ssid"] = WiFi.SSID();
    wifi["rssi"] = WiFi.RSSI();
    wifi["ip"] = WiFi.localIP().toString();
    wifi["mac"] = WiFi.macAddress();
    wifi["connected"] = WiFi.status() == WL_CONNECTED;
    
    // MQTT info
    JsonObject mqtt_status = doc.createNestedObject("mqtt");
    mqtt_status["enabled"] = mqttConfig.enabled;
    mqtt_status["connected"] = mqttConfig.enabled && mqtt.connected();
    mqtt_status["server"] = mqttConfig.server;
    mqtt_status["topic_prefix"] = mqttConfig.topic_prefix;
    
    // WebSocket info
    JsonObject websocket = doc.createNestedObject("websocket");
    websocket["clients"] = ws.count();
    
    // System time
    JsonObject timeInfo = doc.createNestedObject("time");
    time_t now = time(nullptr);
    timeInfo["timestamp"] = now;
    timeInfo["synced"] = now > 1000000000L;
    
    // Relay states with names and schedules
    JsonObject relays = doc.createNestedObject("relays");
    for(int i = 0; i < NUM_RELAYS; i++) {
      JsonObject relay = relays.createNestedObject(String(i));
      relay["state"] = relayState[i];
      relay["name"] = relayConfigs[i].name;
      relay["description"] = relayConfigs[i].description;
      relay["scheduled"] = relayConfigs[i].scheduled;
      relay["numSchedules"] = relayConfigs[i].numSchedules;
    }
    
    String res;
    serializeJson(doc, res);
    req->send(200, "application/json", res);
  });

  // Get schedules for a relay
  server.on("/api/schedules", HTTP_GET, [](AsyncWebServerRequest *req){
    if(!req->hasParam("relay")) {
      req->send(400, "text/plain", "Missing relay parameter");
      return;
    }
    
    int relayIdx = req->getParam("relay")->value().toInt();
    if(relayIdx < 0 || relayIdx >= NUM_RELAYS) {
      req->send(400, "text/plain", "Invalid relay index");
      return;
    }
    
    StaticJsonDocument<1024> doc;
    doc["scheduled"] = relayConfigs[relayIdx].scheduled;
    JsonArray schedules = doc.createNestedArray("schedules");
    
    for(int i = 0; i < relayConfigs[relayIdx].numSchedules; i++) {
      JsonObject sched = schedules.createNestedObject();
      Schedule& s = relayConfigs[relayIdx].schedules[i];
      sched["hour"] = s.hour;
      sched["minute"] = s.minute;
      JsonArray days = sched.createNestedArray("days");
      for(int d = 0; d < 7; d++) {
        days.add(s.daysOfWeek[d]);
      }
      sched["state"] = s.state;
      sched["enabled"] = s.enabled;
    }
    
    String response;
    serializeJson(doc, response);
    req->send(200, "application/json", response);
  });

  // Update schedules for a relay
  server.on("/api/schedules", HTTP_POST, [](AsyncWebServerRequest *req){
    req->send(400, "text/plain", "Please send schedule data in request body");
  },
  NULL,
  [](AsyncWebServerRequest *req, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, (char*)data);
    
    if (error) {
      req->send(400, "text/plain", "Invalid JSON");
      return;
    }
    
    if (!doc.containsKey("relay")) {
      req->send(400, "text/plain", "Missing relay parameter");
      return;
    }
    
    int relayIdx = doc["relay"].as<int>();
    if (relayIdx < 0 || relayIdx >= NUM_RELAYS) {
      req->send(400, "text/plain", "Invalid relay index");
      return;
    }
    
    relayConfigs[relayIdx].scheduled = doc["scheduled"] | false;
    relayConfigs[relayIdx].numSchedules = 0;
    
    if (doc.containsKey("schedules")) {
      JsonArray schedules = doc["schedules"];
      for(JsonObject s : schedules) {
        if (relayConfigs[relayIdx].numSchedules >= 5) break;
        
        Schedule& sched = relayConfigs[relayIdx].schedules[relayConfigs[relayIdx].numSchedules];
        sched.hour = s["hour"] | 0;
        sched.minute = s["minute"] | 0;
        JsonArray days = s["days"];
        for(int d = 0; d < 7 && d < days.size(); d++) {
          sched.daysOfWeek[d] = days[d] | false;
        }
        sched.state = s["state"] | false;
        sched.enabled = s["enabled"] | false;
        relayConfigs[relayIdx].numSchedules++;
      }
    }
    
    saveConfig();
    req->send(200, "text/plain", "OK");
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
    ws.cleanupClients();
    
    // Time sync check
    if (millis() - lastTimeUpdate > TIME_UPDATE_INTERVAL) {
        syncTime();
        lastTimeUpdate = millis();
    }
    
    // Check schedules (function has internal timing check)
    checkSchedules();
    
    // Handle MQTT
    if (mqttConfig.enabled) {
        if (!mqtt.connected()) {
            reconnectMQTT();  // Function has internal timing check
        } else {
            mqtt.loop();
        }
    }
    
    // Handle buttons (function has internal timing check)
    handleButtons();
    
    // Update LED status
    if (millis() - lastLEDUpdate > LED_UPDATE_INTERVAL) {
        lastLEDUpdate = millis();
        bool connected = WiFi.status() == WL_CONNECTED && 
                        (!mqttConfig.enabled || mqtt.connected());
        setLEDStatus(false, connected, true);
    }
    
    yield();  // Allow ESP8266 to handle system tasks
}
