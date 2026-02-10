#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ArduinoJson.h>
#include <esp_system.h>

void logAppendChar(uint8_t c);
void addLogLine(const String &line);
void publishLogLine(const String &line);
void handleLogPublishQueue();
void publishLogBacklog();
String buildLogPayload(unsigned long seq, const String &line);
String escapeJsonString(const String &input);
void sendLogSnapshot(unsigned long sinceSeq);

class LogSerialWrapper : public Print {
public:
    explicit LogSerialWrapper(HardwareSerial &serialRef) : serial(serialRef) {}

    void begin(unsigned long baud) { serial.begin(baud); }
    void begin(unsigned long baud, uint32_t config) { serial.begin(baud, config); }
    void flush() { serial.flush(); }

    size_t write(uint8_t c) override {
        logAppendChar(c);
        return serial.write(c);
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        for (size_t i = 0; i < size; i++) {
            logAppendChar(buffer[i]);
        }
        return serial.write(buffer, size);
    }

    using Print::print;
    using Print::println;
    using Print::printf;

private:
    HardwareSerial &serial;
};

LogSerialWrapper LogSerial(::Serial);
#define Serial LogSerial

// Versão do firmware
#define FIRMWARE_VERSION "3.0.4"

// Definições para DS18B20
#define ONE_WIRE_BUS 4
#define TEMPERATURE_PRECISION 12

// Intervalo de leitura de temperatura em milissegundos
#define TEMP_READ_INTERVAL 300000  // 5 minutos
// Intervalo do heartbeat em milissegundos
#define HEARTBEAT_INTERVAL 60000  // 1 minuto

// Botao BOOT (GPIO0) para reset de fabrica
#define FACTORY_RESET_BUTTON_PIN 0
#define FACTORY_RESET_HOLD_MS 10000

// Log
#define LOG_BUFFER_LINES 120
#define LOG_MAX_LINE_LENGTH 256
#define LOG_MQTT_MIN_INTERVAL_MS 200

// Estruturas para configuração
struct SensorConfig {
  char sensorName[32];
  char location[64];
  int readInterval;  // Intervalo de leitura em segundos
};

struct WiFiConfig {
  char ssid[32];
  char password[64];
};

struct MQTTConfig {
  char server[64];
  int port;
  char username[32];
  char password[64];
  char topic[64];
};

struct WebAuthConfig {
  char username[32];
  char password[64];
};

struct LogEntry {
    unsigned long seq;
    String line;
};

// Objetos globais
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebAuthConfig webAuthConfig;
SensorConfig sensorConfig;


// Configurações
WiFiConfig wifiConfig;
MQTTConfig mqttConfig;
bool shouldReboot = false;
bool isAPMode = false;
int mqttFailCount = 0;
unsigned long lastHeartbeat = 0;
unsigned long factoryResetPressStart = 0;
bool factoryResetPressed = false;
LogEntry logBuffer[LOG_BUFFER_LINES];
unsigned int logBufferHead = 0;
unsigned long logSeq = 0;
String logLineBuffer;
String deviceMac;
String deviceMacTopic;
String logMqttTopic;
unsigned long lastLogPublishMs = 0;
unsigned long lastLogPublishedSeq = 0;
bool logPublishPending = false;
unsigned long logPendingSeq = 0;
String logPendingLine;

// Endereços EEPROM
#define EEPROM_SIZE 1024
#define WEB_CONFIG_ADDR 200
#define WIFI_CONFIG_ADDR 0
#define MQTT_CONFIG_ADDR 100
#define SENSOR_CONFIG_ADDR 300

// Variáveis OTA
bool otaInProgress = false;

// Variáveis do sensor
DeviceAddress tempSensor;
bool sensorFound = false;

// Declaração antecipada das funções
void performOTAUpload(String url);
float readTemperature();
void setupDS18B20();
void loadConfig();
void saveConfig();
void connectWiFi();
void setupAPMode();
void connectMQTT();
void readAndSendTemperature();
void sendHeartbeat();
bool isAuthenticated();
void setupWebServer();
void handleFactoryResetButton();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Iniciando Sistema ESP32 Temperature ===");
  Serial.println("\n=== INICIANDO SISTEMA ===");
  
  // Inicializar EEPROM
  EEPROM.begin(EEPROM_SIZE);

    WiFi.mode(WIFI_STA);
    delay(50);
    uint8_t macBytes[6];
    WiFi.macAddress(macBytes);
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
    deviceMac = String(macStr);
    deviceMacTopic = deviceMac;
    deviceMacTopic.replace(":", "");
    deviceMacTopic.toLowerCase();
    logMqttTopic = "devices/" + deviceMacTopic + "/log";

    // BOOT (GPIO0) e ativo em nivel baixo
    pinMode(FACTORY_RESET_BUTTON_PIN, INPUT_PULLUP);
  
  // Inicializar sensor DS18B20
  setupDS18B20();
  
  // Carregar configurações
  loadConfig();

  // DEBUG - Mostrar configuração carregada
  printCurrentConfig();
  
  // Conectar WiFi
  connectWiFi();
  
// Configurar MQTT apenas se não estiver em modo AP
  if (!isAPMode) {
    mqttClient.setServer(mqttConfig.server, mqttConfig.port);
    mqttClient.setBufferSize(512);  // Aumentar buffer para suportar mensagens maiores
    // Tentar conectar imediatamente
    connectMQTT();
  }
  
  // Configurar rotas web
  setupWebServer();
  
  Serial.println("Sistema iniciado com sucesso!");
  Serial.print("Versão: ");
  Serial.println(FIRMWARE_VERSION);
  
  if (isAPMode) {
    Serial.println("Modo: AP (Aguardando configuração)");
    Serial.println("Conecte-se ao WiFi: ESP32-Temperature");
    Serial.println("Senha: 12345678");
    Serial.println("Acesse: http://192.168.4.1");
  } else {
    Serial.println("Modo: Estação WiFi");
    Serial.println("Acesse: http://" + WiFi.localIP().toString());
  }
}

void setupDS18B20() {
  sensors.begin();
  
  // Localizar sensores
  int deviceCount = sensors.getDeviceCount();
  
  if (deviceCount > 0) {
    sensorFound = true;
    // Configurar o primeiro sensor encontrado
    if (!sensors.getAddress(tempSensor, 0)) {
      sensorFound = false;
    } else {
      Serial.print("Endereço do sensor: ");
      for (uint8_t i = 0; i < 8; i++) {
        Serial.print("0x");
        if (tempSensor[i] < 16) Serial.print("0");
        Serial.print(tempSensor[i], HEX);
        if (i < 7) Serial.print(", ");
      }
      Serial.println();
      
      // Configurar resolução
      sensors.setResolution(tempSensor, TEMPERATURE_PRECISION);
    }
  } else {
    sensorFound = false;
  }
}

void loop() {
    handleFactoryResetButton();
  server.handleClient();
  
  if (!otaInProgress && !isAPMode) {
    // Verificar reconexão WiFi
    static unsigned long lastWiFiCheck = 0;
    if (millis() - lastWiFiCheck > 30000) { // Verifica a cada 30 segundos
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("⚠️ WiFi desconectado! Tentando reconectar...");
        WiFi.disconnect();
        WiFi.begin(wifiConfig.ssid, wifiConfig.password);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 10) {
          delay(500);
          Serial.print(".");
          attempts++;
        }
        
        if (WiFi.status() == WL_CONNECTED) {
          Serial.println("\n✅ WiFi reconectado! IP: " + WiFi.localIP().toString());
        } else {
          Serial.println("\n❌ Falha na reconexão WiFi");
        }
      }
      lastWiFiCheck = millis();
    }
    
    // Verificar reconexão MQTT
    if (!mqttClient.connected()) {
      static unsigned long lastMQTTAttempt = 0;
      if (millis() - lastMQTTAttempt > 5000) { // Tenta reconectar a cada 5 segundos
        connectMQTT();
        lastMQTTAttempt = millis();
      }
    }
    
    mqttClient.loop();
    handleLogPublishQueue();

        if (mqttClient.connected()) {
            unsigned long now = millis();
            if (now - lastHeartbeat >= HEARTBEAT_INTERVAL) {
                sendHeartbeat();
                lastHeartbeat = now;
            }
        }
    
    // Leitura e envio de temperatura
    static unsigned long lastTempRead = 0;
    unsigned long interval = (unsigned long)sensorConfig.readInterval * 1000; // Converter para ms
    if (millis() - lastTempRead > interval) {
      readAndSendTemperature();
      lastTempRead = millis();
    }
  }
  
  if (shouldReboot) {
    delay(1000);
    ESP.restart();
  }
}

void handleFactoryResetButton() {
    bool pressed = (digitalRead(FACTORY_RESET_BUTTON_PIN) == LOW);
    unsigned long now = millis();

    if (pressed) {
        if (!factoryResetPressed) {
            factoryResetPressed = true;
            factoryResetPressStart = now;
        }
        return;
    }

    if (factoryResetPressed) {
        unsigned long heldMs = now - factoryResetPressStart;
        factoryResetPressed = false;
        factoryResetPressStart = 0;

        if (heldMs >= FACTORY_RESET_HOLD_MS) {
            Serial.println("Botao BOOT segurado por 10s. Reset de fabrica...");
            performFactoryReset();
        }
    }
}

void logAppendChar(uint8_t c) {
    if (c == '\r') {
        return;
    }

    if (c == '\n') {
        if (logLineBuffer.length() > 0) {
            addLogLine(logLineBuffer);
            logLineBuffer = "";
        }
        return;
    }

    if (logLineBuffer.length() >= LOG_MAX_LINE_LENGTH) {
        addLogLine(logLineBuffer);
        logLineBuffer = "";
    }

    logLineBuffer += (char)c;
}

void addLogLine(const String &line) {
    logSeq++;
    logBuffer[logBufferHead].seq = logSeq;
    logBuffer[logBufferHead].line = line;
    logBufferHead = (logBufferHead + 1) % LOG_BUFFER_LINES;

    publishLogLine(line);
}

void publishLogLine(const String &line) {
    if (!mqttClient.connected() || logMqttTopic.length() == 0) {
        return;
    }

    unsigned long now = millis();
    if (now - lastLogPublishMs < LOG_MQTT_MIN_INTERVAL_MS) {
        logPublishPending = true;
        logPendingSeq = logSeq;
        logPendingLine = line;
        return;
    }

    String payload = buildLogPayload(logSeq, line);
    if (mqttClient.publish(logMqttTopic.c_str(), payload.c_str())) {
        lastLogPublishMs = now;
        lastLogPublishedSeq = logSeq;
    }
}

void handleLogPublishQueue() {
    if (!logPublishPending || !mqttClient.connected() || logMqttTopic.length() == 0) {
        return;
    }

    unsigned long now = millis();
    if (now - lastLogPublishMs < LOG_MQTT_MIN_INTERVAL_MS) {
        return;
    }

    String payload = buildLogPayload(logPendingSeq, logPendingLine);
    if (mqttClient.publish(logMqttTopic.c_str(), payload.c_str())) {
        lastLogPublishMs = now;
        lastLogPublishedSeq = logPendingSeq;
        logPublishPending = false;
        logPendingSeq = 0;
        logPendingLine = "";
    }
}

void publishLogBacklog() {
    if (!mqttClient.connected() || logMqttTopic.length() == 0) {
        return;
    }

    for (unsigned int i = 0; i < LOG_BUFFER_LINES; i++) {
        unsigned int index = (logBufferHead + i) % LOG_BUFFER_LINES;
        if (logBuffer[index].seq == 0) {
            continue;
        }
        if (logBuffer[index].seq <= lastLogPublishedSeq) {
            continue;
        }

        unsigned long now = millis();
        if (now - lastLogPublishMs < LOG_MQTT_MIN_INTERVAL_MS) {
            delay(LOG_MQTT_MIN_INTERVAL_MS - (now - lastLogPublishMs));
        }

        String payload = buildLogPayload(logBuffer[index].seq, logBuffer[index].line);
        if (mqttClient.publish(logMqttTopic.c_str(), payload.c_str())) {
            lastLogPublishMs = millis();
            lastLogPublishedSeq = logBuffer[index].seq;
        }
    }
}

String buildLogPayload(unsigned long seq, const String &line) {
    String payload = "{\"seq\":" + String(seq) + ",\"ts\":" + String(millis()) + ",\"msg\":\"" +
                                     escapeJsonString(line) + "\"}";
    return payload;
}

String escapeJsonString(const String &input) {
    String out;
    out.reserve(input.length() + 8);
    for (size_t i = 0; i < input.length(); i++) {
        char c = input[i];
        switch (c) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            default: out += c; break;
        }
    }
    return out;
}

void sendLogSnapshot(unsigned long sinceSeq) {
    String json = "{\"nextSeq\":" + String(logSeq) + ",\"lines\":[";
    bool first = true;

    for (unsigned int i = 0; i < LOG_BUFFER_LINES; i++) {
        unsigned int index = (logBufferHead + i) % LOG_BUFFER_LINES;
        if (logBuffer[index].seq == 0) {
            continue;
        }
        if (logBuffer[index].seq <= sinceSeq) {
            continue;
        }
        if (!first) {
            json += ",";
        }
        json += "{\"seq\":" + String(logBuffer[index].seq) + ",\"msg\":\"" +
                        escapeJsonString(logBuffer[index].line) + "\"}";
        first = false;
    }

    json += "]}";
    server.send(200, "application/json", json);
}

// Funções de configuração
void loadConfig() {
  // Limpar as estruturas antes de carregar
  memset(&wifiConfig, 0, sizeof(wifiConfig));
  memset(&mqttConfig, 0, sizeof(mqttConfig));
  memset(&webAuthConfig, 0, sizeof(webAuthConfig));
  memset(&sensorConfig, 0, sizeof(sensorConfig));
  
  // Carregar da EEPROM
  EEPROM.get(WIFI_CONFIG_ADDR, wifiConfig);
  EEPROM.get(MQTT_CONFIG_ADDR, mqttConfig);
  EEPROM.get(WEB_CONFIG_ADDR, webAuthConfig);
  EEPROM.get(SENSOR_CONFIG_ADDR, sensorConfig);

  // VERIFICAÇÃO WIFI - Validar se os dados são válidos
  bool wifiValid = true;
  if (strlen(wifiConfig.ssid) == 0 || strlen(wifiConfig.ssid) > 31) {
    wifiValid = false;
  } else {
    for (int i = 0; i < strlen(wifiConfig.ssid); i++) {
      if (!isprint(wifiConfig.ssid[i]) || wifiConfig.ssid[i] < 32 || wifiConfig.ssid[i] > 126) {
        wifiValid = false;
        Serial.println("⚠️ SSID WiFi corrompido detectado!");
        break;
      }
    }
  }
  
  if (!wifiValid) {
    Serial.println("⚠️ Configuração WiFi corrompida - limpando...");
    memset(&wifiConfig, 0, sizeof(wifiConfig));
  }

  // VERIFICAÇÃO MQTT - Validar se os dados são realmente inválidos (lixo da EEPROM)
  bool mqttValid = true;
  
  // Verificar se o servidor tem caracteres válidos
  if (strlen(mqttConfig.server) == 0 || strlen(mqttConfig.server) > 63) {
    mqttValid = false;
  } else {
    // Verificar se contém caracteres inválidos (lixo de memória)
    for (int i = 0; i < strlen(mqttConfig.server); i++) {
      if (!isprint(mqttConfig.server[i]) || mqttConfig.server[i] < 32 || mqttConfig.server[i] > 126) {
        mqttValid = false;
        break;
      }
    }
  }
  
  // Verificar porta válida (1-65535)
  if (mqttConfig.port < 1 || mqttConfig.port > 65535) {
    mqttValid = false;
  }
  
  // Verificar tópico
  if (strlen(mqttConfig.topic) == 0 || strlen(mqttConfig.topic) > 63) {
    mqttValid = false;
  } else {
    for (int i = 0; i < strlen(mqttConfig.topic); i++) {
      if (!isprint(mqttConfig.topic[i]) || mqttConfig.topic[i] < 32 || mqttConfig.topic[i] > 126) {
        mqttValid = false;
        Serial.println("⚠️ Tópico MQTT corrompido detectado!");
        break;
      }
    }
  }
  
  // Se configuração MQTT não for válida, definir padrão
  if (!mqttValid) {
    Serial.println("⚠️ Configuração MQTT inválida ou corrompida, usando padrão");
    strcpy(mqttConfig.server, "189.90.40.78");
    mqttConfig.port = 1883;
    strcpy(mqttConfig.username, "esp32");
    strcpy(mqttConfig.password, "123@Mudar");
    strcpy(mqttConfig.topic, "esp32/temperature");
    saveConfig(); // Salvar os padrões
  } else {
    Serial.println("✅ Configuração MQTT carregada da EEPROM");
    Serial.println("  Servidor: " + String(mqttConfig.server));
    Serial.println("  Porta: " + String(mqttConfig.port));
    Serial.println("  Tópico: " + String(mqttConfig.topic));
  }
  
  
  // Verificar se as credenciais web são válidas
  bool credentialsValid = true;
  
  // Verificar se o username tem caracteres válidos
  if (strlen(webAuthConfig.username) == 0 || strlen(webAuthConfig.username) > 30) {
    credentialsValid = false;
  } else {
    // Verificar caracteres do username
    for (int i = 0; i < strlen(webAuthConfig.username); i++) {
      if (!isprint(webAuthConfig.username[i])) {
        credentialsValid = false;
        break;
      }
    }
  }
  
  // Verificar se a senha tem caracteres válidos
  if (strlen(webAuthConfig.password) == 0 || strlen(webAuthConfig.password) > 60) {
    credentialsValid = false;
  } else {
    // Verificar caracteres da senha
    for (int i = 0; i < strlen(webAuthConfig.password); i++) {
      if (!isprint(webAuthConfig.password[i])) {
        credentialsValid = false;
        break;
      }
    }
  }
  
  // Se as credenciais não forem válidas, definir padrão
  if (!credentialsValid) {
    strcpy(webAuthConfig.username, "admin");
    strcpy(webAuthConfig.password, "admin123");
    saveConfig();
  }
  
  // Configurações WiFi - não definir SSID padrão para forçar configuração inicial
  // O modo AP será ativado automaticamente se não houver SSID configurado

  // Verificar e validar configurações do sensor
  bool sensorValid = true;
  
  // Verificar nome do sensor
  if (strlen(sensorConfig.sensorName) == 0 || strlen(sensorConfig.sensorName) > 31) {
    sensorValid = false;
  } else {
    // Verificar caracteres válidos
    for (int i = 0; i < strlen(sensorConfig.sensorName); i++) {
      if (!isprint(sensorConfig.sensorName[i]) || sensorConfig.sensorName[i] < 32 || sensorConfig.sensorName[i] > 126) {
        sensorValid = false;
        Serial.println("⚠️ Nome do sensor corrompido detectado!");
        break;
      }
    }
  }
  
  // Verificar localização (pode estar vazia, mas não pode ter lixo)
  if (strlen(sensorConfig.location) > 63) {
    sensorValid = false;
  } else if (strlen(sensorConfig.location) > 0) {
    for (int i = 0; i < strlen(sensorConfig.location); i++) {
      if (!isprint(sensorConfig.location[i]) || sensorConfig.location[i] < 32 || sensorConfig.location[i] > 126) {
        sensorValid = false;
        Serial.println("⚠️ Localização do sensor corrompida detectada!");
        break;
      }
    }
  }
  
  // Se configuração do sensor não for válida, definir padrão
  if (!sensorValid) {
    Serial.println("⚠️ Configuração do sensor inválida ou corrompida, usando padrão");
    strcpy(sensorConfig.sensorName, "DS18B20");
    sensorConfig.location[0] = '\0'; // Localização vazia
    sensorConfig.readInterval = 300;
    saveConfig();
  }
  
  // Validar intervalo de leitura (10 segundos a 1 hora)
  if (sensorConfig.readInterval < 10 || sensorConfig.readInterval > 3600) {
    sensorConfig.readInterval = 300; // 5 minutos padrão se fora do intervalo
  }
}

void saveConfig() {
  Serial.println("========================================");
  Serial.println("💾 Salvando configurações na EEPROM...");
  Serial.println("========================================");
  
  // Salvar WiFi
  Serial.println("📡 WiFi Config:");
  Serial.println("   SSID: " + String(wifiConfig.ssid));
  EEPROM.put(WIFI_CONFIG_ADDR, wifiConfig);
  
  // Salvar MQTT
  Serial.println("🔌 MQTT Config:");
  Serial.println("   Servidor: " + String(mqttConfig.server));
  Serial.println("   Porta: " + String(mqttConfig.port));
  Serial.println("   Tópico: " + String(mqttConfig.topic));
  EEPROM.put(MQTT_CONFIG_ADDR, mqttConfig);
  
  // Salvar Autenticação Web
  Serial.println("🔐 Web Auth:");
  Serial.println("   Usuário: " + String(webAuthConfig.username));
  EEPROM.put(WEB_CONFIG_ADDR, webAuthConfig);
  
  // Salvar Sensor
  Serial.println("🌡️ Sensor Config:");
  Serial.println("   Nome: " + String(sensorConfig.sensorName));
  Serial.println("   Local: " + String(sensorConfig.location));
  Serial.println("   Intervalo: " + String(sensorConfig.readInterval) + "s");
  EEPROM.put(SENSOR_CONFIG_ADDR, sensorConfig);
  
  // Commit (gravar fisicamente na EEPROM)
  bool committed = EEPROM.commit();
  if (committed) {
    Serial.println("✅ EEPROM.commit() SUCESSO!");
  } else {
    Serial.println("❌ ERRO: EEPROM.commit() FALHOU!");
  }
  Serial.println("========================================");
}

// Conexão WiFi
void connectWiFi() {
  // Se não tem SSID configurado, vai direto para modo AP
  if (strlen(wifiConfig.ssid) == 0 || wifiConfig.ssid[0] == '\0') {
    Serial.println("Nenhuma rede WiFi configurada");
    setupAPMode();
    return;
  }
  
  Serial.print("Conectando ao WiFi: ");
  Serial.println(wifiConfig.ssid);
  
  WiFi.begin(wifiConfig.ssid, wifiConfig.password);
  
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConectado! IP: " + WiFi.localIP().toString());
    isAPMode = false;
  } else {
    // Modo AP como fallback
    setupAPMode();
  }
}

void setupAPMode() {
  Serial.println("🔴 Iniciando modo AP...");
  
  // Para o WiFi anterior
  WiFi.disconnect();
  delay(500);
  
  // Configura como AP
  WiFi.mode(WIFI_AP);
  delay(100);
  
  // Inicia o AP
  bool apStarted = WiFi.softAP("ESP32-Temperature", "12345678");
  
  if (apStarted) {
    isAPMode = true;
    Serial.println("✅ Modo AP ativo");
    Serial.println("   SSID: ESP32-Temperature");
    Serial.println("   Password: 12345678");
    Serial.println("   IP: " + WiFi.softAPIP().toString());
  } else {
    Serial.println("❌ Falha ao iniciar modo AP");
  }
}

// Funções MQTT
void connectMQTT() {
    if (!mqttClient.connected()) {
        Serial.print("Conectando MQTT...");
        Serial.print(" Servidor: ");
        Serial.print(mqttConfig.server);
        Serial.print(":");
        Serial.print(mqttConfig.port);
        Serial.print(" Usuário: ");
        Serial.print(mqttConfig.username);

        // Definir callback para mensagens
        mqttClient.setCallback([](char* topic, byte* payload, unsigned int length){
            String msg;
            for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
            String t = String(topic);
            Serial.println("MQTT mensagem recebida -> Topic: " + t + " Payload: " + msg);
            StaticJsonDocument<256> doc;
            DeserializationError err = deserializeJson(doc, msg);
            if (err) {
              Serial.println("Falha ao desserializar comando MQTT");
              return;
            }

            if (t.endsWith("/config")) {
                const char* name = doc["name"] | "";
                const char* location = doc["location"] | "";
                if (strlen(name) > 0) {
                    strncpy(sensorConfig.sensorName, name, sizeof(sensorConfig.sensorName) - 1);
                    sensorConfig.sensorName[sizeof(sensorConfig.sensorName) - 1] = '\0';
                }
                if (strlen(location) > 0) {
                    strncpy(sensorConfig.location, location, sizeof(sensorConfig.location) - 1);
                    sensorConfig.location[sizeof(sensorConfig.location) - 1] = '\0';
                }
                saveConfig();
                return;
            }

            if (t.endsWith("/command")) {
                const char* action = doc["action"] | "";
                if (strcmp(action, "restart") == 0) {
                    Serial.println("Reinicio solicitado via MQTT");
                    delay(500);
                    ESP.restart();
                }
                if (strcmp(action, "update") == 0 || strcmp(action, "ota") == 0) {
                    const char* url = doc["url"] | "";
                    if (strlen(url) > 0) {
                        performOTAUpload(String(url));
                    }
                }
            }
        });

        // Client ID unico com MAC
        String clientId = "ESP32_Temp_" + deviceMacTopic;

        if (mqttClient.connect(clientId.c_str(), mqttConfig.username, mqttConfig.password)) {
            Serial.println(" Conectado!");
            mqttFailCount = 0;
            lastHeartbeat = 0;
            
            // Anunciar dispositivo para o servidor
            String announceTopic = "devices/announce";
            String announcePayload = "{";
            announcePayload += "\"mac\":\"" + deviceMac + "\",";
            announcePayload += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
            announcePayload += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
            announcePayload += "\"sensor\":\"" + String(sensorConfig.sensorName) + "\",";
            announcePayload += "\"location\":\"" + String(sensorConfig.location) + "\",";
            announcePayload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
            announcePayload += "\"heap\":" + String(esp_get_free_heap_size()) + ",";
            announcePayload += "\"uptime\":" + String(millis() / 1000);
            announcePayload += "}";
            
            Serial.println("📤 Enviando anúncio ao tópico: " + announceTopic);
            Serial.println("   Tamanho: " + String(announcePayload.length()) + " bytes");
            Serial.println("   Payload: " + announcePayload);
            
            // Publicar com QoS 1 e retained para garantir entrega
            if (mqttClient.publish(announceTopic.c_str(), announcePayload.c_str(), true)) {
                Serial.println("✅ Dispositivo anunciado ao servidor com sucesso!");
            } else {
                Serial.println("⚠️ Falha ao anunciar dispositivo ao servidor");
                Serial.println("   Estado MQTT: " + String(mqttClient.state()));
                Serial.println("   Buffer Size: " + String(mqttClient.getBufferSize()));
            }
            
            // Subscrever tópico de configuração
            String cfgTopic = "devices/" + deviceMacTopic + "/config";
            mqttClient.subscribe(cfgTopic.c_str());
            Serial.println("📥 Inscrito no tópico: " + cfgTopic);

            // Subscrever tópico de comandos
            String cmdTopic = "devices/" + deviceMacTopic + "/command";
            mqttClient.subscribe(cmdTopic.c_str());
            Serial.println("📥 Inscrito no tópico: " + cmdTopic);

            publishLogBacklog();
        } else {
            Serial.print(" Falha, rc=");
            Serial.println(mqttClient.state());
            mqttFailCount++;
            if (mqttFailCount >= 10) {
              Serial.println("❌ MQTT falhou 10 vezes. Reiniciando...");
              shouldReboot = true;
            }
        }
    }
}

float readTemperature() {
  if (!sensorFound) {
    return NAN;
  }
  
  sensors.requestTemperatures();
  float tempC = sensors.getTempC(tempSensor);
  
  // Verificar se a leitura é válida
  if (tempC == DEVICE_DISCONNECTED_C) {
    return NAN;
  }
  
  return tempC;
}

void printCurrentConfig() {
  Serial.println("=== CONFIGURAÇÃO ATUAL MQTT ===");
  Serial.println("Servidor: " + String(mqttConfig.server));
  Serial.println("Porta: " + String(mqttConfig.port));
  Serial.println("Usuário: " + String(mqttConfig.username));
  Serial.println("Tópico: " + String(mqttConfig.topic));
  Serial.println("================================");
}

void readAndSendTemperature() {
  float temperature = readTemperature();
  
  if (!isnan(temperature)) {
    // Publicar via MQTT
    connectMQTT();
    
    // Obter informações adicionais
    unsigned long uptime = millis() / 1000;
    int rssi = WiFi.RSSI();
    
    String payload = "{\"temperature\":" + String(temperature, 2) + 
                     ",\"version\":\"" + String(FIRMWARE_VERSION) + "\"" +
                     ",\"sensor\":\"" + String(sensorConfig.sensorName) + "\"" +
                     ",\"location\":\"" + String(sensorConfig.location) + "\"" +
                     ",\"ip\":\"" + WiFi.localIP().toString() + "\"" +
                     ",\"uptime\":" + String(uptime) +
                     ",\"rssi\":" + String(rssi) +
                     ",\"heap\":" + String(esp_get_free_heap_size()) + "}";
    
    if (mqttClient.connected()) {
      bool published = mqttClient.publish(mqttConfig.topic, payload.c_str());
      if (published) {
        Serial.println("✅ Dados enviados: " + payload);
      } else {
        Serial.println("❌ Falha ao publicar MQTT");
      }
    } else {
      Serial.println("⚠️ MQTT desconectado, dados não enviados");
    }
  } else {
    Serial.println("❌ Erro na leitura do sensor DS18B20");
  }
}

void sendHeartbeat() {
    if (!mqttClient.connected()) return;

    String payload = "{";
    payload += "\"mac\":\"" + deviceMac + "\",";
    payload += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    payload += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
    payload += "\"uptime\":" + String(millis() / 1000) + ",";
    payload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
    payload += "\"heap\":" + String(esp_get_free_heap_size());
    payload += "}";

    mqttClient.publish("devices/heartbeat", payload.c_str());
}

String formatUptime(unsigned long seconds) {
    unsigned long days = seconds / 86400;
    seconds %= 86400;
    unsigned long hours = seconds / 3600;
    seconds %= 3600;
    unsigned long minutes = seconds / 60;
    seconds %= 60;
    
    String result = "";
    if (days > 0) {
        result += String(days) + "d ";
    }
    if (hours > 0 || days > 0) {
        result += String(hours) + "h ";
    }
    if (minutes > 0 || hours > 0 || days > 0) {
        result += String(minutes) + "m ";
    }
    result += String(seconds) + "s";
    
    return result;
}

// Autenticação web
bool isAuthenticated() {
  //Serial.println("=== TENTATIVA DE AUTENTICAÇÃO ===");
  //Serial.print("Usuário esperado: ");
  //Serial.println(webAuthConfig.username);
  //Serial.print("Senha esperada: ");
  //Serial.println(webAuthConfig.password);
  
  if (!server.authenticate(webAuthConfig.username, webAuthConfig.password)) {
    //Serial.println("❌ AUTENTICAÇÃO FALHOU");
    server.requestAuthentication();
    return false;
  }
  //Serial.println("✅ AUTENTICAÇÃO BEM-SUCEDIDA");
  return true;
}

// Função para atualização OTA por URL
void performOTAUpload(String url) {
  Serial.println("Iniciando OTA por URL: " + url);

    HTTPClient http;
    WiFiClientSecure secureClient;

    if (url.startsWith("https://")) {
        secureClient.setInsecure();
        http.begin(secureClient, url);
    } else {
        http.begin(url);
    }
  int httpCode = http.GET();
  
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    Serial.println("Tamanho do firmware: " + String(contentLength));
    
    if (contentLength > 0) {
      otaInProgress = true;
      
      if (Update.begin(contentLength)) {
        Serial.println("Iniciando atualização...");
        
        WiFiClient* stream = http.getStreamPtr();
        uint8_t buffer[1024];
        size_t totalRead = 0;
        
        while (http.connected() && totalRead < contentLength) {
          size_t read = stream->readBytes(buffer, min((size_t)1024, contentLength - totalRead));
          if (read > 0) {
            Update.write(buffer, read);
            totalRead += read;
            Serial.printf("Progresso: %d%%\n", (totalRead * 100) / contentLength);
          }
        }
        
        if (Update.end()) {
          Serial.println("Atualização OTA concluída com sucesso!");
          Serial.println("Reiniciando...");
          delay(1000);
          ESP.restart();
        } else {
          Serial.println("Erro na atualização OTA");
          Update.printError(Serial);
        }
      } else {
        Serial.println("Não foi possível iniciar a atualização");
      }
    }
  } else {
    Serial.println("Falha no download: " + String(httpCode));
  }
  
  http.end();
  otaInProgress = false;
}

// Função para reset de fábrica
void performFactoryReset() {
  Serial.println("🚨 INICIANDO RESET DE FÁBRICA 🚨");
  Serial.println("Limpando toda a EEPROM...");
  
  // Limpar toda a EEPROM
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++) {
    EEPROM.write(i, 0);
  }
  EEPROM.commit();
  
  Serial.println("✅ EEPROM limpa completamente");
  Serial.println("✅ Reset de fábrica concluído");
  Serial.println("✅ Sistema reiniciando em modo AP...");
  
  delay(3000);
  ESP.restart();
}

// Servidor Web com Interface Moderna
void setupWebServer()
{
  server.on("/", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    float temp = readTemperature();
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP32 Temperature Monitor</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
        }
        .header {
            background: white;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 30px;
            text-align: center;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #6366f1;
        }
        .header h1 {
            color: #6366f1;
            margin-bottom: 10px;
            font-size: 2.5em;
        }
        .nav {
            display: flex;
            gap: 10px;
            flex-wrap: wrap;
            justify-content: center;
            margin-top: 20px;
        }
        .nav-btn {
            padding: 12px 20px;
            background: #6366f1;
            color: white;
            text-decoration: none;
            border-radius: 6px;
            font-weight: bold;
            transition: background 0.3s;
        }
        .nav-btn:hover { background: #4f46e5; }
        .nav-btn.wifi { background: #10b981; }
        .nav-btn.wifi:hover { background: #0da271; }
        .nav-btn.mqtt { background: #8b5cf6; }
        .nav-btn.mqtt:hover { background: #7c3aed; }
        .nav-btn.firmware { background: #f59e0b; }
        .nav-btn.firmware:hover { background: #d97706; }
        .nav-btn.admin { background: #ec4899; }
        .nav-btn.admin:hover { background: #db2777; }
        .nav-btn.logout { background: #dc2626; }
        .nav-btn.logout:hover { background: #b91c1c; }
        
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 25px;
            margin-bottom: 30px;
        }
        .card {
            background: white;
            padding: 25px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-top: 4px solid #6366f1;
        }
        .card h2 {
            color: #333;
            margin-bottom: 15px;
            font-size: 1.4em;
        }
        .value {
            font-size: 3em;
            font-weight: bold;
            text-align: center;
            margin: 20px 0;
            color: #dc2626;
        }
        .status-badge {
            display: inline-block;
            padding: 8px 15px;
            border-radius: 20px;
            font-weight: bold;
            margin-bottom: 15px;
            font-size: 0.9em;
        }
        .status-online { background: #d1fae5; color: #065f46; }
        .status-offline { background: #fee2e2; color: #991b1b; }
        .status-ap { background: #fef3c7; color: #92400e; }
        .info-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-top: 20px;
        }
        .info-item {
            display: flex;
            flex-direction: column;
            gap: 5px;
        }
        .info-label {
            font-size: 0.85em;
            color: #6b7280;
            font-weight: 600;
        }
        .info-value {
            font-weight: 700;
            color: #1f2937;
        }
        @media (max-width: 768px) {
            .grid { grid-template-columns: 1fr; }
            .nav { flex-direction: column; }
            .nav-btn { text-align: center; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🌡️ Temperature Monitor</h1>
            <div class="nav">
                <a href="/" class="nav-btn">📊 Dashboard</a>
                <a href="/wifi" class="nav-btn wifi">📶 WiFi</a>
                <a href="/mqtt" class="nav-btn mqtt">📡 MQTT</a>
                <a href="/firmware" class="nav-btn firmware">⚡ Firmware</a>
                <a href="/admin" class="nav-btn admin">⚙️ Admin</a>
                <a href="/logout" class="nav-btn logout">🚪 Sair</a>
            </div>
        </div>

        <div class="grid">
            <div class="card">
                <h2>📊 Status do Sistema</h2>
                )=====";

    if (isAPMode) {
        html += "<span class='status-badge status-ap'>🔴 Modo AP Ativo</span>";
        html += "<p>Conecte-se ao WiFi <strong>ESP32-Temperature</strong> e configure a rede.</p>";
    } else {
        html += "<span class='status-badge " + String(mqttClient.connected() ? "status-online" : "status-offline") + "'>";
        html += String(mqttClient.connected() ? "🟢 MQTT Conectado" : "🔴 MQTT Desconectado");
        html += "</span>";
        html += "<p>Sistema operando normalmente em modo WiFi.</p>";
    }
    
    html += "<div class='info-grid'>";
    html += "<div class='info-item'><span class='info-label'>IP</span><span class='info-value'>" + (isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "</span></div>";
    html += "<div class='info-item'><span class='info-label'>Modo</span><span class='info-value'>" + String(isAPMode ? "AP" : "WiFi Station") + "</span></div>";
    
    if (!isAPMode) {
        html += "<div class='info-item'><span class='info-label'>WiFi RSSI</span><span class='info-value'>" + String(WiFi.RSSI()) + " dBm</span></div>";
        html += "<div class='info-item'><span class='info-label'>Qualidade</span><span class='info-value'>";
        int rssi = WiFi.RSSI();
        if (rssi > -50) html += "Excelente 📶";
        else if (rssi > -60) html += "Bom 📶";
        else if (rssi > -70) html += "Regular 📶";
        else html += "Fraco 📶";
        html += "</span></div>";
    }
    
    html += "<div class='info-item'><span class='info-label'>Uptime</span><span class='info-value'>" + formatUptime(millis()/1000) + "</span></div>";
    html += "<div class='info-item'><span class='info-label'>RAM Livre</span><span class='info-value'>" + String(esp_get_free_heap_size()/1024) + " KB</span></div>";
    html += "</div></div>";

    if (!isnan(temp)) {
        html += "<div class='card'><h2>🌡️ Temperatura</h2>";
        html += "<div class='value'>" + String(temp, 1) + "°C</div>";
        html += "<div style='text-align: center; color: #6b7280;'>Atualizado agora</div></div>";

        html += "<div class='card'><h2>🔍 Dados do Sensor</h2>";
        html += "<span class='status-badge status-online'>🟢 Sensor Conectado</span>";
        html += "<div class='info-grid'>";
        html += "<div class='info-item'><span class='info-label'>Nome</span><span class='info-value'>" + String(sensorConfig.sensorName) + "</span></div>";
        html += "<div class='info-item'><span class='info-label'>Localização</span><span class='info-value'>" + String(sensorConfig.location) + "</span></div>";
        html += "</div></div>";
    } else {
        html += "<div class='card'><h2>❌ Erro no Sensor</h2>";
        html += "<div style='text-align: center; padding: 20px;'>";
        html += "<div style='font-size: 4em; margin-bottom: 20px;'>⚠️</div>";
        html += "<p style='color: #dc2626; font-weight: 600;'>Sensor DS18B20 não detectado</p>";
        html += "</div></div>";
    }

    html += R"=====(
        </div>
    </div>
    <script>
        setTimeout(() => location.reload(), 30000);
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

    server.on("/logs", HTTP_GET, []() {
        if (!isAuthenticated()) return;

        unsigned long sinceSeq = 0;
        if (server.hasArg("since")) {
            sinceSeq = strtoul(server.arg("since").c_str(), nullptr, 10);
        }
        sendLogSnapshot(sinceSeq);
    });

server.on("/wifi", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configurar WiFi - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
        }
        .card {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #10b981;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .header h1 {
            color: #10b981;
            margin-bottom: 10px;
        }
        .form-group {
            margin-bottom: 20px;
        }
        .form-label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #333;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            font-size: 16px;
        }
        .form-input:focus {
            outline: none;
            border-color: #10b981;
        }
        .btn-group {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }
        .btn {
            flex: 1;
            padding: 15px;
            border: none;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
            font-size: 16px;
            text-decoration: none;
            text-align: center;
        }
        .btn-primary {
            background: #10b981;
            color: white;
        }
        .btn-primary:hover { background: #0da271; }
        .btn-secondary {
            background: #6b7280;
            color: white;
        }
        .btn-secondary:hover { background: #4b5563; }
        .alert {
            padding: 15px;
            border-radius: 6px;
            margin-bottom: 20px;
            font-weight: 600;
        }
        .alert-info {
            background: #d1fae5;
            color: #065f46;
            border: 1px solid #a7f3d0;
        }
        .wifi-list {
            margin: 20px 0;
            max-height: 300px;
            overflow-y: auto;
            border: 1px solid #e5e7eb;
            border-radius: 6px;
        }
        .wifi-item {
            padding: 12px 15px;
            border-bottom: 1px solid #e5e7eb;
            cursor: pointer;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .wifi-item:hover {
            background: #f3f4f6;
        }
        .wifi-item:last-child {
            border-bottom: none;
        }
        .wifi-signal {
            font-size: 18px;
        }
        .btn-scan {
            width: 100%;
            margin-bottom: 10px;
            background: #3b82f6;
            color: white;
        }
        .btn-scan:hover {
            background: #2563eb;
        }
        .loading {
            text-align: center;
            padding: 20px;
            color: #6b7280;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="header">
                <h1>📶 Configurar WiFi</h1>
                <p>Configure a rede WiFi para conectar o ESP32</p>
            </div>
            
            <button onclick="scanWiFi()" class="btn btn-scan" id="scanBtn">🔍 Escanear Redes WiFi</button>
            <div id="wifiList" style="display:none;"></div>
            
            <div class="alert alert-info">💡 Clique em uma rede ou digite manualmente</div>
            <form method="POST" action="/savewifi">
                <div class="form-group">
                    <label class="form-label">Nome da Rede (SSID)</label>
                    <input type="text" class="form-input" id="ssidInput" name="ssid" value=")=====";
    html += String(wifiConfig.ssid);
    html += R"=====(" placeholder="Ex: MinhaRedeWiFi" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Senha da Rede</label>
                    <input type="password" class="form-input" name="password" value=")=====";
    html += String(wifiConfig.password);
    html += R"=====(" placeholder="Senha do WiFi">
                </div>
                <div class="btn-group">
                    <button type="submit" class="btn btn-primary">💾 Salvar & Reiniciar</button>
                    <a href="/" class="btn btn-secondary">↩️ Cancelar</a>
                </div>
            </form>
        </div>
    </div>
    
    <script>
        function scanWiFi() {
            const btn = document.getElementById('scanBtn');
            const listDiv = document.getElementById('wifiList');
            
            btn.disabled = true;
            btn.textContent = '⏳ Escaneando...';
            listDiv.innerHTML = '<div class="loading">🔍 Procurando redes WiFi...</div>';
            listDiv.style.display = 'block';
            
            fetch('/scanwifi')
                .then(response => response.json())
                .then(data => {
                    btn.disabled = false;
                    btn.textContent = '🔍 Escanear Redes WiFi';
                    
                    if (data.count === 0) {
                        listDiv.innerHTML = '<div class="loading">❌ Nenhuma rede encontrada</div>';
                        return;
                    }
                    
                    let html = '<div class="wifi-list">';
                    data.networks.forEach(network => {
                        const signal = network.rssi > -50 ? '📶📶📶📶' : 
                                      network.rssi > -60 ? '📶📶📶' : 
                                      network.rssi > -70 ? '📶📶' : '📶';
                        const lock = network.encryption != 7 ? '🔒' : '🔓';
                        html += `<div class="wifi-item" onclick="selectNetwork('${network.ssid}')">
                                   <span>${lock} ${network.ssid}</span>
                                   <span class="wifi-signal">${signal} ${network.rssi}dBm</span>
                                 </div>`;
                    });
                    html += '</div>';
                    listDiv.innerHTML = html;
                })
                .catch(error => {
                    btn.disabled = false;
                    btn.textContent = '🔍 Escanear Redes WiFi';
                    listDiv.innerHTML = '<div class="loading">❌ Erro ao escanear redes</div>';
                    console.error('Erro:', error);
                });
        }
        
        function selectNetwork(ssid) {
            document.getElementById('ssidInput').value = ssid;
            document.getElementById('wifiList').style.display = 'none';
        }
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

  // Rota para escanear redes WiFi disponíveis
  server.on("/scanwifi", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    Serial.println("🔍 Iniciando scan de redes WiFi...");
    int n = WiFi.scanNetworks();
    
    String json = "{\"networks\":[";
    
    if (n == 0) {
      json += "],\"count\":0}";
    } else {
      for (int i = 0; i < n; i++) {
        if (i > 0) json += ",";
        json += "{";
        json += "\"ssid\":\"" + WiFi.SSID(i) + "\",";
        json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
        json += "\"encryption\":" + String(WiFi.encryptionType(i));
        json += "}";
      }
      json += "],\"count\":" + String(n) + "}";
    }
    
    Serial.println("✅ Scan concluído: " + String(n) + " redes encontradas");
    server.send(200, "application/json", json);
    
    // Limpar resultados do scan
    WiFi.scanDelete();
  });

  server.on("/savewifi", HTTP_POST, []() {
    if (!isAuthenticated()) return;
    
    String newSSID = server.arg("ssid");
    String newPassword = server.arg("password");
    
    // Validar comprimentos para evitar buffer overflow
    if (newSSID.length() > 0 && newSSID.length() < 32 && newPassword.length() < 64) {
      strcpy(wifiConfig.ssid, newSSID.c_str());
      strcpy(wifiConfig.password, newPassword.c_str());
      saveConfig();
      
      String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configuração Salva - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            text-align: center;
            max-width: 500px;
        }
        .success-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }
        h1 {
            color: #10b981;
            margin-bottom: 20px;
        }
    </style>
</head>
<body>
    <div class="card">
        <div class="success-icon">✅</div>
        <h1>Configuração Salva!</h1>
        <p>WiFi configurado para: <strong>)=====";
      html += newSSID;
      html += R"=====(</strong></p>
        <p>Reiniciando em 5 segundos...</p>
        <p>Conecte-se à rede <strong>)=====";
      html += newSSID;
      html += R"=====(</strong> e acesse o novo IP do ESP32</p>
    </div>
    <script>setTimeout(() => location.href='/', 5000);</script>
</body>
</html>
      )=====";
      
      server.send(200, "text/html", html);
      
      Serial.println("Nova configuração WiFi salva: " + newSSID);
      Serial.println("Reiniciando em 3 segundos...");
      delay(3000);
      shouldReboot = true;
    } else {
      server.send(400, "text/plain", "SSID não pode estar vazio");
    }
  });

server.on("/mqtt", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configurar MQTT - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
        }
        .card {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #8b5cf6;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .header h1 {
            color: #8b5cf6;
            margin-bottom: 10px;
        }
        .form-group { margin-bottom: 20px; }
        .form-label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #333;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            font-size: 16px;
        }
        .form-input:focus {
            outline: none;
            border-color: #8b5cf6;
        }
        .btn-group {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }
        .btn {
            flex: 1;
            padding: 15px;
            border: none;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
            font-size: 16px;
            text-decoration: none;
            text-align: center;
        }
        .btn-primary {
            background: #8b5cf6;
            color: white;
        }
        .btn-primary:hover { background: #7c3aed; }
        .btn-secondary {
            background: #6b7280;
            color: white;
        }
        .btn-secondary:hover { background: #4b5563; }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="header">
                <h1>📡 Configurar MQTT</h1>
                <p>Configure o servidor MQTT para envio de dados</p>
            </div>
            <form method="POST" action="/savemqtt">
                <div class="form-group">
                    <label class="form-label">Servidor MQTT</label>
                    <input type="text" class="form-input" name="server" value=")=====";
    html += String(mqttConfig.server);
    html += R"=====(" placeholder="mqtt.seudominio.com" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Porta</label>
                    <input type="number" class="form-input" name="port" value=")=====";
    html += String(mqttConfig.port);
    html += R"=====(" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Usuário (Opcional)</label>
                    <input type="text" class="form-input" name="username" value=")=====";
    html += String(mqttConfig.username);
    html += R"=====(" placeholder="Usuário MQTT">
                </div>
                <div class="form-group">
                    <label class="form-label">Senha (Opcional)</label>
                    <input type="password" class="form-input" name="password" value=")=====";
    html += String(mqttConfig.password);
    html += R"=====(" placeholder="Senha MQTT">
                </div>
                <div class="form-group">
                    <label class="form-label">Tópico</label>
                    <input type="text" class="form-input" name="topic" value=")=====";
    html += String(mqttConfig.topic);
    html += R"=====(" placeholder="esp32/temperature" required>
                </div>
                <div class="btn-group">
                    <button type="submit" class="btn btn-primary">💾 Salvar Configuração</button>
                    <a href="/" class="btn btn-secondary">↩️ Cancelar</a>
                </div>
            </form>
        </div>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

  server.on("/savemqtt", HTTP_POST, []() {
    if (!isAuthenticated()) return;
    
    Serial.println("\n=== RECEBENDO CONFIGURAÇÃO MQTT ===");
    
    // Validar comprimentos para evitar buffer overflow
    String server_str = server.arg("server");
    String username_str = server.arg("username");
    String password_str = server.arg("password");
    String topic_str = server.arg("topic");
    int port_val = server.arg("port").toInt();
    
    Serial.println("Dados recebidos do formulário:");
    Serial.println("  Server: " + server_str);
    Serial.println("  Port: " + String(port_val));
    Serial.println("  Username: " + username_str);
    Serial.println("  Topic: " + topic_str);
    
    if (server_str.length() >= 64 || username_str.length() >= 32 || 
        password_str.length() >= 64 || topic_str.length() >= 64) {
      Serial.println("❌ ERRO: Tamanho de campo excedido!");
      server.send(400, "text/plain", "Erro: Um ou mais campos excedem o tamanho máximo permitido");
      return;
    }
    
    // Limpar as estruturas antes de copiar novos dados
    memset(mqttConfig.server, 0, sizeof(mqttConfig.server));
    memset(mqttConfig.username, 0, sizeof(mqttConfig.username));
    memset(mqttConfig.password, 0, sizeof(mqttConfig.password));
    memset(mqttConfig.topic, 0, sizeof(mqttConfig.topic));
    
    // Copiar novos dados
    strcpy(mqttConfig.server, server_str.c_str());
    mqttConfig.port = port_val;
    strcpy(mqttConfig.username, username_str.c_str());
    strcpy(mqttConfig.password, password_str.c_str());
    strcpy(mqttConfig.topic, topic_str.c_str());
    
    Serial.println("\nDados copiados para mqttConfig:");
    Serial.println("  Server: " + String(mqttConfig.server));
    Serial.println("  Port: " + String(mqttConfig.port));
    Serial.println("  Username: " + String(mqttConfig.username));
    Serial.println("  Topic: " + String(mqttConfig.topic));
    
    // Salvar na EEPROM
    saveConfig();
    
    // FORÇAR RECONEXÃO MQTT
  if (!isAPMode) {
    mqttClient.disconnect();
    delay(1000);
    mqttClient.setServer(mqttConfig.server, mqttConfig.port);
    connectMQTT();
  }
    
String html = R"=====(
  <!DOCTYPE html>
  <html lang="pt-BR">
  <head>
      <meta charset="UTF-8">
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <title>Configuração Salva - ESP32 Temperature</title>
      <style>
          body {
              font-family: 'Segoe UI', system-ui, sans-serif;
              background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
              min-height: 100vh;
              padding: 20px;
              color: #1f2937;
              display: flex;
              align-items: center;
              justify-content: center;
          }
          .card {
              background: rgba(255, 255, 255, 0.95);
              backdrop-filter: blur(10px);
              border-radius: 20px;
              padding: 50px;
              box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
              border: 1px solid rgba(255, 255, 255, 0.2);
              text-align: center;
              max-width: 500px;
          }
          .success-icon {
              font-size: 4em;
              margin-bottom: 20px;
          }
          h1 {
              color: #10b981;
              margin-bottom: 20px;
          }
          .info-box {
              background: #f0f9ff;
              padding: 15px;
              border-radius: 10px;
              margin: 20px 0;
              text-align: left;
          }
          .btn {
              display: inline-block;
              padding: 15px 30px;
              background: #6366f1;
              color: white;
              text-decoration: none;
              border-radius: 50px;
              font-weight: 600;
              margin-top: 20px;
              transition: all 0.3s ease;
          }
          .btn:hover {
              background: #4f46e5;
              transform: translateY(-2px);
          }
      </style>
  </head>
  <body>
      <div class="card">
          <div class="success-icon">✅</div>
          <h1>Configuração MQTT Salva!</h1>
          
          <div class="info-box">
              <p><strong>Servidor:</strong> )=====";
  html += String(mqttConfig.server);
  html += R"=====(</p>
              <p><strong>Porta:</strong> )=====";
  html += String(mqttConfig.port);
  html += R"=====(</p>
              <p><strong>Usuário:</strong> )=====";
  html += String(mqttConfig.username);
  html += R"=====(</p>
              <p><strong>Tópico:</strong> )=====";
  html += String(mqttConfig.topic);
  html += R"=====(</p>
          </div>
          
          <p>Configurações salvas e reconectando ao MQTT...</p>
          
          <a href="/" class="btn">📊 Voltar ao Dashboard</a>
          <a href="/mqtt" class="btn" style="background: #8b5cf6;">📡 Configurar Novamente</a>
          <a href="/testmqtt" class="btn" style="background: #10b981;">🧪 Testar Conexão</a>
      </div>
  </body>
  </html>
  )=====";
  
  server.send(200, "text/html", html);
  
  Serial.println("✅ Configuração MQTT atualizada:");
  Serial.println("   Servidor: " + String(mqttConfig.server));
  Serial.println("   Porta: " + String(mqttConfig.port));
  Serial.println("   Usuário: " + String(mqttConfig.username));
  Serial.println("   Tópico: " + String(mqttConfig.topic));
});

// Rota para testar conexão MQTT
server.on("/testmqtt", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String result = "";
    bool success = false;
    
    Serial.println("\n=== TESTE DE CONEXÃO MQTT ===");
    
    // Desconectar e tentar reconectar
    mqttClient.disconnect();
    delay(500);
    mqttClient.setServer(mqttConfig.server, mqttConfig.port);
    
    if (mqttClient.connect("ESP32_Temp_Test", mqttConfig.username, mqttConfig.password)) {
        Serial.println("✅ Conexão MQTT bem-sucedida!");
        
        // Tentar publicar mensagem de teste
        String testPayload = "{\"test\":true,\"message\":\"Teste de conexão\",\"timestamp\":" + String(millis()) + "}";
        if (mqttClient.publish(mqttConfig.topic, testPayload.c_str())) {
            result = "✅ Conexão e publicação bem-sucedidas!";
            success = true;
            Serial.println("✅ Mensagem de teste publicada");
        } else {
            result = "⚠️ Conectado, mas falha ao publicar mensagem";
            Serial.println("⚠️ Falha ao publicar");
        }
    } else {
        int state = mqttClient.state();
        result = "❌ Falha na conexão. Código de erro: " + String(state);
        Serial.print("❌ Falha na conexão MQTT. Estado: ");
        Serial.println(state);
        
        // Decodificar erro
        switch(state) {
            case -4: result += " (Timeout)"; break;
            case -3: result += " (Conexão perdida)"; break;
            case -2: result += " (Falha na conexão)"; break;
            case -1: result += " (Desconectado)"; break;
            case 1: result += " (Protocolo incorreto)"; break;
            case 2: result += " (ID rejeitado)"; break;
            case 3: result += " (Servidor indisponível)"; break;
            case 4: result += " (Credenciais inválidas)"; break;
            case 5: result += " (Não autorizado)"; break;
        }
    }
    
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Teste MQTT - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            text-align: center;
            max-width: 600px;
        }
        .test-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }
        h1 {
            color: #8b5cf6;
            margin-bottom: 20px;
        }
        .result-box {
            background: )=====";
    html += success ? "#d1fae5" : "#fee2e2";
    html += R"=====(;
            padding: 20px;
            border-radius: 10px;
            margin: 20px 0;
            font-weight: 600;
            color: )=====";
    html += success ? "#065f46" : "#991b1b";
    html += R"=====(;
        }
        .btn {
            display: inline-block;
            padding: 15px 30px;
            background: #6366f1;
            color: white;
            text-decoration: none;
            border-radius: 50px;
            font-weight: 600;
            margin: 10px;
            transition: all 0.3s ease;
        }
        .btn:hover {
            background: #4f46e5;
            transform: translateY(-2px);
        }
    </style>
</head>
<body>
    <div class="card">
        <div class="test-icon">🧪</div>
        <h1>Teste de Conexão MQTT</h1>
        
        <div class="result-box">
            )=====";
    html += result;
    html += R"=====(
        </div>
        
        <p><strong>Servidor:</strong> )=====";
    html += String(mqttConfig.server);
    html += R"=====(</p>
        <p><strong>Porta:</strong> )=====";
    html += String(mqttConfig.port);
    html += R"=====(</p>
        
        <a href="/mqtt" class="btn">📡 Configurar MQTT</a>
        <a href="/" class="btn">📊 Dashboard</a>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

server.on("/firmware", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Atualizar Firmware - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 800px;
            margin: 0 auto;
        }
        .card {
            background: white;
            padding: 25px;
            border-radius: 10px;
            margin-bottom: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #f59e0b;
        }
        .alert {
            padding: 15px;
            border-radius: 6px;
            margin-bottom: 20px;
            font-weight: 600;
        }
        .alert-warning {
            background: #fef3c7;
            color: #92400e;
            border: 1px solid #f59e0b;
        }
        .alert-info {
            background: #dbeafe;
            color: #1e40af;
            border: 1px solid #3b82f6;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            margin: 10px 0;
            font-size: 16px;
        }
        .form-input:focus {
            outline: none;
            border-color: #f59e0b;
        }
        .btn {
            padding: 15px 25px;
            background: #f59e0b;
            color: white;
            border: none;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
            font-size: 16px;
            text-decoration: none;
            display: inline-block;
            text-align: center;
        }
        .btn:hover { background: #d97706; }
        .btn-secondary {
            background: #6b7280;
        }
        .btn-secondary:hover { background: #4b5563; }
        .info-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 15px;
            margin-top: 20px;
        }
        .info-item {
            display: flex;
            flex-direction: column;
            gap: 5px;
        }
        .info-label {
            font-size: 0.85em;
            color: #6b7280;
            font-weight: 600;
        }
        .info-value {
            font-weight: 700;
            color: #1f2937;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <h1>⚡ Atualizar Firmware</h1>
            <div class="alert alert-warning">⚠️ <strong>Atenção:</strong> Não desconecte a energia durante a atualização!</div>
        </div>

        <div class="card">
            <h2>📤 Upload Local</h2>
            <form method="POST" action="/update" enctype="multipart/form-data">
                <input type="file" class="form-input" name="firmware" accept=".bin" required style="border: 2px dashed #ddd; text-align: center;">
                <br>
                <button type="submit" class="btn">🚀 Enviar e Atualizar</button>
            </form>
        </div>

        <div class="card">
            <h2>🌐 Atualização por URL</h2>
            <div class="alert alert-info">💡 <strong>Nota:</strong> URLs HTTPS não são suportadas. Use apenas HTTP.</div>
            <form method="POST" action="/updateurl">
                <input type="url" class="form-input" name="url" placeholder="http://exemplo.com/firmware.bin" required>
                <br>
                <button type="submit" class="btn">📥 Baixar e Atualizar</button>
            </form>
        </div>
        
        <div class="card">
            <h2>📋 Informações do Sistema</h2>
            <div class="info-grid">
                <div class="info-item">
                    <span class="info-label">Versão Atual</span>
                    <span class="info-value">1.0.0</span>
                </div>
                <div class="info-item">
                    <span class="info-label">ESP32 Chip ID</span>
                    <span class="info-value">)=====";
    html += String((uint32_t)ESP.getEfuseMac(), HEX);
    html += R"=====(</span>
                </div>
                <div class="info-item">
                    <span class="info-label">Flash Size</span>
                    <span class="info-value">)=====";
    html += String(ESP.getFlashChipSize() / (1024 * 1024));
    html += R"=====( MB</span>)=====";
    html += String(FIRMWARE_VERSION);
    html += R"=====(
                </div>
                <div class="info-item">
                    <span class="info-label">Sketch Size</span>
                    <span class="info-value">)=====";
    html += String(ESP.getSketchSize() / 1024);
    html += R"=====( KB</span>
                </div>
            </div>
        </div>
        
        <div style="text-align: center; margin-top: 20px;">
            <a href="/" class="btn btn-secondary">↩️ Voltar ao Dashboard</a>
        </div>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

  // Handler para upload local
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "Falha na atualização" : "Atualização concluída!");
    delay(1000);
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      otaInProgress = true;
      Serial.printf("Iniciando atualização: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Atualização concluída: %u bytes\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  // Handler para atualização por URL
  server.on("/updateurl", HTTP_POST, []() {
    if (!isAuthenticated()) return;
    
    String url = server.arg("url");
    if (url.length() > 0) {
      server.send(200, "text/html", 
        "<html><body style='font-family: Arial; margin: 40px; display: flex; align-items: center; justify-content: center; min-height: 100vh; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);'>"
        "<div style='background: white; padding: 50px; border-radius: 20px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); text-align: center;'>"
        "<h1 style='color: #6366f1; margin-bottom: 20px;'>🔄 Atualizando Firmware</h1>"
        "<p><strong>URL:</strong> " + url + "</p>"
        "<p>⏳ Não desligue o dispositivo!</p>"
        "<p>Esta operação pode levar alguns minutos.</p>"
        "</div>"
        "<script>setTimeout(() => { location.href = '/'; }, 30000);</script>"
        "</body></html>");
      
      performOTAUpload(url);
    } else {
      server.send(400, "text/plain", "URL inválida");
    }
  });

// Logout - Versão com Estilo Unificado
server.on("/logout", []() {
  Serial.println("👤 Usuário solicitou logout");
  
  String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Logout - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
        }
        .card {
            background: white;
            padding: 40px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #10b981;
            text-align: center;
        }
        .success-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }
        h1 {
            color: #10b981;
            margin-bottom: 15px;
            font-size: 2.2em;
        }
        p {
            color: #6b7280;
            margin-bottom: 15px;
            line-height: 1.5;
        }
        .btn {
            display: inline-block;
            padding: 15px 30px;
            background: #10b981;
            color: white;
            text-decoration: none;
            border-radius: 6px;
            font-weight: 600;
            margin-top: 25px;
            transition: background 0.3s;
            border: none;
            cursor: pointer;
            font-size: 16px;
        }
        .btn:hover {
            background: #0da271;
        }
        .login-info {
            background: #f0f9ff;
            padding: 15px;
            border-radius: 6px;
            margin: 20px 0;
            border: 1px solid #bae6fd;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="success-icon">✅</div>
            <h1>Logout Realizado!</h1>
            <p>Você foi desconectado com sucesso do sistema.</p>
            
            <div class="login-info">
                <p><strong>Para acessar novamente:</strong></p>
                <p>Clique no botão abaixo e informe suas credenciais</p>
            </div>
            
            <a href="/" class="btn">🔐 Fazer Login Novamente</a>
            
            <div style="margin-top: 25px; font-size: 0.9em; color: #9ca3af;">
                <p>Redirecionando automaticamente em <span id="countdown">10</span> segundos...</p>
            </div>
        </div>
    </div>
    
    <script>
        // Forçar limpeza do cache de autenticação
        function clearAuthCache() {
            // Tentativas de limpar cache de autenticação
            if (window.fetch) {
                fetch('/', { 
                    method: 'GET',
                    headers: {
                        'Authorization': 'Basic ' + btoa('logout:logout')
                    }
                }).catch(() => {});
            }
            
            // Tentar fazer logout via XMLHttpRequest
            var xhr = new XMLHttpRequest();
            xhr.open('GET', '/', true, 'logout', 'logout');
            xhr.send();
        }
        
        // Redirecionamento automático
        function startCountdown() {
            let seconds = 10;
            const countdownElement = document.getElementById('countdown');
            const countdownInterval = setInterval(() => {
                seconds--;
                countdownElement.textContent = seconds;
                
                if (seconds <= 0) {
                    clearInterval(countdownInterval);
                    window.location.href = '/';
                }
            }, 1000);
        }
        
        // Executar quando a página carregar
        document.addEventListener('DOMContentLoaded', function() {
            clearAuthCache();
            startCountdown();
            console.log('Logout realizado - cache limpo');
        });
    </script>
</body>
</html>
)=====";
  
  server.send(200, "text/html", html);
  Serial.println("✅ Página de logout exibida");
});

  // Informações do sistema (JSON)
  server.on("/info", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String json = "{";
    json += "\"version\":\"1.0.0\",";
    json += "\"chip_id\":\"" + String((uint32_t)ESP.getEfuseMac(), HEX) + "\",";
    json += "\"free_heap\":" + String(esp_get_free_heap_size()) + ",";
    json += "\"sketch_size\":" + String(ESP.getSketchSize()) + ",";
    json += "\"free_sketch_space\":" + String(ESP.getFreeSketchSpace()) + ",";
    json += "\"wifi_ssid\":\"" + String(wifiConfig.ssid) + "\",";
    json += "\"ap_mode\":" + String(isAPMode ? "true" : "false") + ",";
    json += "\"ip\":\"" + (isAPMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString()) + "\",";
    json += "\"mqtt_connected\":" + String(mqttClient.connected() ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
  });

  // Handler para página não encontrada
  server.onNotFound([]() {
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Página Não Encontrada - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
            text-align: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }
        h1 {
            font-size: 4em;
            margin-bottom: 20px;
            color: #ef4444;
        }
        .btn {
            display: inline-block;
            padding: 15px 30px;
            background: #6366f1;
            color: white;
            text-decoration: none;
            border-radius: 50px;
            font-weight: 600;
            margin-top: 20px;
            transition: all 0.3s ease;
        }
        .btn:hover {
            background: #4f46e5;
            transform: translateY(-2px);
        }
    </style>
</head>
<body>
    <div class="card">
        <h1>404</h1>
        <h2>Página Não Encontrada</h2>
        <p>A página que você está procurando não existe.</p>
        <a href="/" class="btn">🏠 Voltar ao Dashboard</a>
    </div>
</body>
</html>
    )=====";
    
    server.send(404, "text/html", html);
  });

server.on("/password", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Alterar Senha - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
        }
        .card {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #ec4899;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .header h1 {
            color: #ec4899;
            margin-bottom: 10px;
        }
        .form-group { margin-bottom: 20px; }
        .form-label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #333;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            font-size: 16px;
        }
        .form-input:focus {
            outline: none;
            border-color: #ec4899;
        }
        .btn-group {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }
        .btn {
            flex: 1;
            padding: 15px;
            border: none;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
            font-size: 16px;
            text-decoration: none;
            text-align: center;
        }
        .btn-primary {
            background: #ec4899;
            color: white;
        }
        .btn-primary:hover { background: #db2777; }
        .btn-secondary {
            background: #6b7280;
            color: white;
        }
        .btn-secondary:hover { background: #4b5563; }
        .current-user {
            background: #fdf2f8;
            padding: 15px;
            border-radius: 6px;
            margin-bottom: 20px;
            text-align: center;
            font-weight: 600;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="header">
                <h1>🔐 Alterar Senha</h1>
                <p>Altere o usuário e senha de acesso ao sistema</p>
            </div>
            <div class="current-user">
                <strong>Usuário Atual:</strong> )=====";
    html += String(webAuthConfig.username);
    html += R"=====(
            </div>
            <form method="POST" action="/savepassword">
                <div class="form-group">
                    <label class="form-label">Usuário Atual</label>
                    <input type="text" class="form-input" name="current_username" placeholder="Digite o usuário atual" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Senha Atual</label>
                    <input type="password" class="form-input" name="current_password" placeholder="Digite a senha atual" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Novo Usuário</label>
                    <input type="text" class="form-input" name="new_username" value=")=====";
    html += String(webAuthConfig.username);
    html += R"=====(" placeholder="Novo usuário" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Nova Senha</label>
                    <input type="password" class="form-input" name="new_password" placeholder="Nova senha" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Confirmar Nova Senha</label>
                    <input type="password" class="form-input" name="confirm_password" placeholder="Digite a nova senha novamente" required>
                </div>
                <div class="btn-group">
                    <button type="submit" class="btn btn-primary">💾 Salvar Alterações</button>
                    <a href="/" class="btn btn-secondary">↩️ Cancelar</a>
                </div>
            </form>
        </div>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

  // Handler para salvar nova senha
  server.on("/savepassword", HTTP_POST, []() {
    if (!isAuthenticated()) return;
    
    String currentUsername = server.arg("current_username");
    String currentPassword = server.arg("current_password");
    String newUsername = server.arg("new_username");
    String newPassword = server.arg("new_password");
    String confirmPassword = server.arg("confirm_password");
    
    // Verificar credenciais atuais
    if (strcmp(currentUsername.c_str(), webAuthConfig.username) != 0 || 
        strcmp(currentPassword.c_str(), webAuthConfig.password) != 0) {
      server.send(400, "text/html", 
        "<html><body style='font-family: Arial; margin: 40px; display: flex; align-items: center; justify-content: center; min-height: 100vh; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);'>"
        "<div style='background: white; padding: 50px; border-radius: 20px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); text-align: center;'>"
        "<h1 style='color: #ef4444; margin-bottom: 20px;'>❌ Erro</h1>"
        "<p>Usuário ou senha atual incorretos!</p>"
        "<a href='/password' style='display: inline-block; padding: 10px 20px; background: #6366f1; color: white; text-decoration: none; border-radius: 5px; margin-top: 20px;'>Voltar</a>"
        "</div></body></html>");
      return;
    }
    
    // Verificar se senhas coincidem
    if (newPassword != confirmPassword) {
      server.send(400, "text/html", 
        "<html><body style='font-family: Arial; margin: 40px; display: flex; align-items: center; justify-content: center; min-height: 100vh; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);'>"
        "<div style='background: white; padding: 50px; border-radius: 20px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); text-align: center;'>"
        "<h1 style='color: #ef4444; margin-bottom: 20px;'>❌ Erro</h1>"
        "<p>As novas senhas não coincidem!</p>"
        "<a href='/password' style='display: inline-block; padding: 10px 20px; background: #6366f1; color: white; text-decoration: none; border-radius: 5px; margin-top: 20px;'>Voltar</a>"
        "</div></body></html>");
      return;
    }
    
    // Verificar comprimento
    if (newUsername.length() < 3 || newPassword.length() < 3 ||
        newUsername.length() >= 32 || newPassword.length() >= 64) {
      server.send(400, "text/html", 
        "<html><body style='font-family: Arial; margin: 40px; display: flex; align-items: center; justify-content: center; min-height: 100vh; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);'>"
        "<div style='background: white; padding: 50px; border-radius: 20px; box-shadow: 0 20px 40px rgba(0,0,0,0.1); text-align: center;'>"
        "<h1 style='color: #ef4444; margin-bottom: 20px;'>❌ Erro</h1>"
        "<p>Usuário deve ter entre 3 e 31 caracteres e senha entre 3 e 63 caracteres!</p>"
        "<a href='/password' style='display: inline-block; padding: 10px 20px; background: #6366f1; color: white; text-decoration: none; border-radius: 5px; margin-top: 20px;'>Voltar</a>"
        "</div></body></html>");
      return;
    }
    
    // Salvar novas credenciais
    strcpy(webAuthConfig.username, newUsername.c_str());
    strcpy(webAuthConfig.password, newPassword.c_str());
    saveConfig();
    
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Senha Alterada - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            text-align: center;
            max-width: 500px;
        }
        .success-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }
        h1 {
            color: #10b981;
            margin-bottom: 20px;
        }
        .btn {
            display: inline-block;
            padding: 15px 30px;
            background: #6366f1;
            color: white;
            text-decoration: none;
            border-radius: 50px;
            font-weight: 600;
            margin-top: 20px;
            transition: all 0.3s ease;
        }
        .btn:hover {
            background: #4f46e5;
            transform: translateY(-2px);
        }
        .info-box {
            background: #f0f9ff;
            padding: 15px;
            border-radius: 10px;
            margin: 20px 0;
        }
    </style>
</head>
<body>
    <div class="card">
        <div class="success-icon">✅</div>
        <h1>Senha Alterada!</h1>
        <p>As credenciais de acesso foram atualizadas com sucesso.</p>
        
        <div class="info-box">
            <p><strong>Novo Usuário:</strong> )=====";
    html += newUsername;
    html += R"=====(</p>
            <p><strong>Nova Senha:</strong> ••••••••</p>
        </div>
        
        <p><strong>⚠️ Importante:</strong> Você será desconectado e precisará fazer login novamente com as novas credenciais.</p>
        
        <a href="/logout" class="btn">🚪 Fazer Login Novamente</a>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
    
    Serial.println("Credenciais web atualizadas: " + newUsername);
  });

server.on("/sensor", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configurar Sensor - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
        }
        .card {
            background: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #06b6d4;
        }
        .header {
            text-align: center;
            margin-bottom: 30px;
        }
        .header h1 {
            color: #06b6d4;
            margin-bottom: 10px;
        }
        .form-group { margin-bottom: 20px; }
        .form-label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #333;
        }
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e5e7eb;
            border-radius: 6px;
            font-size: 16px;
        }
        .form-input:focus {
            outline: none;
            border-color: #06b6d4;
        }
        .btn-group {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }
        .btn {
            flex: 1;
            padding: 15px;
            border: none;
            border-radius: 6px;
            font-weight: 600;
            cursor: pointer;
            font-size: 16px;
            text-decoration: none;
            text-align: center;
        }
        .btn-primary {
            background: #06b6d4;
            color: white;
        }
        .btn-primary:hover { background: #0891b2; }
        .btn-secondary {
            background: #6b7280;
            color: white;
        }
        .btn-secondary:hover { background: #4b5563; }
        .current-config {
            background: #f0f9ff;
            padding: 20px;
            border-radius: 6px;
            margin-bottom: 20px;
            text-align: center;
        }
        .config-item {
            margin: 10px 0;
            font-size: 1.1em;
        }
        .config-label {
            font-weight: 600;
            color: #6b7280;
        }
        .config-value {
            font-weight: 700;
            color: #1f2937;
        }
        .btn-announce {
            background: #8b5cf6;
            color: white;
            margin-top: 15px;
        }
        .btn-announce:hover {
            background: #7c3aed;
        }
        #announceResult {
            margin-top: 15px;
            padding: 10px;
            border-radius: 6px;
            display: none;
        }
        .result-success {
            background: #d1fae5;
            color: #065f46;
            border: 1px solid #a7f3d0;
        }
        .result-error {
            background: #fee2e2;
            color: #991b1b;
            border: 1px solid #fecaca;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="header">
                <h1>🔧 Configurar Sensor</h1>
                <p>Personalize o nome e localização do sensor</p>
            </div>
            <div class="current-config">
                <div class="config-item">
                    <span class="config-label">Nome Atual:</span>
                    <span class="config-value">)=====";
    html += String(sensorConfig.sensorName);
    html += R"=====(</span>
                </div>
                <div class="config-item">
                    <span class="config-label">Localização Atual:</span>
                    <span class="config-value">)=====";
    html += String(sensorConfig.location);
    html += R"=====(</span>
                </div>
                <button onclick="announceDevice()" class="btn btn-announce" id="announceBtn">📡 Anunciar ao Servidor</button>
                <div id="announceResult"></div>
            </div>
            <form method="POST" action="/savesensor">
                <div class="form-group">
                    <label class="form-label">Nome do Sensor</label>
                    <input type="text" class="form-input" name="sensor_name" value=")=====";
    html += String(sensorConfig.sensorName);
    html += R"=====(" placeholder="Ex: Sensor_Sala, Termometro_Cozinha" required>
                </div>
                <div class="form-group">
                    <label class="form-label">Localização (opcional)</label>
                    <input type="text" class="form-input" name="location" value=")=====";
    html += String(sensorConfig.location);
    html += R"=====(" placeholder="Ex: Sala, Cozinha, Quarto, Externo">
                </div>
                <div class="btn-group">
                    <button type="submit" class="btn btn-primary">💾 Salvar Configuração</button>
                    <a href="/admin" class="btn btn-secondary">↩️ Cancelar</a>
                </div>
            </form>
        </div>
    </div>
    
    <script>
        function announceDevice() {
            const btn = document.getElementById('announceBtn');
            const resultDiv = document.getElementById('announceResult');
            
            btn.disabled = true;
            btn.textContent = '⏳ Anunciando...';
            resultDiv.style.display = 'none';
            
            fetch('/announce')
                .then(response => response.json())
                .then(data => {
                    btn.disabled = false;
                    btn.textContent = '📡 Anunciar ao Servidor';
                    resultDiv.style.display = 'block';
                    
                    if (data.status === 'success') {
                        resultDiv.className = 'result-success';
                        resultDiv.innerHTML = '✅ ' + data.message + '<br><small>' + data.payload + '</small>';
                    } else {
                        resultDiv.className = 'result-error';
                        resultDiv.innerHTML = '❌ ' + data.message;
                    }
                })
                .catch(error => {
                    btn.disabled = false;
                    btn.textContent = '📡 Anunciar ao Servidor';
                    resultDiv.style.display = 'block';
                    resultDiv.className = 'result-error';
                    resultDiv.innerHTML = '❌ Erro ao anunciar dispositivo';
                    console.error('Erro:', error);
                });
        }
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

  // Handler para salvar configuração do sensor
  server.on("/savesensor", HTTP_POST, []() {
    if (!isAuthenticated()) return;
    
    String newSensorName = server.arg("sensor_name");
    String newLocation = server.arg("location");
    
    // Validar e sanitizar nome do sensor
    bool valid = true;
    if (newSensorName.length() == 0 || newSensorName.length() >= 32) {
      valid = false;
    }
    
    // Validar localização (pode estar vazia)
    if (newLocation.length() >= 64) {
      valid = false;
    }
    
    if (valid) {
      // Limpar buffers antes de copiar
      memset(sensorConfig.sensorName, 0, sizeof(sensorConfig.sensorName));
      memset(sensorConfig.location, 0, sizeof(sensorConfig.location));
      
      // Copiar dados sanitizados
      strncpy(sensorConfig.sensorName, newSensorName.c_str(), sizeof(sensorConfig.sensorName) - 1);
      sensorConfig.sensorName[sizeof(sensorConfig.sensorName) - 1] = '\0';
      
      if (newLocation.length() > 0) {
        strncpy(sensorConfig.location, newLocation.c_str(), sizeof(sensorConfig.location) - 1);
        sensorConfig.location[sizeof(sensorConfig.location) - 1] = '\0';
      }
      
      saveConfig();
      
      Serial.println("✅ Configuração do sensor atualizada:");
      Serial.println("   Nome: " + String(sensorConfig.sensorName));
      Serial.println("   Local: " + String(sensorConfig.location));
      
      String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Configuração Salva - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            text-align: center;
            max-width: 500px;
        }
        .success-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }
        h1 {
            color: #10b981;
            margin-bottom: 20px;
        }
        .btn {
            display: inline-block;
            padding: 15px 30px;
            background: #6366f1;
            color: white;
            text-decoration: none;
            border-radius: 50px;
            font-weight: 600;
            margin-top: 20px;
            transition: all 0.3s ease;
        }
        .btn:hover {
            background: #4f46e5;
            transform: translateY(-2px);
        }
        .info-box {
            background: #f0f9ff;
            padding: 20px;
            border-radius: 10px;
            margin: 20px 0;
        }
    </style>
</head>
<body>
    <div class="card">
        <div class="success-icon">✅</div>
        <h1>Configuração Salva!</h1>
        <p>As configurações do sensor foram atualizadas com sucesso.</p>
        
        <div class="info-box">
            <p><strong>Novo Nome:</strong> )=====";
    html += newSensorName;
    html += R"=====(</p>
            <p><strong>Nova Localização:</strong> )=====";
    html += newLocation;
    html += R"=====(</p>
        </div>
        
        <p>📊 Os próximos dados MQTT incluirão estas informações.</p>
        
        <a href="/" class="btn">📊 Voltar ao Dashboard</a>
        <a href="/sensor" class="btn" style="background: #06b6d4;">🔧 Configurar Novamente</a>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
    
    Serial.println("✅ Configuração do sensor atualizada:");
    Serial.println("   Nome: " + newSensorName);
    Serial.println("   Localização: " + newLocation);
    
    } else {
      server.send(400, "text/plain", "Nome do sensor e localização não podem estar vazios");
    }
  });

    // Página de reinicialização
  server.on("/restart", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Reiniciar Sistema - ESP32 Temperature</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        :root {
            --primary: #6366f1;
            --primary-dark: #4f46e5;
            --secondary: #10b981;
            --danger: #ef4444;
            --warning: #f59e0b;
            --dark: #1f2937;
            --light: #f8fafc;
            --gray: #6b7280;
            --gray-light: #e5e7eb;
        }

        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: var(--dark);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .container {
            max-width: 500px;
            width: 100%;
        }

        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            text-align: center;
        }

        .header {
            text-align: center;
            margin-bottom: 30px;
        }

        .header h1 {
            font-size: 2em;
            margin-bottom: 10px;
            background: linear-gradient(135deg, #ef4444, #dc2626);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .alert {
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 25px;
            font-weight: 600;
        }

        .alert-warning {
            background: #fef3c7;
            color: #92400e;
            border: 2px solid #f59e0b;
        }

        .btn-group {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }

        .btn {
            flex: 1;
            padding: 15px 30px;
            border: none;
            border-radius: 50px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            font-size: 16px;
            text-decoration: none;
            text-align: center;
        }

        .btn-danger {
            background: #ef4444;
            color: white;
        }

        .btn-danger:hover {
            background: #dc2626;
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(239, 68, 68, 0.3);
        }

        .btn-secondary {
            background: var(--gray);
            color: white;
        }

        .btn-secondary:hover {
            background: #4b5563;
            transform: translateY(-2px);
        }

        .restart-icon {
            font-size: 4em;
            margin-bottom: 20px;
        }

        .countdown {
            font-size: 1.2em;
            font-weight: 600;
            color: #ef4444;
            margin: 20px 0;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="restart-icon">⚡</div>
            
            <div class="header">
                <h1>Reiniciar Sistema</h1>
                <p>Reinicialização do ESP32</p>
            </div>

            <div class="alert alert-warning">
                ⚠️ <strong>Atenção:</strong> O sistema será reiniciado. 
                Todas as conexões ativas serão interrompidas temporariamente.
            </div>

            <p>Esta operação reiniciará completamente o ESP32. 
               O sistema voltará ao normal em aproximadamente 30 segundos.</p>

            <div class="btn-group">
                <button onclick="confirmRestart()" class="btn btn-danger">🔁 Reiniciar Agora</button>
                <a href="/admin" class="btn btn-secondary">↩️ Cancelar</a>
            </div>

            <div id="countdown" class="countdown" style="display: none;"></div>
        </div>
    </div>

    <script>
        function confirmRestart() {
            if (confirm('Tem certeza que deseja reiniciar o sistema?')) {
                // Mostrar contagem regressiva
                const countdownEl = document.getElementById('countdown');
                const btnGroup = document.querySelector('.btn-group');
                btnGroup.style.display = 'none';
                countdownEl.style.display = 'block';
                
                let seconds = 5;
                const countdownInterval = setInterval(() => {
                    countdownEl.innerHTML = `Reiniciando em ${seconds} segundos...`;
                    seconds--;
                    
                    if (seconds < 0) {
                        clearInterval(countdownInterval);
                        // Fazer a requisição para reiniciar
                        window.location.href = '/dorestart';
                    }
                }, 1000);
            }
        }
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
  });

  // Handler para executar o reinício
  server.on("/dorestart", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Reiniciando - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
            text-align: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            max-width: 500px;
        }
        .restart-icon {
            font-size: 4em;
            margin-bottom: 20px;
            animation: pulse 1s infinite;
        }
        @keyframes pulse {
            0% { transform: scale(1); }
            50% { transform: scale(1.1); }
            100% { transform: scale(1); }
        }
        h1 {
            color: #ef4444;
            margin-bottom: 20px;
        }
        .loading-bar {
            width: 100%;
            height: 8px;
            background: #e5e7eb;
            border-radius: 4px;
            margin: 30px 0;
            overflow: hidden;
        }
        .loading-progress {
            width: 0%;
            height: 100%;
            background: linear-gradient(90deg, #ef4444, #dc2626);
            border-radius: 4px;
            animation: loading 30s linear forwards;
        }
        @keyframes loading {
            to { width: 100%; }
        }
    </style>
</head>
<body>
    <div class="card">
        <div class="restart-icon">⚡</div>
        <h1>Reiniciando Sistema...</h1>
        <p>O ESP32 está sendo reiniciado. Aguarde aproximadamente 30 segundos.</p>
        <p>Em seguida, recarregue a página ou acesse novamente o IP do dispositivo.</p>
        
        <div class="loading-bar">
            <div class="loading-progress"></div>
        </div>
        
        <p><strong>Não desconecte a energia durante este processo!</strong></p>
    </div>
    
    <script>
        // Tentar recarregar automaticamente após 35 segundos
        setTimeout(() => {
            window.location.href = '/';
        }, 35000);
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
    
    // Agendar o reinício
    Serial.println("✅ Reinício solicitado via web - reiniciando em 3 segundos...");
    delay(3000);
    ESP.restart();
  });

    // Página de reset de fábrica
  server.on("/factoryreset", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Reset de Fábrica - ESP32 Temperature</title>
    <style>
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        :root {
            --primary: #6366f1;
            --primary-dark: #4f46e5;
            --secondary: #10b981;
            --danger: #ef4444;
            --warning: #f59e0b;
            --dark: #1f2937;
            --light: #f8fafc;
            --gray: #6b7280;
            --gray-light: #e5e7eb;
        }

        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: var(--dark);
            display: flex;
            align-items: center;
            justify-content: center;
        }

        .container {
            max-width: 600px;
            width: 100%;
        }

        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 40px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
        }

        .header {
            text-align: center;
            margin-bottom: 30px;
        }

        .header h1 {
            font-size: 2em;
            margin-bottom: 10px;
            background: linear-gradient(135deg, #dc2626, #b91c1c);
            -webkit-background-clip: text;
            -webkit-text-fill-color: transparent;
        }

        .alert {
            padding: 20px;
            border-radius: 15px;
            margin-bottom: 25px;
            font-weight: 600;
        }

        .alert-danger {
            background: #fef2f2;
            color: #dc2626;
            border: 2px solid #ef4444;
        }

        .btn-group {
            display: flex;
            gap: 15px;
            margin-top: 30px;
        }

        .btn {
            flex: 1;
            padding: 15px 30px;
            border: none;
            border-radius: 50px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s ease;
            font-size: 16px;
            text-decoration: none;
            text-align: center;
        }

        .btn-danger {
            background: #dc2626;
            color: white;
        }

        .btn-danger:hover {
            background: #b91c1c;
            transform: translateY(-2px);
            box-shadow: 0 10px 20px rgba(220, 38, 38, 0.3);
        }

        .btn-secondary {
            background: var(--gray);
            color: white;
        }

        .btn-secondary:hover {
            background: #4b5563;
            transform: translateY(-2px);
        }

        .warning-icon {
            font-size: 4em;
            margin-bottom: 20px;
            text-align: center;
        }

        .config-list {
            background: #f8fafc;
            padding: 20px;
            border-radius: 10px;
            margin: 20px 0;
        }

        .config-item {
            margin: 10px 0;
            padding: 8px;
            border-left: 3px solid #ef4444;
            background: white;
        }

        .countdown {
            font-size: 1.2em;
            font-weight: 600;
            color: #dc2626;
            margin: 20px 0;
            text-align: center;
        }

        .input-confirm {
            width: 100%;
            padding: 15px;
            border: 2px solid #ef4444;
            border-radius: 10px;
            font-size: 16px;
            text-align: center;
            margin: 20px 0;
            background: #fef2f2;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="card">
            <div class="warning-icon">⚠️</div>
            
            <div class="header">
                <h1>Reset de Fábrica</h1>
                <p>Restaurar todas as configurações padrão</p>
            </div>

            <div class="alert alert-danger">
                🚨 <strong>ATENÇÃO: Esta operação é IRREVERSÍVEL!</strong>
            </div>

            <p>Todas as configurações serão perdidas e resetadas para os valores de fábrica:</p>

            <div class="config-list">
                <div class="config-item">
                    <strong>🔸 Credenciais Web:</strong> admin / admin123
                </div>
                <div class="config-item">
                    <strong>🔸 Configurações WiFi:</strong> Serão removidas
                </div>
                <div class="config-item">
                    <strong>🔸 Configurações MQTT:</strong> Valores padrão
                </div>
                <div class="config-item">
                    <strong>🔸 Nome do Sensor:</strong> DS18B20
                </div>
                <div class="config-item">
                    <strong>🔸 Localização:</strong> Sala
                </div>
            </div>

            <p>Após o reset, o sistema entrará em modo AP e você precisará reconfigurar o WiFi.</p>

            <div style="text-align: center; margin: 20px 0;">
                <label for="confirmText">Digite <strong>RESETAR</strong> para confirmar:</label>
                <input type="text" id="confirmText" class="input-confirm" placeholder="Digite RESETAR aqui" oninput="checkConfirmation()">
            </div>

            <div class="btn-group">
                <button id="resetBtn" disabled class="btn btn-danger">🗑️ Reset de Fábrica</button>
                <a href="/admin" class="btn btn-secondary">↩️ Cancelar</a>
            </div>

            <div id="countdown" class="countdown" style="display: none;"></div>
        </div>
    </div>

    <script>
        function checkConfirmation() {
            const input = document.getElementById('confirmText');
            const button = document.getElementById('resetBtn');
            
            if (input.value.toUpperCase() === 'RESETAR') {
                button.disabled = false;
                button.innerHTML = '🗑️ Confirmar Reset de Fábrica';
            } else {
                button.disabled = true;
                button.innerHTML = '🗑️ Reset de Fábrica';
            }
        }

        function confirmFactoryReset() {
            if (confirm('CONFIRMAÇÃO FINAL: Tem certeza absoluta que deseja resetar para os padrões de fábrica? TODAS as configurações serão perdidas!')) {
                // Mostrar contagem regressiva
                const countdownEl = document.getElementById('countdown');
                const btnGroup = document.querySelector('.btn-group');
                const confirmInput = document.getElementById('confirmText');
                
                btnGroup.style.display = 'none';
                confirmInput.style.display = 'none';
                countdownEl.style.display = 'block';
                
                let seconds = 5;
                const countdownInterval = setInterval(() => {
                    countdownEl.innerHTML = `Resetando em ${seconds} segundos...`;
                    seconds--;
                    
                    if (seconds < 0) {
                        clearInterval(countdownInterval);
                        // Fazer a requisição para resetar
                        window.location.href = '/dofactoryreset';
                    }
                }, 1000);
            }
        }

        // Ativar o botão quando a confirmação estiver correta
        document.getElementById('resetBtn').addEventListener('click', confirmFactoryReset);
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
  });

  // Handler para executar o reset de fábrica
  server.on("/dofactoryreset", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Resetando - ESP32 Temperature</title>
    <style>
        body {
            font-family: 'Segoe UI', system-ui, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
            color: #1f2937;
            display: flex;
            align-items: center;
            justify-content: center;
            text-align: center;
        }
        .card {
            background: rgba(255, 255, 255, 0.95);
            backdrop-filter: blur(10px);
            border-radius: 20px;
            padding: 50px;
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            border: 1px solid rgba(255, 255, 255, 0.2);
            max-width: 500px;
        }
        .reset-icon {
            font-size: 4em;
            margin-bottom: 20px;
            animation: pulse 1s infinite;
        }
        @keyframes pulse {
            0% { transform: scale(1); }
            50% { transform: scale(1.1); }
            100% { transform: scale(1); }
        }
        h1 {
            color: #dc2626;
            margin-bottom: 20px;
        }
        .progress-container {
            width: 100%;
            background: #e5e7eb;
            border-radius: 10px;
            margin: 30px 0;
            overflow: hidden;
        }
        .progress-bar {
            width: 0%;
            height: 20px;
            background: linear-gradient(90deg, #dc2626, #b91c1c);
            border-radius: 10px;
            animation: progress 5s linear forwards;
        }
        @keyframes progress {
            to { width: 100%; }
        }
        .status-list {
            text-align: left;
            margin: 20px 0;
        }
        .status-item {
            margin: 10px 0;
            padding: 8px;
            border-left: 3px solid #10b981;
            background: #f0f9ff;
        }
    </style>
</head>
<body>
    <div class="card">
        <div class="reset-icon">🔄</div>
        <h1>Reset de Fábrica em Andamento</h1>
        <p>Restaurando todas as configurações para os padrões de fábrica...</p>
        
        <div class="progress-container">
            <div class="progress-bar"></div>
        </div>

        <div class="status-list">
            <div class="status-item">✅ Limpando configurações da EEPROM</div>
            <div class="status-item">✅ Restaurando credenciais web padrão</div>
            <div class="status-item">✅ Resetando configurações WiFi</div>
            <div class="status-item">✅ Configurações MQTT padrão</div>
            <div class="status-item">✅ Nome do sensor padrão</div>
        </div>
        
        <p><strong>O sistema reiniciará automaticamente em modo AP.</strong></p>
        <p>Conecte-se ao WiFi <strong>ESP32-Temperature</strong> para reconfigurar.</p>
    </div>
    
    <script>
        // Tentar recarregar automaticamente após 10 segundos
        setTimeout(() => {
            window.location.href = '/';
        }, 10000);
    </script>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
    
    // Executar o reset de fábrica
    performFactoryReset();
  });

// VERSÃO ULTRA-SEGURA - Sem gradientes complexos
server.on("/admin", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String html = R"=====(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Admin - ESP32 Temperature</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background: #f8f9fa;
            margin: 0;
            padding: 20px;
            color: #333;
        }
        .container {
            max-width: 1000px;
            margin: 0 auto;
        }
        .header {
            background: white;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 30px;
            text-align: center;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-left: 5px solid #6366f1;
        }
        .header h1 {
            color: #6366f1;
            margin-bottom: 10px;
            font-size: 2.5em;
        }
        .grid {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 25px;
            margin-bottom: 30px;
        }
        .card {
            background: white;
            padding: 30px;
            border-radius: 10px;
            text-align: center;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            border-top: 4px solid;
        }
        .card:nth-child(1) { border-color: #ec4899; }
        .card:nth-child(2) { border-color: #f59e0b; }
        .card:nth-child(3) { border-color: #dc2626; }
        .card h2 {
            margin-bottom: 15px;
            color: #333;
        }
        .btn {
            display: block;
            padding: 15px;
            margin: 10px 0;
            color: white;
            text-decoration: none;
            border-radius: 8px;
            font-weight: bold;
            border: none;
            width: 100%;
            font-size: 16px;
        }
        .btn-password { background: #ec4899; }
        .btn-password:hover { background: #db2777; }
        .btn-restart { background: #f59e0b; color: #000; }
        .btn-restart:hover { background: #d97706; }
        .btn-factory { background: #dc2626; }
        .btn-factory:hover { background: #b91c1c; }
        .btn-sensor { background: #06b6d4; }
        .btn-sensor:hover { background: #0891b2; }
        .btn-back {
            background: #6b7280;
            width: auto;
            padding: 12px 25px;
            display: inline-block;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>⚙️ Painel Administrativo</h1>
            <p>Gerenciamento completo do sistema</p>
        </div>
        
        <div class="grid">
            <div class="card">
                <h2>🔐 Segurança</h2>
                <p>Alterar credenciais de acesso</p>
                <a href="/password" class="btn btn-password">Alterar Senha</a>
            </div>
            
            <div class="card">
                <h2>🔁 Reinicialização</h2>
                <p>Reiniciar o sistema</p>
                <a href="/restart" class="btn btn-restart">Reiniciar Sistema</a>
            </div>
            
            <div class="card">
                <h2>🏭 Reset Fábrica</h2>
                <p>Restaurar configurações padrão</p>
                <a href="/factoryreset" class="btn btn-factory">Reset de Fábrica</a>
            </div>
        </div>
        <div class="grid">
            <div class="card">
                <h2>🔧 Sensor</h2>
                <p>Alterar dados do sensor</p>
                <a href="/sensor" class="btn btn-sensor">Alterar dados do Sensor</a>
            </div>
        </div>
        
        <div style="text-align:center;">
            <a href="/" class="btn btn-back">↩️ Voltar</a>
        </div>
    </div>
</body>
</html>
    )=====";
    
    server.send(200, "text/html", html);
});

  // Rota para reenviar anúncio ao servidor (readoção)
  server.on("/announce", HTTP_GET, []() {
    if (!isAuthenticated()) return;
    
    String json = "{";
    
    if (!mqttClient.connected()) {
      connectMQTT();
      delay(500);
    }
    
    if (mqttClient.connected()) {
      String announceTopic = "devices/announce";
      String announcePayload = "{";
    announcePayload += "\"mac\":\"" + deviceMac + "\",";
      announcePayload += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
      announcePayload += "\"version\":\"" + String(FIRMWARE_VERSION) + "\",";
      announcePayload += "\"sensor\":\"" + String(sensorConfig.sensorName) + "\",";
      announcePayload += "\"location\":\"" + String(sensorConfig.location) + "\",";
      announcePayload += "\"rssi\":" + String(WiFi.RSSI()) + ",";
      announcePayload += "\"heap\":" + String(esp_get_free_heap_size()) + ",";
      announcePayload += "\"uptime\":" + String(millis() / 1000);
      announcePayload += "}";
      
      Serial.println("📤 Anúncio manual - Tópico: " + announceTopic);
      Serial.println("   Tamanho: " + String(announcePayload.length()) + " bytes");
      Serial.println("   Payload: " + announcePayload);
      
      if (mqttClient.publish(announceTopic.c_str(), announcePayload.c_str(), true)) {
        json += "\"status\":\"success\",";
        json += "\"message\":\"Dispositivo anunciado com sucesso!\",";
        json += "\"payload\":" + announcePayload;
        json += "}";
        Serial.println("✅ Dispositivo reanunciado manualmente ao servidor");
      } else {
        json += "\"status\":\"error\",";
        json += "\"message\":\"Falha ao publicar no MQTT\"";
        json += "}";
        Serial.println("⚠️ Falha no anúncio manual - Estado: " + String(mqttClient.state()));
      }
    } else {
      json += "\"status\":\"error\",";
      json += "\"message\":\"MQTT não conectado\"";
      json += "}";
    }
    
    server.send(200, "application/json", json);
  });

  server.begin();
  Serial.println("✅ Servidor web iniciado na porta 80");
}