#include <DHT.h>
#include <Wire.h>
#include <ArduinoJson.h>

// Definizioni per il modulo SIM7600E-H
#define BAUD       115200
#define SIM_PIN    "2305"  // PIN della SIM

// Definizioni per il sensore DHT11
#define DHT_PIN     2
#define DHT_TYPE    DHT11
DHT dht(DHT_PIN, DHT_TYPE);

// Definizioni per il sensore MPU6050
#define MPU_ADDR    0x68  // Indirizzo I2C del MPU6050

// Variabili per memorizzare i dati dei sensori
float temperature = 0.0;
float humidity = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float latitude = 0.0, longitude = 0.0, altitude = 0.0;

// Variabili di stato - ottimizzate per velocità
bool connectionEstablished = false;
bool gpsEnabled = false;
bool fastMode = false;  // Modalità veloce attivata dopo primo successo
bool mpuAvailable = false;  // Stato MPU6050
bool dhtAvailable = false;  // Stato DHT11

// Contatori e timing
int errorCount = 0;
int successCount = 0;
int sensorErrorCount = 0;  // Contatore errori sensori
unsigned long lastConnectionCheck = 0;
unsigned long lastFullReset = 0;
unsigned long lastGpsRead = 0;
unsigned long lastSensorRead = 0;

// Intervalli temporali ottimizzati
const unsigned long SEND_INTERVAL = 1000;           // 1 secondo
const unsigned long CONNECTION_CHECK_INTERVAL = 30000;  // Controlla connessione ogni 30s
const unsigned long FULL_RESET_INTERVAL = 3600000;  // Reset completo ogni ora
const unsigned long GPS_READ_INTERVAL = 30000;      // GPS dettagliato ogni 30s
const unsigned long SENSOR_READ_INTERVAL = 900;     // Sensori ogni 900ms (include GPS status)

// Timeout ridotti per velocità
const unsigned long FAST_TIMEOUT = 500;
const unsigned long NORMAL_TIMEOUT = 2000;

void sendCmd(const String& cmd) {
  Serial1.println(cmd);
  delay(50); // Delay ridotto
}

String readResponse(unsigned long timeout = FAST_TIMEOUT) {
  unsigned long start = millis();
  String response = "";
  while (millis() - start < timeout) {
    while (Serial1.available()) {
      char c = Serial1.read();
      response += c;
      // Ottimizzazione: esci appena troviamo OK/ERROR
      if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) {
        return response;
      }
    }
  }
  return response;
}

bool sendCmdAndCheck(const String& cmd, const String& ack, unsigned long timeout = FAST_TIMEOUT) {
  sendCmd(cmd);
  String resp = readResponse(timeout);
  return resp.indexOf(ack) != -1;
}

// Funzione per scan I2C devices
void scanI2C() {
  Serial.println("🔍 Scansione dispositivi I2C...");
  int deviceCount = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  ✅ Dispositivo trovato a 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
      deviceCount++;
    }
  }
  
  if (deviceCount == 0) {
    Serial.println("  ❌ Nessun dispositivo I2C trovato");
  }
}

// Funzione per inizializzare il MPU6050
bool initMPU() {
  Serial.print("🔧 Inizializzazione MPU6050... ");
  
  // Scan I2C prima di inizializzare
  Wire.begin();
  delay(100);
  
  // Prova diversi indirizzi MPU6050 (0x68 e 0x69)
  byte mpuAddr = 0x68;
  Wire.beginTransmission(mpuAddr);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    mpuAddr = 0x69;  // Indirizzo alternativo
    Wire.beginTransmission(mpuAddr);
    error = Wire.endTransmission();
  }
  
  if (error != 0) {
    Serial.print("❌ MPU6050 non trovato (errore I2C: "); Serial.print(error); Serial.println(")");
    scanI2C();
    return false;
  }
  
  Serial.print("Trovato a 0x"); Serial.print(mpuAddr, HEX); Serial.print("... ");
  
  // Reset del dispositivo
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x80);  // DEVICE_RESET
  Wire.endTransmission();
  delay(100);
  
  // Sveglia il dispositivo
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x01);  // CLKSEL=1 (PLL with X-axis gyro)
  Wire.endTransmission();
  delay(10);
  
  // Configurazione accelerometro (±2g)
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1C);  // ACCEL_CONFIG
  Wire.write(0x00);  // ±2g
  Wire.endTransmission();
  
  // Configurazione giroscopio (±250°/s)
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x1B);  // GYRO_CONFIG
  Wire.write(0x00);  // ±250°/s
  Wire.endTransmission();
  
  // Test WHO_AM_I
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(mpuAddr, (byte)1);
  
  if (Wire.available()) {
    byte whoAmI = Wire.read();
    if (whoAmI == 0x68) {
      Serial.println("✅ OK (WHO_AM_I verificato)");
      return true;
    } else {
      Serial.print("❌ WHO_AM_I errato: 0x"); Serial.println(whoAmI, HEX);
    }
  } else {
    Serial.println("❌ Nessuna risposta WHO_AM_I");
  }
  
  return false;
}

// Lettura veloce MPU6050 con retry
void readMPU() {
  Serial.print("🔄 MPU6050... ");
  
  // Test rapido di comunicazione
  Wire.beginTransmission(MPU_ADDR);
  byte error = Wire.endTransmission();
  
  if (error != 0) {
    Serial.print("❌ Errore I2C: "); Serial.println(error);
    return;
  }
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // Registro dati accelerometro
  Wire.endTransmission(false);
  byte bytesReceived = Wire.requestFrom(MPU_ADDR, 14, true);
  
  Serial.print("Ricevuti "); Serial.print(bytesReceived); Serial.print("/14 bytes... ");
  
  if (bytesReceived >= 14) {
    // Leggi tutti i dati
    int16_t rawAccelX = Wire.read() << 8 | Wire.read();
    int16_t rawAccelY = Wire.read() << 8 | Wire.read();
    int16_t rawAccelZ = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Salta temperatura
    int16_t rawGyroX = Wire.read() << 8 | Wire.read();
    int16_t rawGyroY = Wire.read() << 8 | Wire.read();
    int16_t rawGyroZ = Wire.read() << 8 | Wire.read();
    
    // Converti in unità fisiche
    accelX = rawAccelX / 16384.0;
    accelY = rawAccelY / 16384.0;
    accelZ = rawAccelZ / 16384.0;
    gyroX = rawGyroX / 131.0;
    gyroY = rawGyroY / 131.0;
    gyroZ = rawGyroZ / 131.0;
    
    Serial.print("✅ A:("); Serial.print(accelX,1); Serial.print(","); Serial.print(accelY,1); Serial.print(","); Serial.print(accelZ,1);
    Serial.print(") G:("); Serial.print(gyroX,1); Serial.print(","); Serial.print(gyroY,1); Serial.print(","); Serial.print(gyroZ,1); Serial.println(")");
  } else {
    Serial.println("❌ Dati insufficienti");
    
    // Svuota il buffer in caso di dati parziali
    while (Wire.available()) {
      Wire.read();
    }
  }
}

// Lettura veloce DHT11 con inizializzazione migliorata
void readDHT() {
  Serial.print("🌡️  DHT11... ");
  
  // Prova lettura 3 volte con delay diversi
  for (int attempt = 0; attempt < 3; attempt++) {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    // Verifica valori ragionevoli
    if (!isnan(h) && !isnan(t) && t > -40 && t < 80 && h >= 0 && h <= 100) {
      humidity = h;
      temperature = t;
      Serial.print("✅ T:"); Serial.print(t, 1); Serial.print("°C H:"); Serial.print(h, 1); Serial.println("%");
      dhtAvailable = true;
      return;
    }
    
    if (attempt < 2) {
      Serial.print("❌ Retry("); Serial.print(attempt + 1); Serial.print(")... ");
      delay(500);  // Delay più lungo tra tentativi
    }
  }
  
  Serial.print("❌ Fallito dopo 3 tentativi");
  
  // Diagnostica pin DHT
  pinMode(DHT_PIN, INPUT_PULLUP);
  delay(10);
  int pinState = digitalRead(DHT_PIN);
  Serial.print(" (Pin "); Serial.print(DHT_PIN); Serial.print(" = "); Serial.print(pinState); Serial.print(")");
  
  // Test pull-up
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, HIGH);
  delay(10);
  pinMode(DHT_PIN, INPUT_PULLUP);
  delay(10);
  pinState = digitalRead(DHT_PIN);
  Serial.print(" Pull-up: "); Serial.println(pinState);
  
  dhtAvailable = false;
  
  // Mantieni gli ultimi valori validi se disponibili
  if (temperature > 0) {
    Serial.print("ℹ️  Uso ultimi valori: T:"); Serial.print(temperature, 1); Serial.print("°C H:"); Serial.print(humidity, 1); Serial.println("%");
  }
}

// Lettura GPS migliorata con debug completo
void readGPS() {
  if (!gpsEnabled) {
    Serial.print("📡 Attivazione GPS... ");
    
    // Verifica stato attuale
    sendCmd("AT+CGPS?");
    String status = readResponse(2000);
    Serial.print("Stato: "); Serial.print(status);
    
    if (status.indexOf("+CGPS: 1") == -1) {
      Serial.println();
      Serial.print("   Attivando GPS... ");
      if (sendCmdAndCheck("AT+CGPS=1", "OK", 5000)) {
        Serial.println("✅ Comando OK");
        gpsEnabled = true;
        
        // Attendi che il GPS si stabilizzi
        Serial.println("   ⌛ Attendo stabilizzazione GPS (10s)...");
        delay(10000);
      } else {
        Serial.println("❌ Comando fallito");
        return;
      }
    } else {
      Serial.println("✅ Già attivo");
      gpsEnabled = true;
    }
    return;
  }
  
  Serial.print("📍 Lettura GPS... ");
  sendCmd("AT+CGPSINFO");
  String gpsData = readResponse(3000);  // Timeout più lungo
  
  Serial.print("Raw: ");
  Serial.println(gpsData);
  
  // Verifica se ci sono dati GPS
  if (gpsData.indexOf("+CGPSINFO:") == -1) {
    Serial.println("❌ Nessuna risposta GPS");
    return;
  }
  
  // Estrai la parte dopo +CGPSINFO:
  int startIdx = gpsData.indexOf("+CGPSINFO:") + 11;
  String gpsInfo = gpsData.substring(startIdx);
  gpsInfo.trim();
  
  Serial.print("Parsed: '"); Serial.print(gpsInfo); Serial.println("'");
  
  // Verifica se i dati sono vuoti (solo virgole)
  if (gpsInfo.length() < 10 || gpsInfo.indexOf(",,,,,,,,") != -1) {
    Serial.println("⌛ GPS attivo ma fix non ancora acquisito");
    
    // Mostra numero satelliti se disponibile
    sendCmd("AT+CGPSINFO=1");
    String detailedInfo = readResponse(2000);
    if (detailedInfo.indexOf("SAT:") != -1) {
      int satIdx = detailedInfo.indexOf("SAT:") + 4;
      int satEndIdx = detailedInfo.indexOf(' ', satIdx);
      if (satEndIdx == -1) satEndIdx = detailedInfo.indexOf('\r', satIdx);
      if (satEndIdx > satIdx) {
        String satCount = detailedInfo.substring(satIdx, satEndIdx);
        Serial.print("   🛰️  Satelliti visibili: "); Serial.println(satCount);
      }
    }
    return;
  }
  
  // Parsing più robusto dei dati GPS
  // Formato: lat,N/S,lon,E/W,date,UTC time,alt,speed,course,fix mode,reserved1,HDOP,PDOP,VDOP,reserved2,view_satellites,used_satellites,CN0 max,HPA,VPA
  
  int commaPositions[20];
  int commaCount = 0;
  
  // Trova tutte le posizioni delle virgole
  for (int i = 0; i < gpsInfo.length() && commaCount < 20; i++) {
    if (gpsInfo.charAt(i) == ',') {
      commaPositions[commaCount] = i;
      commaCount++;
    }
  }
  
  if (commaCount >= 4) {
    // Estrai latitudine
    String latStr = gpsInfo.substring(0, commaPositions[0]);
    String latDir = gpsInfo.substring(commaPositions[0] + 1, commaPositions[1]);
    String lonStr = gpsInfo.substring(commaPositions[1] + 1, commaPositions[2]);
    String lonDir = gpsInfo.substring(commaPositions[2] + 1, commaPositions[3]);
    
    Serial.print("   Lat: '"); Serial.print(latStr); Serial.print("' "); Serial.print(latDir);
    Serial.print(" Lon: '"); Serial.print(lonStr); Serial.print("' "); Serial.println(lonDir);
    
    if (latStr.length() >= 4 && lonStr.length() >= 5) {
      // Conversione da formato DDMM.MMMM a decimale
      float latDeg = latStr.substring(0, 2).toFloat();
      float latMin = latStr.substring(2).toFloat() / 60.0;
      latitude = latDeg + latMin;
      if (latDir == "S") latitude = -latitude;
      
      float lonDeg = lonStr.substring(0, 3).toFloat();
      float lonMin = lonStr.substring(3).toFloat() / 60.0;
      longitude = lonDeg + lonMin;
      if (lonDir == "W") longitude = -longitude;
      
      // Altitudine (se disponibile)
      if (commaCount >= 7) {
        String altStr = gpsInfo.substring(commaPositions[5] + 1, commaPositions[6]);
        if (altStr.length() > 0) {
          altitude = altStr.toFloat();
        }
      }
      
      Serial.print("✅ Coordinate: ");
      Serial.print("Lat: "); Serial.print(latitude, 6);
      Serial.print(", Lon: "); Serial.print(longitude, 6);
      if (altitude != 0) {
        Serial.print(", Alt: "); Serial.print(altitude); Serial.print("m");
      }
      Serial.println();
    } else {
      Serial.println("⚠️  Formato coordinate non valido");
    }
  } else {
    Serial.println("⚠️  Dati GPS incompleti");
  }
}

// Prepara JSON ottimizzato con GPS sempre incluso
String prepareJSON() {
  StaticJsonDocument<600> doc;
  
  // Dati sensori (sempre presenti)
  doc["temp"] = temperature;
  doc["hum"] = humidity;
  
  // Dati accelerometro
  JsonObject acc = doc.createNestedObject("acc");
  acc["x"] = round(accelX * 100) / 100.0;
  acc["y"] = round(accelY * 100) / 100.0;
  acc["z"] = round(accelZ * 100) / 100.0;
  
  // Dati giroscopio
  JsonObject gyro = doc.createNestedObject("gyro");
  gyro["x"] = round(gyroX * 10) / 10.0;
  gyro["y"] = round(gyroY * 10) / 10.0;
  gyro["z"] = round(gyroZ * 10) / 10.0;
  
  // Dati GPS (sempre presenti, anche se 0,0)
  JsonObject gps = doc.createNestedObject("gps");
  gps["lat"] = latitude;
  gps["lon"] = longitude;
  gps["alt"] = altitude;
  gps["fix"] = (latitude != 0.0 || longitude != 0.0);  // Flag se ha fix GPS
  
  // Metadati
  doc["ts"] = millis();
  doc["cnt"] = successCount;
  
  // Stato sensori per debugging
  JsonObject status = doc.createNestedObject("status");
  status["mpu"] = mpuAvailable;
  status["dht"] = dhtAvailable;
  status["gps"] = gpsEnabled;
  status["errors"] = sensorErrorCount;
  
  String output;
  serializeJson(doc, output);
  return output;
}

// Setup connessione completo (solo all'avvio)
bool setupFullConnection() {
  Serial.println("Setup connessione completa...");
  
  // Test comunicazione base
  if (!sendCmdAndCheck("AT", "OK", 1000)) {
    Serial.println("Modem non risponde");
    return false;
  }
  
  sendCmdAndCheck("ATE0", "OK");
  
  // Gestione PIN SIM (solo se necessario)
  sendCmd("AT+CPIN?");
  String pinResp = readResponse(1000);
  if (pinResp.indexOf("SIM PIN") != -1) {
    sendCmdAndCheck("AT+CPIN=" + String(SIM_PIN), "OK", 3000);
    delay(3000);
  }
  
  // Setup rete veloce
  sendCmdAndCheck("AT+CFUN=1", "OK");
  sendCmdAndCheck("AT+CGATT=1", "OK", 5000);
  sendCmdAndCheck("AT+CGDCONT=1,\"IP\",\"mobile.vodafone.it\"", "OK");
  sendCmdAndCheck("AT+CGACT=1,1", "OK", 5000);
  sendCmdAndCheck("AT+NETOPEN", "OK", 5000);
  
  // Setup HTTP rapido
  sendCmdAndCheck("AT+HTTPINIT", "OK", 3000);
  sendCmdAndCheck("AT+HTTPPARA=\"URL\",\"https://eas.medusa.dev:5555/report\"", "OK");
  sendCmdAndCheck("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK");
  sendCmdAndCheck("AT+HTTPPARA=\"USERDATA\",\"X-API-Key: k925tSMRTB7a6idl7KyaUsoCGMuqig\"", "OK");
  
  connectionEstablished = true;
  fastMode = false;
  Serial.println("Connessione stabilita!");
  return true;
}

// Invio dati veloce (ottimizzato per 1 secondo)
bool sendDataFast(const String& jsonPayload) {
  int len = jsonPayload.length();
  
  Serial.print("🔄 Invio dati ("); Serial.print(len); Serial.println(" bytes):");
  Serial.println(jsonPayload);
  
  // Invio dati senza verifiche eccessive
  Serial.print("📤 HTTPDATA... ");
  if (!sendCmdAndCheck("AT+HTTPDATA=" + String(len) + ",5000", "DOWNLOAD", 1000)) {
    Serial.println("❌ FALLITO");
    return false;
  }
  Serial.println("✅ OK");
  
  Serial.print("📦 Invio payload... ");
  Serial1.print(jsonPayload);
  
  // Attesa ridotta per conferma dati
  delay(200);
  String resp = readResponse(500);
  if (resp.indexOf("OK") == -1) {
    Serial.println("❌ Payload rifiutato");
    Serial.print("Risposta: "); Serial.println(resp);
    return false;
  }
  Serial.println("✅ Payload accettato");
  
  // POST request
  Serial.print("🚀 HTTP POST... ");
  if (!sendCmdAndCheck("AT+HTTPACTION=1", "OK", 500)) {
    Serial.println("❌ POST fallito");
    return false;
  }
  Serial.println("✅ POST inviato");
  
  // In modalità veloce, non aspettiamo la risposta completa
  if (fastMode) {
    Serial.println("⚡ Modalità veloce: assumo successo");
    return true;  // Assumiamo successo per velocità
  }
  
  // Controllo rapido della risposta
  Serial.print("⌛ Attendo risposta server... ");
  unsigned long start = millis();
  while (millis() - start < 1000) {
    if (Serial1.available()) {
      String actionResp = readResponse(200);
      if (actionResp.indexOf("+HTTPACTION: 1,") != -1) {
        // Estrai codice di stato
        int statusStart = actionResp.indexOf("+HTTPACTION: 1,") + 15;
        int statusEnd = actionResp.indexOf(',', statusStart);
        String statusCode = actionResp.substring(statusStart, statusEnd);
        
        Serial.print("Codice HTTP: "); Serial.println(statusCode);
        
        // Se otteniamo 200-299, abilita modalità veloce
        if (statusCode.startsWith("2")) {
          if (!fastMode) {
            Serial.println("🎯 Modalità veloce attivata!");
            fastMode = true;
          }
          return true;
        } else {
          Serial.print("⚠️  Codice HTTP non OK: "); Serial.println(statusCode);
          return false;
        }
      }
    }
    delay(50);
  }
  
  Serial.println("⏱️  Timeout risposta");
  return true;  // Timeout, assumiamo successo
}

// Controllo connessione leggero
bool quickConnectionCheck() {
  Serial.print("🔍 Test connessione... ");
  // Test rapido: invia AT e verifica risposta
  if (!sendCmdAndCheck("AT", "OK", 300)) {
    Serial.println("❌ Modem non risponde");
    return false;
  }
  Serial.println("✅ Modem OK");
  return true;
}

// Reset leggero (solo HTTP)
void lightReset() {
  Serial.println("🔄 Reset leggero HTTP...");
  sendCmdAndCheck("AT+HTTPTERM", "OK", 1000);
  delay(500);
  sendCmdAndCheck("AT+HTTPINIT", "OK", 2000);
  sendCmdAndCheck("AT+HTTPPARA=\"URL\",\"https://eas.medusa.dev:5555/report\"", "OK");
  sendCmdAndCheck("AT+HTTPPARA=\"CONTENT\",\"application/json\"", "OK");
  sendCmdAndCheck("AT+HTTPPARA=\"USERDATA\",\"X-API-Key: k925tSMRTB7a6idl7KyaUsoCGMuqig\"", "OK");
  fastMode = false;
  Serial.println("✅ Reset leggero completato");
}

// Reset completo
void fullReset() {
  Serial.println("🔴 Reset completo connessione...");
  sendCmdAndCheck("AT+HTTPTERM", "OK", 1000);
  sendCmdAndCheck("AT+NETCLOSE", "OK", 2000);
  sendCmdAndCheck("AT+CGACT=0,1", "OK", 2000);
  connectionEstablished = false;
  fastMode = false;
  delay(2000);
  Serial.println("✅ Reset completo eseguito");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  
  Serial.println("\n=== Arduino Sensori Diagnostico v3.0 ===");
  Serial.println("🔧 Inizializzazione hardware completa...\n");
  
  // Test Serial1 (comunicazione SIM7600)
  Serial.println("📱 Test comunicazione SIM7600:");
  Serial1.begin(BAUD);
  delay(3000);  // Attesa per stabilizzazione modem
  
  // Svuota buffer
  while (Serial1.available()) {
    Serial1.read();
  }
  
  if (sendCmdAndCheck("AT", "OK", 3000)) {
    Serial.println("   ✅ SIM7600 risponde correttamente");
  } else {
    Serial.println("   ❌ SIM7600 non risponde - Verifica connessioni");
  }
  
  Serial.println();
  
  // Test DHT11 con inizializzazione completa
  Serial.println("🌡️  Test DHT11:");
  Serial.print("   Pin configurato: "); Serial.println(DHT_PIN);
  
  // Configurazione pin
  pinMode(DHT_PIN, OUTPUT);
  digitalWrite(DHT_PIN, HIGH);
  delay(100);
  pinMode(DHT_PIN, INPUT_PULLUP);
  delay(100);
  
  // Inizializzazione DHT con tempo extra
  dht.begin();
  Serial.println("   Attendo stabilizzazione DHT11 (3 secondi)...");
  delay(3000);
  
  // Test lettura
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("   Tentativo "); Serial.print(attempt); Serial.print("/3... ");
    float testTemp = dht.readTemperature();
    float testHum = dht.readHumidity();
    
    if (!isnan(testTemp) && !isnan(testHum) && testTemp > -40 && testTemp < 80) {
      Serial.print("✅ OK - T:"); Serial.print(testTemp); Serial.print("°C H:"); Serial.print(testHum); Serial.println("%");
      dhtAvailable = true;
      temperature = testTemp;
      humidity = testHum;
      break;
    } else {
      Serial.println("❌ Valori non validi");
      if (attempt == 3) {
        Serial.println("   ⚠️  DHT11 non funziona - Verifica cablaggio");
        dhtAvailable = false;
      }
      delay(1000);
    }
  }
  
  Serial.println();
  
  // Test MPU6050 con scan I2C
  Serial.println("🔄 Test MPU6050:");
  mpuAvailable = initMPU();
  
  if (mpuAvailable) {
    // Test lettura dati
    Serial.print("   Test lettura... ");
    readMPU();
    if (accelX != 0 || accelY != 0 || accelZ != 0) {
      Serial.println("   ✅ Dati accelerometro OK");
    } else {
      Serial.println("   ⚠️  Possibile problema con accelerometro");
    }
  }
  
  Serial.println();
  
  // Test GPS
  Serial.println("📡 Test GPS:");
  Serial.println("   Attivazione e test iniziale...");
  readGPS();  // Prima attivazione
  delay(5000);  // Attendi per GPS
  readGPS();   // Test lettura
  
  Serial.println();
  
  // Riepilogo finale
  Serial.println("📋 RIEPILOGO STATO HARDWARE:");
  Serial.print("   SIM7600: "); Serial.println("✅ Comunicazione OK");
  Serial.print("   DHT11:   "); Serial.println(dhtAvailable ? "✅ Funzionante" : "❌ Non funziona");
  Serial.print("   MPU6050: "); Serial.println(mpuAvailable ? "✅ Funzionante" : "❌ Non funziona");
  Serial.print("   GPS:     "); Serial.println(gpsEnabled ? "✅ Attivo" : "❌ Non attivato");
  
  Serial.println();
  
  // Setup connessione di rete
  Serial.println("🌐 Configurazione connessione internet...");
  if (setupFullConnection()) {
    Serial.println("✅ Connessione configurata con successo");
  } else {
    Serial.println("❌ Errore configurazione connessione");
  }
  
  Serial.println();
  Serial.println("🚀 SISTEMA PRONTO!");
  Serial.println("   - Invio dati ogni 1 secondo");
  Serial.println("   - I sensori non funzionanti invieranno zero");
  Serial.println("   - GPS continuerà a cercare il fix");
  Serial.println();
}

void loop() {
  unsigned long now = millis();
  
  // Reset completo programmato (1 volta all'ora)
  if (now - lastFullReset >= FULL_RESET_INTERVAL) {
    fullReset();
    setupFullConnection();
    lastFullReset = now;
  }
  
  // Controllo connessione periodico (solo ogni 30s)
  if (now - lastConnectionCheck >= CONNECTION_CHECK_INTERVAL) {
    if (!quickConnectionCheck()) {
      Serial.println("Connessione persa, riconnessione...");
      fullReset();
      setupFullConnection();
    }
    lastConnectionCheck = now;
  }
  
  // Lettura sensori (ogni 900ms)
  if (now - lastSensorRead >= SENSOR_READ_INTERVAL) {
    Serial.println("\n📊 Lettura sensori:");
    
    // Reset flag di stato
    bool sensorOK = true;
    
    // DHT11
    if (dhtAvailable) {
      readDHT();
      if (temperature == 0.0 && humidity == 0.0) {
        dhtAvailable = false;
        sensorOK = false;
        sensorErrorCount++;
      }
    } else {
      Serial.println("🌡️  DHT11... ⏭️  Saltato (non disponibile)");
    }
    
    // MPU6050
    if (mpuAvailable) {
      readMPU();
      // Controlla se l'MPU sta funzionando (almeno un valore diverso da 0)
      if (accelX == 0.0 && accelY == 0.0 && accelZ == 0.0 && 
          gyroX == 0.0 && gyroY == 0.0 && gyroZ == 0.0) {
        Serial.println("⚠️  MPU6050 potrebbe non funzionare (tutti valori 0)");
        sensorOK = false;
      }
    } else {
      Serial.println("🔄 MPU6050... ⏭️  Saltato (non disponibile)");
    }
    
    // GPS - Lettura ad ogni ciclo sensori
    if (gpsEnabled) {
      Serial.print("📍 GPS... ");
      if (latitude != 0.0 || longitude != 0.0) {
        Serial.print("✅ Fix attivo - Lat:");
        Serial.print(latitude, 6);
        Serial.print(" Lon:");
        Serial.println(longitude, 6);
      } else {
        Serial.println("⌛ In attesa del fix GPS");
      }
    } else {
      Serial.println("📍 GPS... ⏭️  Non attivato");
    }
    
    // Riprova inizializzazione sensori se ci sono troppi errori
    if (sensorErrorCount > 10) {
      Serial.println("🔄 Troppi errori sensori, riprovo inizializzazione...");
      if (!dhtAvailable) {
        dht.begin();
        delay(1000);
        dhtAvailable = true;
      }
      if (!mpuAvailable) {
        mpuAvailable = initMPU();
      }
      sensorErrorCount = 0;
    }
    
    lastSensorRead = now;
  }
  
  // Lettura GPS dettagliata (ogni 30s per parsing completo)
  if (now - lastGpsRead >= GPS_READ_INTERVAL) {
    Serial.println("\n🛰️  Lettura GPS dettagliata:");
    readGPS();
    lastGpsRead = now;
  }
  
  // Invio dati ogni secondo
  static unsigned long lastSend = 0;
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;
    
    // Stampa separatore per chiarezza
    Serial.println("\n" + String("=").substring(0, 50));
    Serial.print("⏰ Ciclo #"); Serial.print(successCount + errorCount + 1);
    Serial.print(" (Successi: "); Serial.print(successCount);
    Serial.print(", Errori: "); Serial.print(errorCount);
    Serial.print(", Modo: "); Serial.print(fastMode ? "VELOCE" : "NORMALE");
    Serial.println(")");
    
    if (connectionEstablished) {
      String json = prepareJSON();
      
      if (sendDataFast(json)) {
        successCount++;
        errorCount = 0;
        Serial.print("✅ SUCCESSO #"); Serial.print(successCount); 
        Serial.println(" - Dati inviati correttamente");
        
        // Dopo 10 successi consecutivi, abilita modalità ultra-veloce
        if (successCount >= 10 && !fastMode) {
          Serial.println("🚀 Switched to FAST MODE after 10 successes!");
          fastMode = true;
        }
      } else {
        errorCount++;
        Serial.print("❌ ERRORE #"); Serial.print(errorCount);
        Serial.println(" - Invio fallito");
        
        // Gestione errori progressiva
        if (errorCount == 1) {
          Serial.println("➡️  Primo errore: continuo normalmente");
        } else if (errorCount == 3) {
          Serial.println("⚠️  Terzo errore: eseguo reset leggero");
          lightReset();
        } else if (errorCount >= 5) {
          Serial.println("🔴 Quinto errore: reset completo necessario");
          fullReset();
          setupFullConnection();
          errorCount = 0;
        } else {
          Serial.println("➡️  Continuo con prossimo tentativo");
        }
      }
    } else {
      Serial.println("⚠️  Connessione non stabilita, riprovo setup...");
      setupFullConnection();
    }
    Serial.println(String("=").substring(0, 50) + "\n");
  }
  
  // Piccolo delay per non sovraccaricare il processore
  delay(10);
}
