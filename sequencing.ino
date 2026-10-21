#include <ModbusMaster.h>

// =================================================================
// 1. PINOUT MODBUS RS485
// =================================================================
#define PIN_DRE_DWIN     2        // Label DRE2 -> Pin 2 Arduino Mega
#define SERIAL_DWIN      Serial3  // TX3 (Pin 14) & RX3 (Pin 15)

#define PIN_DRE_CWT      23       // Label DRE1 -> Pin 23 Arduino Mega
#define SERIAL_CWT       Serial1  // TX1 (Pin 18) & RX1 (Pin 19)

// =================================================================
// 2. PINOUT AKTUATOR 
// =================================================================
const int pinHeater        = A15; // Pemanas + Kipas Heater (Parallel Hardware)
const int pinCompressor    = A14; // Kompresor / Pendingin
const int pinHumidifier    = A13; // Mist Maker
const int pinFanHumidifier = A1;  // SSR Kipas Mist Maker (Sinkron A13)
const int pinLight         = A2;  // Lampu Chamber

// =================================================================
// 3. DAFTAR ALAMAT VP DWIN (MODBUS)
// =================================================================
#define VP_RUN_TRIGGER   0x4500 
#define VP_LIGHT_TRIGGER 0x4600 
#define VP_RUN_ICON      0x2100 
#define VP_SET_TEMP      0x2500
#define VP_SET_HUMI      0x2600
#define VP_SUHU_READ     0x3000
#define VP_HUMI_READ     0x3200
#define VP_ICON_RH       0x1100
#define VP_ICON_COMP     0x1200
#define VP_ICON_HEAT     0x1300

// =================================================================
// 4. PARAMETER KALIBRASI & FILTER (SENSOR CWT)
// =================================================================
const unsigned long COMP_DELAY = 5000; // Jeda proteksi kompresor
bool compReady = true;
unsigned long compTimer = 0;
bool reqComp = false;

// Timer Siklus 1 Menit Mist Maker
unsigned long lastMistToggle = 0;
bool mistCycleState = true; 
const unsigned long MIST_INTERVAL = 60000; // 60.000 ms (1 Menit)

// Kalibrasi Suhu CWT (Update Regresi 2 HTC)
const float SLOPE_TEMP   = 1.035; 
const float OFFSET_TEMP  = -1.2; 
const float ALPHA_TEMP   = 0.3;    // Filter EMA Suhu

// Kalibrasi Kelembapan Digital CWT
const float SLOPE_HUMI   = 1.0;
const float OFFSET_HUMI  = 0.3; 
const float ALPHA_HUMI   = 0.03;   // Filter EMA Kelembapan (Didesain lebih tenang)

// Histeresis Kontrol On/Off
const float HYS_TEMP     = 0.2;
const float HYS_HUMI     = 0.5;

// =================================================================
// 5. VARIABEL GLOBAL & OBJEK MODBUS
// =================================================================
float filteredSuhu = 28.0;
float filteredHumi = 50.0;
float setpointTemp = 0.0;
float setpointHumi = 0.0;

float cwtRawTemp = 0.0;
float cwtRawHumi = 0.0;
bool  cwtOnline  = false;

bool systemRunning = false; 
bool lightStatus   = false; 

ModbusMaster nodeDWIN;
ModbusMaster nodeCWT;

void preTxDWIN()  { digitalWrite(PIN_DRE_DWIN, HIGH); }
void postTxDWIN() { SERIAL_DWIN.flush(); delayMicroseconds(50); digitalWrite(PIN_DRE_DWIN, LOW); }

void preTxCWT()   { digitalWrite(PIN_DRE_CWT, HIGH); }
void postTxCWT()  { SERIAL_CWT.flush(); delayMicroseconds(50); digitalWrite(PIN_DRE_CWT, LOW); }

// =================================================================
// SETUP
// =================================================================
void setup() {
  Serial.begin(115200);        
  Serial2.begin(9600);       
  SERIAL_DWIN.begin(115200);   
  SERIAL_CWT.begin(9600);      

  pinMode(53, OUTPUT); digitalWrite(53, HIGH);
  pinMode(41, OUTPUT); digitalWrite(41, HIGH);
  pinMode(49, OUTPUT); digitalWrite(49, HIGH);

  pinMode(PIN_DRE_DWIN, OUTPUT); digitalWrite(PIN_DRE_DWIN, LOW);
  pinMode(PIN_DRE_CWT, OUTPUT);  digitalWrite(PIN_DRE_CWT, LOW);

  pinMode(pinHeater, OUTPUT);
  pinMode(pinCompressor, OUTPUT);
  pinMode(pinHumidifier, OUTPUT);
  pinMode(pinFanHumidifier, OUTPUT);
  pinMode(pinLight, OUTPUT);
  
  forceAllOff();

  nodeDWIN.begin(1, SERIAL_DWIN);
  nodeDWIN.preTransmission(preTxDWIN);
  nodeDWIN.postTransmission(postTxDWIN);

  nodeCWT.begin(2, SERIAL_CWT);
  nodeCWT.preTransmission(preTxCWT);
  nodeCWT.postTransmission(postTxCWT);

  Serial.println(F("=================================================="));
  Serial.println(F("  CLIMATIC CHAMBER: FULL CWT CONTROL (PT100 OFF)  "));
  Serial.println(F("=================================================="));
}

// =================================================================
// MAIN LOOP
// =================================================================
void loop() {
  static unsigned long lastInputPoll = 0;
  static byte inputStep = 0;
  if (millis() - lastInputPoll >= 100) { 
    lastInputPoll = millis();
    if (inputStep == 0) { checkRunTrigger(); inputStep = 1; } 
    else if (inputStep == 1) { checkLightTrigger(); inputStep = 2; } 
    else if (inputStep == 2) { readSetpoints(); inputStep = 0; }
  }

  static unsigned long lastCWTRead = 0;
  if (millis() - lastCWTRead >= 1000) {
    readSensorCWT();
    lastCWTRead = millis();
  }

  static unsigned long lastLCDUpdate = 0;
  if (millis() - lastLCDUpdate >= 1000) { 
    lastLCDUpdate = millis();
    sendDataToLCD();
    updateActuatorIcons();
  }

  if (systemRunning) {
    kontrolOtomatis();
  } else {
    forceAllOff(); 
  }

  static unsigned long lastESPUpdate = 0;
  if (millis() - lastESPUpdate >= 2000) {
    kirimDataKeESP();
    lastESPUpdate = millis();
  }

  static unsigned long lastDebug = 0;
  if (Serial && (millis() - lastDebug >= 2000)) {
    tampilDebug();
    lastDebug = millis();
  }
}

// =================================================================
// MODUL PEMBACAAN SENSOR CWT RS485
// =================================================================
// =================================================================
// MODUL PEMBACAAN SENSOR CWT RS485 (DENGAN ANTI-NOISE & RATE LIMITER)
// =================================================================
void readSensorCWT() {
  uint8_t result = nodeCWT.readHoldingRegisters(0x0000, 2);

  if (result == nodeCWT.ku8MBSuccess) {
    cwtOnline = true;
    int16_t rawH = (int16_t)nodeCWT.getResponseBuffer(0);
    int16_t rawT = (int16_t)nodeCWT.getResponseBuffer(1);

    cwtRawHumi = rawH / 10.0;
    cwtRawTemp = rawT / 10.0;

    // 1. BENTENG KONTROL KELEMBAPAN (RH)
    if (cwtRawHumi >= 0.0 && cwtRawHumi <= 100.0) {
      float calHumi = cwtRawHumi + OFFSET_HUMI;

      // A. SPIKE REJECTION: Jika data mentah melompat > 10% dalam 1 detik, abaikan!
      static float lastValidRawHumi = 50.0;
      if (abs(calHumi - lastValidRawHumi) > 10.0) {
        calHumi = lastValidRawHumi; // Pakai data lama yang stabil
      } else {
        lastValidRawHumi = calHumi; // Update data valid
      }

      // B. FILTER EMA DASAR
      float targetEMA = (calHumi * ALPHA_HUMI) + (filteredHumi * (1.0 - ALPHA_HUMI));

      // C. HARD RATE LIMITER: Kunci perubahan maksimal 0.05% per detik
      const float MAX_CHANGE_PER_SEC = 0.05; 
      float delta = targetEMA - filteredHumi;

      if (delta > MAX_CHANGE_PER_SEC) {
        filteredHumi += MAX_CHANGE_PER_SEC;
      } else if (delta < -MAX_CHANGE_PER_SEC) {
        filteredHumi -= MAX_CHANGE_PER_SEC;
      } else {
        filteredHumi = targetEMA;
      }
    }

    // 2. KONTROL SUHU (TETAP PRESISI SAMA SEPERTI SEBELUMNYA)
    if (cwtRawTemp >= -40.0 && cwtRawTemp <= 120.0) {
      float calTemp = (cwtRawTemp * SLOPE_TEMP) + OFFSET_TEMP;
      filteredSuhu = (calTemp * ALPHA_TEMP) + (filteredSuhu * (1.0 - ALPHA_TEMP));
    }
  } else {
    cwtOnline = false;
  }
}

// =================================================================
// LOGIKA KONTROL AKTUATOR OTOMATIS
// =================================================================
void forceAllOff() {
  digitalWrite(pinHeater, LOW);
  
  if (digitalRead(pinCompressor) == HIGH) {
    digitalWrite(pinCompressor, LOW);
    compTimer = millis();
    compReady = false;
  } else {
    digitalWrite(pinCompressor, LOW);
  }
  
  digitalWrite(pinHumidifier, LOW);
  digitalWrite(pinFanHumidifier, LOW); 
  
  // Reset siklus mist maker
  mistCycleState = true;
  lastMistToggle = millis();
}

void kontrolOtomatis() {
  if (!cwtOnline) {
    forceAllOff();
    return;
  }

  if (!compReady) {
    if (millis() - compTimer >= COMP_DELAY) {
      compReady = true; 
    }
  }

  bool cmdHeater = digitalRead(pinHeater);
  bool cmdHumid  = digitalRead(pinHumidifier);
  reqComp = false; 

  // 1. LOGIKA KENDALI SUHU (Jika Target Suhu > 0)
  if (setpointTemp > 0.0) {
    if (filteredSuhu < (setpointTemp - HYS_TEMP)) {
      reqComp = false; 
      cmdHeater = true; 
    } else if (filteredSuhu >= setpointTemp) {
      cmdHeater = false; 
    }

    if (filteredSuhu > (setpointTemp + HYS_TEMP)) {
      reqComp = true; 
    } else if (filteredSuhu <= setpointTemp && reqComp == true) {
      reqComp = false; 
    }
  } else {
    cmdHeater = false; 
  }

  // 2. LOGIKA KENDALI KELEMBAPAN (Jika Target RH > 0)
  if (setpointHumi > 0.0) {
    if (filteredHumi < (setpointHumi - HYS_HUMI)) {
      
      // SIKLUS 1 MENIT ON / 1 MENIT OFF
      if (millis() - lastMistToggle >= MIST_INTERVAL) {
        lastMistToggle = millis();
        mistCycleState = !mistCycleState; // Berganti status ON/OFF
      }
      cmdHumid = mistCycleState;

    } else if (filteredHumi >= setpointHumi) {
      cmdHumid = false; 
      mistCycleState = true; // Reset agar saat dibutuhkan langsung ON dari menit pertama
      lastMistToggle = millis();
    }

    // OVERRIDE DEHUMIDIFIKASI (SUBCOOL & REHEAT)
    if (filteredHumi > (setpointHumi + HYS_HUMI)) {
      reqComp = true;   
      cmdHumid = false; 
      
      if (setpointTemp > 0.0 && filteredSuhu <= setpointTemp) {
        cmdHeater = true; 
      }
    }
  } else {
    cmdHumid = false; 
    mistCycleState = true;
  }

  // 3. EKSEKUSI KE PIN HARDWARE
  digitalWrite(pinHeater, cmdHeater ? HIGH : LOW);
  digitalWrite(pinHumidifier, cmdHumid ? HIGH : LOW);
  
  // Kipas Mist Maker murni mengikuti status Mist Maker (Pin A13)
  digitalWrite(pinFanHumidifier, cmdHumid ? HIGH : LOW);

  if (reqComp) {
    if (compReady) {
      digitalWrite(pinCompressor, HIGH);
    }
  } else {
    if (digitalRead(pinCompressor) == HIGH) {
      digitalWrite(pinCompressor, LOW);
      compTimer = millis(); 
      compReady = false;    
    }
  }
}

// =================================================================
// MODUL MODBUS LAYAR DWIN
// =================================================================
void checkLightTrigger() {
  if (nodeDWIN.readHoldingRegisters(VP_LIGHT_TRIGGER, 1) == nodeDWIN.ku8MBSuccess) {
    if (nodeDWIN.getResponseBuffer(0) == 1) {
      lightStatus = !lightStatus;
      digitalWrite(pinLight, lightStatus ? HIGH : LOW);
      nodeDWIN.writeSingleRegister(VP_LIGHT_TRIGGER, 0); 
    }
  }
}

void checkRunTrigger() {
  if (nodeDWIN.readHoldingRegisters(VP_RUN_TRIGGER, 1) == nodeDWIN.ku8MBSuccess) {
    if (nodeDWIN.getResponseBuffer(0) == 1) {
      systemRunning = !systemRunning;
      nodeDWIN.writeSingleRegister(VP_RUN_ICON, systemRunning ? 1 : 0); 
      nodeDWIN.writeSingleRegister(VP_RUN_TRIGGER, 0); 
    }
  }
}

void readSetpoints() {
  if (nodeDWIN.readHoldingRegisters(VP_SET_TEMP, 1) == nodeDWIN.ku8MBSuccess) {
    int16_t val = (int16_t)nodeDWIN.getResponseBuffer(0);
    if (val >= 0) setpointTemp = val / 100.0; 
  }
  if (nodeDWIN.readHoldingRegisters(VP_SET_HUMI, 1) == nodeDWIN.ku8MBSuccess) {
    int16_t val = (int16_t)nodeDWIN.getResponseBuffer(0);
    if (val >= 0) setpointHumi = val / 100.0; 
  }
}

void sendDataToLCD() {
  nodeDWIN.writeSingleRegister(VP_SUHU_READ, (int16_t)round(filteredSuhu * 10.0));
  nodeDWIN.writeSingleRegister(VP_HUMI_READ, (int16_t)round(filteredHumi * 10.0)); 
}

void updateActuatorIcons() {
  nodeDWIN.writeSingleRegister(VP_ICON_RH,   digitalRead(pinHumidifier)); 
  
  static uint16_t blinkState = 0;
  blinkState = !blinkState; 
  
  uint16_t stateComp = 0;
  if (digitalRead(pinCompressor) == HIGH) {
    stateComp = 1; 
  } else if (reqComp == true && compReady == false) {
    stateComp = blinkState; 
  } else {
    stateComp = 0; 
  }
  
  nodeDWIN.writeSingleRegister(VP_ICON_COMP, stateComp); 
  nodeDWIN.writeSingleRegister(VP_ICON_HEAT, digitalRead(pinHeater));     
}

// =================================================================
// TELEMETRI & SERIAL MONITOR
// =================================================================
void kirimDataKeESP() {
  Serial2.print(filteredSuhu, 1); Serial2.print(",");
  Serial2.print(filteredHumi, 1); Serial2.print(",");
  Serial2.print(digitalRead(pinHeater)); Serial2.print(",");
  Serial2.print(digitalRead(pinCompressor)); Serial2.print(",");
  Serial2.print(digitalRead(pinHumidifier)); Serial2.print(",");
  Serial2.println(systemRunning ? 1 : 0); 
}

void tampilDebug() {
  Serial.println(F("\n>>> CLIMATIC CHAMBER (FULL CWT SENSOR) <<<"));
  Serial.print(F("STATE       : ")); Serial.println(systemRunning ? F("[ RUNNING ]") : F("[ STANDBY ]"));
  Serial.print(F("SUHU (CWT)  : ")); Serial.print(filteredSuhu, 2); Serial.println(F(" C"));
  Serial.print(F("HUMI (CWT)  : ")); Serial.print(filteredHumi, 1); Serial.println(F(" %"));
  Serial.print(F("STATUS CWT  : ")); Serial.println(cwtOnline ? F("[ ONLINE ]") : F("[ TIMEOUT / OFFLINE ]"));
  
  Serial.println(F("--- STATUS AKTUATOR ---"));
  Serial.print(F("Heater      : ")); Serial.println(digitalRead(pinHeater) ? F("ON") : F("OFF"));
  Serial.print(F("Compressor  : ")); Serial.println(digitalRead(pinCompressor) ? F("ON") : F("OFF"));
  Serial.print(F("Humidifier  : ")); Serial.println(digitalRead(pinHumidifier) ? F("ON") : F("OFF"));
  Serial.print(F("Fan Mist    : ")); Serial.println(digitalRead(pinFanHumidifier) ? F("ON") : F("OFF"));
  Serial.print(F("Lampu       : ")); Serial.println(digitalRead(pinLight) ? F("ON") : F("OFF"));
  Serial.println(F("------------------------------------------"));
}