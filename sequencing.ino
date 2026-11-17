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
// 4. PARAMETER KALIBRASI, FILTER & PRESISI ±0.5
// =================================================================
const unsigned long COMP_DELAY = 30000; // Jeda proteksi kompresor (30 detik)
bool compReady = true;
unsigned long compTimer = 0;
bool reqComp = false;

// Kalibrasi & Filter Suhu CWT
const float SLOPE_TEMP   = 1.035; 
const float OFFSET_TEMP  = -1.2; 
const float ALPHA_TEMP   = 0.3;   

// Kalibrasi & Filter Kelembapan CWT
const float SLOPE_HUMI   = 1.0;
const float OFFSET_HUMI  = 0.3; 
const float ALPHA_HUMI   = 0.03;   

// Target Presisi ±0.5
const float HYS_TEMP     = 0.2;    // Target Suhu ±0.5 C
const float HYS_HUMI     = 0.2;    // Target RH ±0.5 %

// Zona Pengereman (Soft Landing) sebelum menyentuh setpoint
const float BRAKE_ZONE_TEMP = 1.5; // Mulai mengerem 1.5 C sebelum target
const float BRAKE_ZONE_HUMI = 3.0; // Mulai mengerem 3.0 % sebelum target

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
byte currentCase   = 0;

// Variabel Duty Cycle (0.0 - 1.0) untuk Slow PWM
float dutyHeater = 0.0;
float dutyHumid  = 0.0;

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
    eksekusiSlowPWM(); // Eksekusi denyut halus setiap siklus loop
  } else {
    forceAllOff(); 
    currentCase = 0;
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
void readSensorCWT() {
  uint8_t result = nodeCWT.readHoldingRegisters(0x0000, 2);

  if (result == nodeCWT.ku8MBSuccess) {
    cwtOnline = true;
    int16_t rawH = (int16_t)nodeCWT.getResponseBuffer(0);
    int16_t rawT = (int16_t)nodeCWT.getResponseBuffer(1);

    cwtRawHumi = rawH / 10.0;
    cwtRawTemp = rawT / 10.0;

    if (cwtRawHumi >= 0.0 && cwtRawHumi <= 100.0) {
      float calHumi = (cwtRawHumi * SLOPE_HUMI) + OFFSET_HUMI;
      static float lastValidRawHumi = 50.0;
      if (abs(calHumi - lastValidRawHumi) > 10.0) {
        calHumi = lastValidRawHumi;
      } else {
        lastValidRawHumi = calHumi;
      }
      filteredHumi = (calHumi * ALPHA_HUMI) + (filteredHumi * (1.0 - ALPHA_HUMI));
    }

    if (cwtRawTemp >= -40.0 && cwtRawTemp <= 120.0) {
      float calTemp = (cwtRawTemp * SLOPE_TEMP) + OFFSET_TEMP;
      filteredSuhu = (calTemp * ALPHA_TEMP) + (filteredSuhu * (1.0 - ALPHA_TEMP));
    }
  } else {
    cwtOnline = false;
  }
}

// =================================================================
// LOGIKA KONTROL PRESISI (9 CASE + PROPORTIONAL BRAKING)
// =================================================================
void forceAllOff() {
  dutyHeater = 0.0;
  dutyHumid  = 0.0;
  reqComp    = false;

  digitalWrite(pinHeater, LOW);
  digitalWrite(pinHumidifier, LOW);
  digitalWrite(pinFanHumidifier, LOW);
  
  if (digitalRead(pinCompressor) == HIGH) {
    digitalWrite(pinCompressor, LOW);
    compTimer = millis();
    compReady = false;
  } else {
    digitalWrite(pinCompressor, LOW);
  }
}

void kontrolOtomatis() {
  if (!cwtOnline || setpointTemp <= 0.0 || setpointHumi <= 0.0) {
    forceAllOff();
    currentCase = 0;
    return;
  }

  if (!compReady) {
    if (millis() - compTimer >= COMP_DELAY) {
      compReady = true; 
    }
  }

  float errTemp = setpointTemp - filteredSuhu; // Positif = Kurang panas
  float errHumi = setpointHumi - filteredHumi; // Positif = Kurang lembap

  // Klasifikasi Status (-1: Kurang, 0: Masuk ±0.5, 1: Lebih)
  int stateTemp = (errTemp > HYS_TEMP) ? -1 : ((errTemp < -HYS_TEMP) ? 1 : 0);
  int stateHumi = (errHumi > HYS_HUMI) ? -1 : ((errHumi < -HYS_HUMI) ? 1 : 0);

  // Hitung daya proporsional (Soft Landing) saat mendekati setpoint
  // Jika selisih >= BRAKE_ZONE -> 100% (1.0). Jika selisih kecil -> mengecil hingga min 25% (0.25)
  float propHeat  = constrain(errTemp / BRAKE_ZONE_TEMP, 0.25, 1.0);
  float propHumid = constrain(errHumi / BRAKE_ZONE_HUMI, 0.25, 1.0);

  dutyHeater = 0.0;
  dutyHumid  = 0.0;
  reqComp    = false;

  // --- MATRIKS 9 CASE ---
  if (stateTemp == 0 && stateHumi == 0) {
    // CASE 1: Tercapai semua (±0.5) -> Total Standby
    currentCase = 1;
  }
  else if (stateTemp == 0 && stateHumi == -1) {
    // CASE 2: Suhu pas, RH kurang -> Mist Maker berdenyut halus sesuai jarak error
    currentCase = 2;
    dutyHumid = propHumid;
  }
  else if (stateTemp == 0 && stateHumi == 1) {
    // CASE 3: Suhu pas, RH lebih -> Kompresor ON + Heater penyeimbang (50% daya)
    currentCase = 3;
    reqComp = true;
    // Heater hanya menyala jika suhu mulai turun di bawah setpoint (mencegah overshoot panas)
    dutyHeater = (errTemp > -0.1) ? 0.50 : 0.0; 
  }
  else if (stateTemp == -1 && stateHumi == 0) {
    // CASE 4: Suhu kurang, RH pas -> Heater berdenyut halus
    currentCase = 4;
    dutyHeater = propHeat;
  }
  else if (stateTemp == 1 && stateHumi == 0) {
    // CASE 5: Suhu lebih, RH pas -> Kompresor ON
    currentCase = 5;
    reqComp = true;
  }
  else if (stateTemp == 1 && stateHumi == -1) {
    // CASE 6: Suhu lebih, RH kurang -> Selesaikan suhu dulu dengan Kompresor
    currentCase = 6;
    reqComp = true;
  }
  else if (stateTemp == -1 && stateHumi == -1) {
    // CASE 7: Suhu kurang, RH kurang -> Heater + Mist Maker (keduanya mengerem halus)
    currentCase = 7;
    dutyHeater = propHeat;
    dutyHumid  = propHumid;
  }
  else if (stateTemp == 1 && stateHumi == 1) {
    // CASE 8: Suhu lebih, RH lebih -> Kompresor menurunkan keduanya
    currentCase = 8;
    reqComp = true;
  }
  else if (stateTemp == -1 && stateHumi == 1) {
    // CASE 9: Suhu kurang, RH lebih -> Buang RH dulu pakai Kompresor
    currentCase = 9;
    reqComp = true;
  }
}

// =================================================================
// EKSEKUSI AKTUATOR (HEATER ON/OFF MURNI TANPA CLICK-CLICK)
// =================================================================
void eksekusiSlowPWM() {
  // 1. Heater: On/Off Murni (Relay mekanik aman, tidak bunyi click-click)
  digitalWrite(pinHeater, (dutyHeater > 0.0) ? HIGH : LOW);

  // 2. Mist Maker: On/Off Murni
  bool humidOn = (dutyHumid > 0.0);
  digitalWrite(pinHumidifier, humidOn ? HIGH : LOW);
  digitalWrite(pinFanHumidifier, humidOn ? HIGH : LOW);

  // 3. Eksekusi Kompresor (Tetap On/Off dengan Proteksi Delay)
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
// MODUL MODBUS LAYAR DWIN & TELEMETRI (TETAP SAMA)
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
  nodeDWIN.writeSingleRegister(VP_ICON_RH, (dutyHumid > 0.0) ? 1 : 0); 
  
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
  nodeDWIN.writeSingleRegister(VP_ICON_HEAT, (dutyHeater > 0.0) ? 1 : 0);     
}

void kirimDataKeESP() {
  Serial2.print(filteredSuhu, 1); Serial2.print(",");
  Serial2.print(filteredHumi, 1); Serial2.print(",");
  Serial2.print(digitalRead(pinHeater)); Serial2.print(",");
  Serial2.print(digitalRead(pinCompressor)); Serial2.print(",");
  Serial2.print(digitalRead(pinHumidifier)); Serial2.print(",");
  Serial2.println(systemRunning ? 1 : 0); 
}

void tampilDebug() {
  Serial.println(F("\n>>> CLIMATIC CHAMBER (PRECISION ±0.5) <<<"));
  Serial.print(F("ACTIVE CASE  : CASE ")); Serial.println(currentCase);
  Serial.print(F("SUHU (CWT)   : ")); Serial.print(filteredSuhu, 2); Serial.print(F(" / ")); Serial.print(setpointTemp, 1); Serial.println(F(" C"));
  Serial.print(F("HUMI (CWT)   : ")); Serial.print(filteredHumi, 2); Serial.print(F(" / ")); Serial.print(setpointHumi, 1); Serial.println(F(" %"));
  Serial.print(F("DUTY HEAT/RH : ")); Serial.print(dutyHeater * 100, 0); Serial.print(F("% / ")); Serial.print(dutyHumid * 100, 0); Serial.println(F("%"));
  Serial.print(F("COMPRESSOR   : ")); Serial.println(digitalRead(pinCompressor) ? F("ON") : (reqComp ? F("WAITING DELAY") : F("OFF")));
  Serial.println(F("------------------------------------------"));
}
