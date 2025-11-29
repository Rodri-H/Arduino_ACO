/********
  Proyecto Domótica (para diagram.json proporcionado)
  Arduino UNO + DHT22 + PIR + MQ-2 + LDR + Buzzer + Relay + LCD I2C
  Mapeo de pines (según diagram.json):
    - DHT22 SDA   -> D2
    - PIR OUT     -> D7
    - MQ-2 AOUT   -> A0
    - LDR SIG     -> A1
    - Buzzer SIG  -> D8
    - Relay IN    -> D6
    - LCD I2C     -> SDA A4, SCL A5
  Serial CSV @115200:
    T:xx.x,H:xx.x,MQ2:nnn,LDR:nnn,PIR:nnn
  Comandos seriales:
    RELAY:1  -> enciende relé
    RELAY:0  -> apaga relé
    SET:GAS:300 -> ajusta umbral gas (analógico)
    SET:LDR:400 -> ajusta umbral LDR (analógico)
    SET:TEMP:28.5 -> ajusta umbral temperatura
********/

#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// --- DHT ---
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// --- LCD I2C (dirección común 0x27) ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
Servo misCortinas;

// --- Pines ---
const int PIN_MQ2   = A0;
const int PIN_LDR   = A1;
const int PIN_PIR   = 7;  // digital output from PIR
const int PIN_RELAY = 6;
const int PIN_BUZZ  = 8;
const int PIN_BTN   = 4;  // Botón Interfaz
const int PIN_LED   = 5;  // LED rojo (movimiento)
const int PIN_LED2  = 3;  // LED blanco (noche)
const int PIN_SERVO = 9;  // Cortinas (Actuador 4)

// --- Umbrales (ajustables) ---
float TEMP_THRESHOLD = 28.0;   // °C
int   GAS_THRESHOLD  = 300;    // 0..1023 (umbral analógico original para relay/LCD)
int   LDR_THRESHOLD  = 400;    // 0..1023
int   LDR_THRESHOLD_CORTINA = 500; // Umbral de luz para cerrar/abrir cortina

// Umbral de gas en PPM para el BUZZER
const float GAS_PPM_THRESHOLD = 3.0f;   // cuando gas >= 3.0 ppm -> buzzer ON

// Variables para controlar el botón (anti-rebote)
bool estadoLampara   = false;
bool ultimoEstadoBtn = HIGH;

unsigned long lastLCD = 0;
const unsigned long LCD_INTERVAL = 1200; // ms

// Serial buffer
String serialBuf = "";

// --- Constantes para convertir LDR -> lux (de docs Wokwi) ---
const float LDR_GAMMA = 0.7;
const float LDR_RL10  = 50.0;

// ---------- utilidades ----------

// Promedio simple de lecturas analógicas
int analogAvg(int pin, int samples = 8, int delayMs = 6) {
  long s = 0;
  for (int i = 0; i < samples; ++i) {
    s += analogRead(pin);
    delay(delayMs);
  }
  return (int)(s / samples);
}

// Convierte lectura analógica del MQ-2 a PPM (modelo simple lineal para simulación)
// 0 -> 0 ppm, 1023 -> 10 ppm (ajusta 10.0f según necesites)
float mq2AnalogToPPM(int analogValue) {
  return (analogValue / 1023.0f) * 10.0f;
}

// Convierte lectura analógica del LDR a lux (fórmula recomendada por Wokwi)
float ldrAnalogToLux(int analogValue) {
  if (analogValue <= 0) {
    return INFINITY;  // muy brillante o error
  }
  float voltage    = analogValue / 1024.0f * 5.0f;
  if (voltage <= 0.0f || voltage >= 5.0f) {
    return INFINITY;
  }
  float resistance = 2000.0f * voltage / (1.0f - voltage / 5.0f);
  float lux = pow(LDR_RL10 * 1e3 * pow(10, LDR_GAMMA) / resistance, (1.0f / LDR_GAMMA));
  return lux;
}

void processSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length() > 0) {
        String cmd = serialBuf;
        cmd.trim();
        // RELAY
        if (cmd.startsWith("RELAY:")) {
          String v = cmd.substring(6);
          v.trim();
          if (v == "1") {
            digitalWrite(PIN_RELAY, HIGH);
            Serial.println("ACK:RELAY:1");
          } else if (v == "0") {
            digitalWrite(PIN_RELAY, LOW);
            Serial.println("ACK:RELAY:0");
          } else Serial.println("ERR:RELAY");
        }
        // SET thresholds: SET:GAS:300, SET:LDR:400, SET:TEMP:28.5
        else if (cmd.startsWith("SET:GAS:")) {
          String v = cmd.substring(8); GAS_THRESHOLD = v.toInt();
          Serial.print("ACK:GAS:"); Serial.println(GAS_THRESHOLD);
        } else if (cmd.startsWith("SET:LDR:")) {
          String v = cmd.substring(8); LDR_THRESHOLD = v.toInt();
          Serial.print("ACK:LDR:"); Serial.println(LDR_THRESHOLD);
        } else if (cmd.startsWith("SET:TEMP:")) {
          String v = cmd.substring(9); TEMP_THRESHOLD = v.toFloat();
          Serial.print("ACK:TEMP:"); Serial.println(TEMP_THRESHOLD);
        } else {
          Serial.print("UNK:"); Serial.println(cmd);
        }
        serialBuf = "";
      }
    } else {
      serialBuf += c;
      if (serialBuf.length() > 150) serialBuf = "";
    }
  }
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
  dht.begin();
  lcd.init();
  lcd.backlight();
  misCortinas.attach(PIN_SERVO);  // Conectar el servo al pin 9

  pinMode(PIN_PIR,   INPUT);
  pinMode(PIN_BTN,   INPUT_PULLUP); // Botón con resistencia interna activada
  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZ,  OUTPUT);
  pinMode(PIN_LED,   OUTPUT);       // LED rojo
  pinMode(PIN_LED2,  OUTPUT);       // LED blanco

  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_BUZZ,  LOW);
  digitalWrite(PIN_LED,   LOW);     // Asegurar que la lámpara inicie apagada
  digitalWrite(PIN_LED2,  LOW);     // LED blanco inicia apagado
  misCortinas.write(0);             // Iniciar con cortinas cerradas

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Sistema Domotico");
  lcd.setCursor(0,1);
  lcd.print("Iniciando...");
  delay(800);
  lcd.clear();

  Serial.println("DOMOTICA: listo");
  Serial.println("Formato CSV: T:xx.x,H:xx.x,MQ2:nnn,LDR:nnn,PIR:nnn");
}

// ---------- loop ----------
void loop() {
  // Procesar comandos por Serial
  processSerial();

  // Leer sensores
  float temp = dht.readTemperature();
  float hum  = dht.readHumidity();
  int   mq2  = analogRead(PIN_MQ2);  // Lectura directa sin promedio para mejor respuesta
  int   ldr  = analogAvg(PIN_LDR, 8, 6);
  int   pir  = digitalRead(PIN_PIR);      // HIGH cuando hay movimiento
  int   lecturaBtn = digitalRead(PIN_BTN);

  // Conversiones
  float gasPPM = mq2AnalogToPPM(mq2);     // gas en PPM aprox (0–10 ppm)
  float lux    = ldrAnalogToLux(ldr);     // lux estimados a partir del LDR

  // Serial CSV output para Node-RED / logging
  Serial.print("T:");   Serial.print(isnan(temp) ? 0.0 : temp, 1);
  Serial.print(",H:");  Serial.print(isnan(hum) ? 0.0 : hum, 1);
  Serial.print(",MQ2:"); Serial.print(mq2);
  Serial.print(",LDR:"); Serial.print(ldr);
  Serial.print(",LUX:"); Serial.print(isfinite(lux) ? lux : -1.0f, 2);
  Serial.print(",PIR:"); Serial.print(pir);
  Serial.print(",GASPPM:"); Serial.println(gasPPM, 3);

  // --- Lógica del botón (solo cambia estado lógico de la lámpara) ---
  if (lecturaBtn == LOW && ultimoEstadoBtn == HIGH) {
    estadoLampara = !estadoLampara; // Cambiar estado (Encendido <-> Apagado)
    delay(50); // Pequeña pausa para evitar rebote
  }
  ultimoEstadoBtn = lecturaBtn;

  // ---------- Lógica de BUZZER ----------
  // Buzzer ON si:
  // - Hay movimiento (PIR = HIGH)
  // - O gasPPM >= 3.0 ppm
  bool movimientoDetectado = (pir == HIGH);
  bool gasAlarmaPPM        = (gasPPM >= GAS_PPM_THRESHOLD);

  if (movimientoDetectado || gasAlarmaPPM) {
    tone(PIN_BUZZ, 2000);  // 2 kHz
  } else {
    noTone(PIN_BUZZ);
  }

  // ---------- Lógica de RELAY (ventilador) ----------
  // Mantenemos tu lógica original con GAS_THRESHOLD (analógico) y TEMP_THRESHOLD
  bool gasAlarmaAnalog = (mq2 >= GAS_THRESHOLD);
  bool tempAlta        = (!isnan(temp) && temp > TEMP_THRESHOLD);

  if (gasAlarmaAnalog || tempAlta) {
    digitalWrite(PIN_RELAY, HIGH);
  } else {
    digitalWrite(PIN_RELAY, LOW);
  }

  // ---------- LED rojo según detección de movimiento ----------
  // LED se enciende ÚNICAMENTE cuando el PIR detecta movimiento
  if (movimientoDetectado) {
    digitalWrite(PIN_LED, HIGH);
  } else {
    digitalWrite(PIN_LED, LOW);
  }

  // ---------- LED blanco (lámpara nocturna) ----------
  // LED2 se enciende cuando lux < 150 (de noche)
  if (isfinite(lux) && lux < 150.0f) {
    digitalWrite(PIN_LED2, HIGH);
  } else {
    digitalWrite(PIN_LED2, LOW);
  }

  // ---------- Update LCD periódicamente ----------
  if (millis() - lastLCD > LCD_INTERVAL) {
    lastLCD = millis();
    lcd.clear();
    if (!isnan(temp)) {
      lcd.setCursor(0,0);
      lcd.print("T:");
      lcd.print(temp, 1);
      lcd.print((char)223); lcd.print("C ");
      lcd.print("H:");
      lcd.print(hum,0); lcd.print("%");
    } else {
      lcd.setCursor(0,0); lcd.print("DHT Err");
    }

    lcd.setCursor(0,1);
    if (gasAlarmaAnalog) {
      lcd.print("GAS! ");
    } else {
      lcd.print("G:"); lcd.print(mq2);
      lcd.print(" ");
    }
    if (digitalRead(PIN_RELAY)) lcd.print("FAN:on");
    else                        lcd.print("FAN:off");
  }

  // ---------- Lógica Servo Cortinas ----------
  // Basado en LUX (no en valor analógico del LDR)
  // >= 600 lux = Mucha luz = Cerrar cortinas (0°)
  // < 600 lux  = Poca luz  = Abrir cortinas (90°)
  
  if (isfinite(lux)) {  // Verificar que lux sea un valor válido
    if (lux >= 600.0f) { 
       misCortinas.write(0);  // Cerrar cortinas (Mucha luz exterior)
    } else {
       misCortinas.write(90); // Abrir cortinas (Poca luz exterior)
    }
  }

  delay(120); // pequeño retardo para estabilidad
}