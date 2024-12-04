#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WebServer.h>

// Inicializar objetos y variables
Adafruit_MPU6050 mpu;
WebServer server(80);

// Configuración Wi-Fi
const char *ssid = "Ramon";       
const char *password = "Juanramon0919";

// Variables globales
bool mpuActive = false;
bool breathingAlert = false;
bool movementAlert = false;
unsigned long lastBreathDetected = 0;
const unsigned long noBreathingThreshold = 5000;

// Datos del sensor de pulso
int threshold = 2000, bpm = 0;

// Multiplexor y termistores
const int PulseSensorPin = 35, LEDPin = 2, pinSIG = 34;
const int S0 = 23, S1 = 22, S2 = 21, S3 = 19;
float temperaturas[8] = {0}; 
float promedioTemperaturas = 0;

// Parámetros del divisor de tensión y termistor NTC
const float resistenciaSerie = 10000.0, beta = 3950.0, resistenciaRef = 10000.0, tempRef = 298.15;

// Funciones del servidor web
void handleRoot() {
  String json = "{";
  json += "\"breathingAlert\": " + String(breathingAlert) + ",";
  json += "\"movementAlert\": " + String(movementAlert) + ",";
  json += "\"bpm\": " + String(bpm) + ",";
  json += "\"averageTemperature\": " + String(promedioTemperaturas);
  json += "}";
  Serial.println(json);
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not Found");
}

void setup() {
  Serial.begin(115200);

  // Configuración Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Conectando a Wi-Fi...");
  }
  Serial.println("Conectado a Wi-Fi: " + WiFi.localIP().toString());

  // Configuración del servidor web
  server.on("/", handleRoot);
  server.onNotFound(handleNotFound);
  server.begin();

  // Configurar MPU6050
  if (mpu.begin()) {
    Serial.println("MPU6050 inicializado correctamente.");
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
    mpuActive = true;
  } else {
    Serial.println("No se detectó el MPU6050.");
  }

  // Configurar pines del multiplexor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(LEDPin, OUTPUT);
}

void loop() {
  // Mantener el servidor web funcionando
  server.handleClient();

  // Actualizar temperaturas y calcular el promedio
  float sumaTemperaturas = 0;
  for (int i = 0; i < 8; i++) {
    seleccionarEntrada(i);
    delay(10);
    temperaturas[i] = leerTemperatura();
    sumaTemperaturas += temperaturas[i];
  }
  promedioTemperaturas = sumaTemperaturas / 8.0;

  if (mpuActive) {
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    // Procesar respiración y movimiento
    detectBreathing(accel.acceleration.z);
    detectMovement(accel, gyro);
  }

  // Leer datos del sensor de pulso
  detectBeat();

  delay(100); // Ajusta la frecuencia según necesidad
}

// Funciones auxiliares
void detectBreathing(float zAccel) {
  static float previousZAccel = 0;
  if (abs(zAccel - previousZAccel) > 0.05) {
    lastBreathDetected = millis();
    breathingAlert = false;
  }
  previousZAccel = zAccel;
  if (millis() - lastBreathDetected > noBreathingThreshold) breathingAlert = true;
}

void detectMovement(sensors_event_t accel, sensors_event_t gyro) {
  movementAlert = abs(accel.acceleration.x) > 0.5 || abs(accel.acceleration.y) > 0.5 || abs(accel.acceleration.z) > 0.5 ||
                  abs(gyro.gyro.x) > 0.5 || abs(gyro.gyro.y) > 0.5 || abs(gyro.gyro.z) > 0.5;
}

void detectBeat() {
  static unsigned long lastBeatTime = 0;
  unsigned long currentTime = millis();
  int rawSignal = analogRead(PulseSensorPin);
  if (rawSignal > threshold && (currentTime - lastBeatTime > 300)) {
    bpm = 60000 / (currentTime - lastBeatTime);
    lastBeatTime = currentTime;
    digitalWrite(LEDPin, HIGH);
  } else {
    digitalWrite(LEDPin, LOW);
  }
}

void seleccionarEntrada(int entrada) {
  digitalWrite(S0, entrada & 0x01);
  digitalWrite(S1, (entrada >> 1) & 0x01);
  digitalWrite(S2, (entrada >> 2) & 0x01);
  digitalWrite(S3, (entrada >> 3) & 0x01);
}

float leerTemperatura() {
  int adcValor = analogRead(pinSIG);
  float voltaje = adcValor * (3.3 / 4095.0);
  float resistenciaNTC = resistenciaSerie * (3.3 / voltaje - 1);
  float temperaturaK = 1.0 / (1.0 / tempRef + log(resistenciaNTC / resistenciaRef) / beta);
  return temperaturaK - 273.15;
}

