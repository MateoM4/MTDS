#include <WiFi.h>
#include <WiFiUdp.h>

// ===== CONFIGURACIÓN CONEXIÓN =====
const char* ssid = "MTDS_ROBOT_AP";     // Debe coincidir con la Receptora
const char* password = "robotseguro";
const char* udpAddress = "192.168.4.1"; // IP por defecto del AP ESP32
const int udpPort = 4210;

// ===== PINES JOYSTICK =====
const int JOY_X_PIN = 34;
const int JOY_Y_PIN = 35;

// ===== OBJETOS =====
WiFiUDP udp;

// ===== PARÁMETROS DE CONTROL =====
const int DEADZONE = 40;
const int MAX_SPEED = 200;
const int SEND_INTERVAL = 20; // 50Hz (Muy rápido, ideal para UDP)

unsigned long lastSend = 0;

int joystickToSpeed(int value) {
  int center = 1840;  // Ajusta esto según tu joystick si no está centrado
  int delta = value - center;
  if (abs(delta) < DEADZONE) return 0;
  return map(delta, -1840, 1840, -MAX_SPEED, MAX_SPEED);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Conectar a la red del Robot
  Serial.print("Conectando al Robot AP");
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado al Robot!");
  Serial.print("IP Asignada: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Re-conectar si se pierde el WiFi (Seguridad)
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi perdido, reconectando...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(500);
    return;
  }

  unsigned long now = millis();
  if (now - lastSend >= SEND_INTERVAL) {
    lastSend = now;

    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    int vx = joystickToSpeed(rawX);
    int vy = joystickToSpeed(rawY);

    // Preparar mensaje: vx,vy,timestamp
    char msg[64];
    snprintf(msg, sizeof(msg), "%d,%d,%lu", vx, vy, now);

    // Envío UDP
    udp.beginPacket(udpAddress, udpPort);
    udp.print(msg);
    udp.endPacket();

    // Debug local (opcional, comentar para mayor velocidad)
    // Serial.println(msg); 
  }
}
