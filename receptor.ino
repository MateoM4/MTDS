#include <WiFi.h>
#include <WiFiUdp.h>

// ===== CONFIGURACIÓN AP (La red que crea el robot) =====
const char* ssid = "MTDS_ROBOT_AP";
const char* password = "robotseguro"; // Mínimo 8 caracteres
const int localPort = 4210;           // Puerto de escucha UDP

// ===== OBJETOS =====
WiFiUDP udp;
// UART hacia Arduino Mega
HardwareSerial MegaSerial(2); // UART2 (RX=16, TX=17)

// Buffer para paquetes entrantes
char packetBuffer[255];

void setup() {
  Serial.begin(115200);
  MegaSerial.begin(115200, SERIAL_8N1, 16, 17);

  // 1. Iniciar en modo Access Point (AP)
  Serial.println("Iniciando Modo AP...");
  WiFi.softAP(ssid, password);
  
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Dirección IP del AP: ");
  Serial.println(myIP); // Normalmente es 192.168.4.1

  // 2. Iniciar escucha UDP
  udp.begin(localPort);
  Serial.printf("Escuchando UDP en puerto %d\n", localPort);
}

void loop() {
  // Verificar si hay paquetes UDP disponibles
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    // Leer el paquete
    int len = udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0; // Terminar string

    int vx = 0, vy = 0;
    unsigned long sentTime = 0;

    // Esperamos formato: "vx,vy,timestamp"
    if (sscanf(packetBuffer, "%d,%d,%lu", &vx, &vy, &sentTime) == 3) {
      
      // Cálculo de Latencia (Un solo viaje: Emisor -> Receptor)
      unsigned long latency = millis() - sentTime;

      // Enviar al Mega (Protocolo robusto)
      MegaSerial.printf("<%d,%d>\n", vx, vy);

      // Diagnóstico de Latencia
      Serial.printf("Latencia UDP: %lu ms | VX: %d\n", latency, vx);

    } else {
      Serial.println("Paquete corrupto o formato incorrecto");
    }
  }
  // Nota: No necesitamos lógica de reconexión MQTT aquí.
  // El Watchdog del Arduino Mega detendrá el robot si dejan de llegar paquetes.
}
