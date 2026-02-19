#include <WiFi.h>
#include <WiFiUdp.h>

// =======================================================
// CONFIGURACIÓN ACCESS POINT
// =======================================================

const char *ssid = "MTDS_ROBOT_AP";
const char *password = "robotseguro";
const uint16_t localPort = 4210;

// =======================================================
// CONFIGURACIÓN GENERAL
// =======================================================

constexpr uint8_t MAX_CLIENTS = 1;    // Solo 1 controlador
constexpr uint32_t TIMEOUT_MS = 1000; // Tiempo máximo sin paquetes

WiFiUDP udp;
HardwareSerial MegaSerial(2); // UART2 (RX=16, TX=17)

unsigned long lastPacketTime = 0;

// =======================================================
// DEFINICIÓN DEL PAQUETE BINARIO (3 BYTES EXACTOS)
// =======================================================

#pragma pack(push, 1)
typedef struct __attribute__((packed))
{
  int16_t joyX; // Cambiado a 16 bits con signo
  int16_t joyY; // Cambiado a 16 bits con signo
  uint8_t pot;  // El pot de 0-255 está perfecto en 8 bits sin signo
} DataPacket;
#pragma pack(pop)

// ACTUALIZAR EL CONTROL DE TAMAÑO
static_assert(sizeof(DataPacket) == 5, "El struct no mide 5 bytes");

// =======================================================
// SETUP
// =======================================================

void setup()
{
  Serial.begin(115200);
  MegaSerial.begin(115200, SERIAL_8N1, 16, 17);

  Serial.println("Iniciando Access Point...");

  // Iniciar AP en canal 6, visible, máximo 1 cliente
  if (!WiFi.softAP(ssid, password, 6, false, MAX_CLIENTS))
  {
    Serial.println("Error iniciando Access Point");
    while (true)
      ; // Bloquear si falla
  }

  Serial.print("IP del AP: ");
  Serial.println(WiFi.softAPIP());

  // Iniciar UDP
  if (!udp.begin(localPort))
  {
    Serial.println("Error iniciando UDP");
    while (true)
      ;
  }

  Serial.printf("Escuchando UDP en puerto %d\n", localPort);
}

// =======================================================
// LOOP PRINCIPAL
// =======================================================

void loop()
{

  int packetSize = udp.parsePacket();

  // Solo aceptamos paquetes EXACTAMENTE de 5 bytes
  if (packetSize == sizeof(DataPacket))
  {

    DataPacket packet;

    int len = udp.read((uint8_t *)&packet, sizeof(packet));

    if (len != sizeof(packet))
    {
      Serial.println("Error leyendo paquete");
      return;
    }

    int vx = packet.joyX;
    int vy = packet.joyY;
    int pot = packet.pot;

    Serial.printf("VX:%d VY:%d POT:%d\n", vx, vy, pot);

    // Enviar al Mega
    MegaSerial.printf("<%d,%d,%d>\n", vx, vy, pot);

    lastPacketTime = millis();
  }

  // Seguridad por timeout
  if (millis() - lastPacketTime > TIMEOUT_MS)
  {
    MegaSerial.println("<0,0,0>");
    lastPacketTime = millis();
  }
}
