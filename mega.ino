#include <AccelStepper.h> // Si usas la librería
// ===== PINES RAMPS 1.6 / 1.4 =====
#define X_STEP 54  // Motor 1 (Carcasa móvil)
#define X_DIR  55
#define X_EN   38

#define Y_STEP 60  // Motor 2 (Fijo a estructura)
#define Y_DIR  61
#define Y_EN   56

// ===== CONSTANTES DE PASOS (A 1/32 MICROSTEPPING = 6400 p/v) =====
const long LIM_X = 178;          // 10 grados para X
const long LIM_Y_DER = 178;      // 10 grados derecha para Y
const long LIM_Y_IZQ = -1066;    // 60 grados izquierda para Y (Negativo por sentido)

// ===== VARIABLES DE CONTROL Y POSICIÓN =====
int vx = 0;
int vy = 0;
long posX = 0; // Posición actual en pasos
long posY = 0; 

unsigned long lastStepX = 0;
unsigned long lastStepY = 0;
unsigned long lastCmdTime = 0;

const unsigned long WATCHDOG_TIMEOUT = 500; 
const int MAX_SPEED = 200;

void setup() {
  Serial.begin(115200);  
  Serial1.begin(115200); // Comunicación con ESP32

  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_EN, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_EN, OUTPUT);

  digitalWrite(X_EN, LOW); // Habilitar Drivers
  digitalWrite(Y_EN, LOW);

  // Al encender, el brazo debe estar centrado físicamente
  posX = 0;
  posY = 0;

  Serial.println("SISTEMA RAMPS 1.6: Online. Limites cargados.");
}

void loop() {
  // ===== RECEPCIÓN DE COMANDOS ROBUSTA =====
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    int startIdx = input.indexOf('<');
    int endIdx = input.indexOf('>');
    
    if (startIdx != -1 && endIdx != -1) {
      String data = input.substring(startIdx + 1, endIdx);
      int tempX, tempY;
      if (sscanf(data.c_str(), "%d,%d", &tempX, &tempY) == 2) {
        vx = constrain(tempX, -MAX_SPEED, MAX_SPEED);
        vy = constrain(tempY, -MAX_SPEED, MAX_SPEED);
        lastCmdTime = millis();
      }
    }
  }

  // ===== WATCHDOG (SEGURIDAD) =====
  if (millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
    vx = 0; vy = 0;
  }

  // ===== EJECUCIÓN DE MOVIMIENTO CON FILTRO DE LÍMITES =====
  // Para Motor X: Limitar entre -178 y +178
  stepMotor(X_STEP, X_DIR, vx, lastStepX, posX, -LIM_X, LIM_X);
  
  // Para Motor Y: Limitar entre -1066 (Izq) y +178 (Der)
  stepMotor(Y_STEP, Y_DIR, vy, lastStepY, posY, LIM_Y_IZQ, LIM_Y_DER);
}

void stepMotor(int stepPin, int dirPin, int speed, unsigned long &lastStep, long &posContador, long limiteMin, long limiteMax) {
  if (speed == 0) return;

  // Determinar dirección
  bool direccion = (speed > 0);

  // --- VALIDACIÓN DE LÍMITES ---
  // Si intenta ir a la derecha (+) y ya llegó al máximo, o izquierda (-) y llegó al mínimo:
  if (direccion && posContador >= limiteMax) return;
  if (!direccion && posContador <= limiteMin) return;

  digitalWrite(dirPin, direccion);

  unsigned long interval = map(abs(speed), 1, MAX_SPEED, 3000, 300);

  if (micros() - lastStep >= interval) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5); 
    digitalWrite(stepPin, LOW);
    
    // Actualizar contador de posición
    if (direccion) posContador++; else posContador--;
    
    lastStep = micros();
  }
}
