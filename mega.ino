// ===== PINES RAMPS =====
#define X_STEP 54
#define X_DIR  55
#define X_EN   38

#define Y_STEP 60
#define Y_DIR  61
#define Y_EN   56

// ===== VARIABLES DE CONTROL =====
int vx = 0;
int vy = 0;
unsigned long lastStepX = 0;
unsigned long lastStepY = 0;
unsigned long lastCmdTime = 0;

const unsigned long WATCHDOG_TIMEOUT = 500; // Aumentado a 500ms por latencia de red
const int MAX_SPEED = 200;

void setup() {
  Serial.begin(115200);  // USB Debugging
  Serial1.begin(115200); // Comunicación con ESP32

  pinMode(X_STEP, OUTPUT); pinMode(X_DIR, OUTPUT); pinMode(X_EN, OUTPUT);
  pinMode(Y_STEP, OUTPUT); pinMode(Y_DIR, OUTPUT); pinMode(Y_EN, OUTPUT);

  // IMPORTANTE: Los drivers se habilitan con LOW
  digitalWrite(X_EN, LOW);
  digitalWrite(Y_EN, LOW);

  Serial.println("MTDS: Brazo Robotico Online. Esperando ESP32...");
}

void loop() {
  // ===== RECEPCIÓN DE COMANDOS ROBUSTA =====
  if (Serial1.available()) {
    String input = Serial1.readStringUntil('\n');
    
    // Buscamos el formato <vx,vy>
    int startIdx = input.indexOf('<');
    int endIdx = input.indexOf('>');
    
    if (startIdx != -1 && endIdx != -1) {
      String data = input.substring(startIdx + 1, endIdx);
      int tempX, tempY;
      if (sscanf(data.c_str(), "%d,%d", &tempX, &tempY) == 2) {
        vx = constrain(tempX, -MAX_SPEED, MAX_SPEED);
        vy = constrain(tempY, -MAX_SPEED, MAX_SPEED);
        lastCmdTime = millis();

        // --- DEBUG LOGS ---
        // Solo imprimimos si realmente cambió algo o para confirmar recepción
        Serial.print("CMD Recibido -> VX: ");
        Serial.print(vx);
        Serial.print(" | VY: ");
        Serial.println(vy);
        // ------------------
      }
    }
  }

  // ===== WATCHDOG (SEGURIDAD) =====
  if (millis() - lastCmdTime > WATCHDOG_TIMEOUT) {
    if (vx != 0 || vy != 0) { // Solo avisar si se estaba moviendo y se detuvo
       Serial.println("WATCHDOG: Señal perdida, deteniendo motores.");
    }
    vx = 0;
    vy = 0;
  }

  // ===== EJECUCIÓN DE MOVIMIENTO =====
  stepMotor(X_STEP, X_DIR, vx, lastStepX);
  stepMotor(Y_STEP, Y_DIR, vy, lastStepY);
}

void stepMotor(int stepPin, int dirPin, int speed, unsigned long &lastStep) {
  if (speed == 0) return;

  digitalWrite(dirPin, speed > 0);

  // Cálculo de intervalo (a menor velocidad, mayor intervalo)
  // 3000us = lento, 300us = rápido
  unsigned long interval = map(abs(speed), 1, MAX_SPEED, 3000, 300);

  if (micros() - lastStep >= interval) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(5); // Pulso ligeramente más ancho para mayor estabilidad
    digitalWrite(stepPin, LOW);
    lastStep = micros();
  }
}
