// ---- Pinos --------------------------------------------------
#define PIN_ENA     9
#define PIN_IN1     8
#define PIN_IN2     7
#define PIN_ENC_A   3    // INT1
#define PIN_ENC_B   2    // INT0

// ---- Encoder ------------------------------------------------
#define PULSOS_POR_REV   840
#define GRAUS_POR_PULSO  (360.0f / PULSOS_POR_REV)

// Rejeicao de ruido
#define REJEITA_RUIDO_US   150

// ---- Ganhos PID ---------------------------------------------
float Kp = 2.5f;
float Ki = 4.0f;
float Kd = 0.08f;

// ---- Anti-windup ----------------------------------------------
#define INTEGRAL_MAX   200.0f
#define INTEGRAL_MIN  -200.0f

// ---- Saturacao do atuador -----------------------------------
#define PWM_MAX   200
#define PWM_MIN     0

// ---- Tempo de amostragem ------------------------------------
#define TS_MS   10       // 10 ms = 100 Hz

// ---- Zona morta ------------
#define ZONA_MORTA   1.5f   // graus

// ---- Variaveis globais --------------------------------------
volatile long  pulsos        = 0;
volatile int   ultimoA       = 0;
volatile unsigned long ultimaTransicaoUs = 0;

float setpoint          = 0.0f;
float posicao            = 0.0f;
float posicaoAnteriorBruta = 0.0f;
float erro               = 0.0f;
float medicaoAnterior     = 0.0f;
float integral            = 0.0f;
float saidaPID            = 0.0f;

unsigned long tAnterior = 0;

//  INTERRUPCOES DO ENCODER
void ISR_A() {
  unsigned long agoraUs = micros();
  if (agoraUs - ultimaTransicaoUs < REJEITA_RUIDO_US) return;
  ultimaTransicaoUs = agoraUs;

  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  if (a != ultimoA) {
    pulsos += (b != a) ? 1 : -1;
  }
  ultimoA = a;
}

void ISR_B() {
  unsigned long agoraUs = micros();
  if (agoraUs - ultimaTransicaoUs < REJEITA_RUIDO_US) return;
  ultimaTransicaoUs = agoraUs;

  int a = digitalRead(PIN_ENC_A);
  int b = digitalRead(PIN_ENC_B);
  pulsos += (a == b) ? 1 : -1;
}

//  ACIONAR MOTOR
void acionarMotor(int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, pwm);
  } else if (pwm < 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
    analogWrite(PIN_ENA, -pwm);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_ENA, 0);
  }
}

//  FILTRO DE MEDIANA 
float medianaTres(float a, float b, float c) {
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

//  CALCULO PID
float calcularPID(float ref, float med, float dt) {
  erro = ref - med;

  if (fabsf(erro) < ZONA_MORTA) {
    integral = 0.0f;
    return 0.0f;
  }

  float P = Kp * erro;

  integral += erro * dt;
  integral  = constrain(integral, INTEGRAL_MIN, INTEGRAL_MAX);
  float I   = Ki * integral;

  float D = -Kd * (med - medicaoAnterior) / dt;

  medicaoAnterior = med;
  return P + I + D;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), ISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), ISR_B, CHANGE);

  ultimoA = digitalRead(PIN_ENC_A);

  Serial.println("Setpoint,Posicao,Erro,PWM");
  Serial.println("# Firmware v3 - sem potenciometro, sem capacitor externo");
  Serial.println("# Envie um numero (graus) para mover, ou STATUS / DIST / R");

  tAnterior = millis();
}

void loop() {
  unsigned long agora = millis();
  float dt = (agora - tAnterior) / 1000.0f;

  if (dt >= (TS_MS / 1000.0f)) {
    tAnterior = agora;

    // 1. Leitura do encoder (secao critica)
    long pLocal;
    noInterrupts();
    pLocal = pulsos;
    interrupts();

    float posicaoBruta = pLocal * GRAUS_POR_PULSO;

    // 2. Filtro de mediana de 3 amostras (suaviza ruido sem
    //    capacitor externo, sem atrasar a resposta como média)
    posicao = medianaTres(posicaoBruta, posicaoAnteriorBruta, posicao);
    posicaoAnteriorBruta = posicaoBruta;

    // 3. Calcula PID (setpoint vem so da serial agora)
    saidaPID = calcularPID(setpoint, posicao, dt);

    // 4. Aciona motor
    int pwmSaida = (int)constrain(saidaPID, -PWM_MAX, PWM_MAX);
    acionarMotor(pwmSaida);

    // 5. Telemetria serial — CSV 115200 baud
    Serial.print(setpoint, 2);  Serial.print(',');
    Serial.print(posicao,  2);  Serial.print(',');
    Serial.print(erro,     2);  Serial.print(',');
    Serial.println(pwmSaida);
  }

  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "R") {
      noInterrupts(); pulsos = 0; interrupts();
      setpoint              = 0.0f;
      integral              = 0.0f;
      medicaoAnterior       = 0.0f;
      posicaoAnteriorBruta  = 0.0f;
      posicao               = 0.0f;
      Serial.println("# RESET executado");

    } else if (cmd == "DIST") {
      float spOrig = setpoint;
      setpoint = spOrig + 30.0f;
      Serial.println("# DISTURBIO aplicado (+30 graus)");
      delay(500);
      setpoint = spOrig;
      Serial.println("# DISTURBIO removido");

    } else if (cmd == "STATUS") {
      Serial.print("# Kp="); Serial.print(Kp, 3);
      Serial.print("  Ki="); Serial.print(Ki, 3);
      Serial.print("  Kd="); Serial.println(Kd, 4);

    } else if (cmd.length() > 1 && cmd[0] == 'P') {
      Kp = cmd.substring(1).toFloat();
      Serial.print("# Kp atualizado: "); Serial.println(Kp, 3);

    } else if (cmd.length() > 1 && cmd[0] == 'I') {
      Ki       = cmd.substring(1).toFloat();
      integral = 0.0f;
      Serial.print("# Ki atualizado: "); Serial.println(Ki, 3);

    } else if (cmd.length() > 1 && cmd[0] == 'D') {
      Kd = cmd.substring(1).toFloat();
      Serial.print("# Kd atualizado: "); Serial.println(Kd, 4);

    } else {
      float sp = cmd.toFloat();
      if (cmd == "0" || sp != 0.0f) {
        setpoint = sp;
        integral = 0.0f;
        Serial.print("# Setpoint: "); Serial.print(setpoint, 1);
        Serial.println(" graus");
      }
    }
  }
}
