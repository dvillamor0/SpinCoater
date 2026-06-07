#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <EEPROM.h>

/*
  Arduino Nano - Control básico de RPM para ESC + BLDC

  Conexiones:
    D2  -> Tacómetro IR (interrupción)
    D3  -> BTN1 (ON/OFF)
    D4  -> BTN2 (pulsación larga = recalibrar)
    D9  -> Señal ESC
    A0  -> Potenciómetro
    A4  -> SDA LCD I2C
    A5  -> SCL LCD I2C

  Flujo:
    1) Si existe calibración válida, se usa.
    2) Si no existe, o BTN2 largo la pide, se calibra 30 s sin carga.
    3) En operación normal:
         comando = feedforward(tabla) + PI + sesgo lento
    4) El usuario solo ve RPM objetivo, RPM real y ON/OFF.
*/

// =========================
// Configuración general
// =========================

static const uint8_t PIN_TACO = 2;
static const uint8_t PIN_BTN1 = 3;
static const uint8_t PIN_BTN2 = 4;
static const uint8_t PIN_ESC   = 9;
static const uint8_t PIN_POT   = A0;

static const uint8_t LCD_ADDR  = 0x27;

// Pulsos por revolución del tacómetro IR.
// Ajusta este valor según tu disco/marca.
static const uint8_t TACHO_PPR = 1;

// ESC.
static const uint16_t ESC_MIN_US   = 1000;
static const uint16_t ESC_START_US = 1000;
static const uint16_t ESC_MAX_US   = 2000;
static const uint16_t ESC_ARM_MS   = 3000;

// Tiempos de lazo.
static const uint16_t CONTROL_MS = 100;
static const uint16_t LCD_MS     = 250;

// Auto-calibración.
static const uint8_t  CAL_POINTS     = 12;
static const uint32_t CAL_TOTAL_MS   = 300000UL;
static const uint32_t CAL_STEP_MS    = CAL_TOTAL_MS / CAL_POINTS;
static const uint32_t CAL_SETTLE_MS  = 1500UL;
static const uint16_t CAL_START_US   = 1000;
uint16_t CAL_END_US     = 2000;

// Filtros.
static const float RPM_ALPHA = 0.25f;
static const float POT_ALPHA  = 0.15f;

// Límites de adaptación.
static const float INTEGRAL_LIMIT_RPMS = 10000.0f;
static const float BIAS_LIMIT_US       = 250.0f;

// Cuantización de la consigna.
static const uint16_t TARGET_QUANTUM_RPM = 20;
static const uint16_t MIN_ACTIVE_RPM     = 50;
static const uint16_t HYST_DEADBAND_RPM  = 25;

// Timeout de tacómetro.
static const uint32_t TACHO_TIMEOUT_US = 1000000UL;

// EEPROM.
static const uint32_t EEPROM_MAGIC = 0x52504D31UL; // "RPM1"
static const uint8_t  EEPROM_VER   = 1;

// =========================
// Tipos
// =========================

enum State : uint8_t
{
  ST_IDLE = 0,
  ST_ARMING,
  ST_CALIBRATING,
  ST_RUNNING,
  ST_FAULT
};

enum Fault : uint8_t
{
  FAULT_NONE = 0,
  FAULT_TACHO,
  FAULT_CAL,
  FAULT_STOP
};

struct CalibrationData
{
  uint32_t magic;
  uint8_t version;
  uint8_t count;
  uint16_t minUs;
  uint16_t maxUs;
  uint16_t escUs[CAL_POINTS];
  uint16_t rpm[CAL_POINTS];
};

struct Button
{
  uint8_t pin;
  bool stablePressed;
  bool lastRawPressed;
  unsigned long lastDebounceMs;
  unsigned long pressStartMs;
  bool shortEvent;
  bool longEvent;
  bool longHandled;
};

// =========================
// Objetos globales
// =========================

LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
Servo esc;
CalibrationData cal;
Button btn1, btn2;

volatile unsigned long g_lastEdgeUs = 0;
volatile unsigned long g_lastPeriodUs = 0;

State g_state = ST_IDLE;
Fault g_fault = FAULT_NONE;

bool g_needCalibration = true;

unsigned long g_stateMs = 0;
unsigned long g_lastControlMs = 0;
unsigned long g_lastLcdMs = 0;
unsigned long g_lastDebugMs = 0;

// Medidas.
float g_rpmRaw = 0.0f;
float g_rpmFilt = 0.0f;

// Potenciómetro filtrado.
float g_potFilt = 0.0f;

// Control.
float g_integral = 0.0f;
float g_biasUs = 0.0f;
float g_kp = 0.02f;
float g_ki = 0.0016f;
float g_biasLearn = 0.0006f;
uint16_t g_targetMaxRpm = 10000;
uint16_t g_targetRpm = 0;
uint16_t g_prevTargetRpm = 0;
uint16_t g_escUs = ESC_MIN_US;

// Calibración.
uint8_t g_calIndex = 0;
unsigned long g_calPointMs = 0;
uint32_t g_calSum = 0;
uint16_t g_calCount = 0;

// =========================
// Utilidades de texto
// =========================

/**
 * @brief Devuelve el nombre textual de un estado para depuración por Serial.
 */
const __FlashStringHelper *stateName(State s)
{
  switch (s)
  {
    case ST_IDLE:        return F("IDLE");
    case ST_ARMING:      return F("ARMING");
    case ST_CALIBRATING: return F("CALIBRATING");
    case ST_RUNNING:     return F("RUNNING");
    case ST_FAULT:       return F("FAULT");
    default:             return F("?");
  }
}

/**
 * @brief Devuelve el nombre textual de una falla para depuración por Serial.
 */
const __FlashStringHelper *faultName(Fault f)
{
  switch (f)
  {
    case FAULT_NONE:  return F("NONE");
    case FAULT_TACHO: return F("TACHO");
    case FAULT_CAL:   return F("CAL");
    case FAULT_STOP:  return F("STOP");
    default:          return F("?");
  }
}

/**
 * @brief Imprime una línea del LCD y rellena con espacios hasta 16 caracteres.
 */
void lcdLine(uint8_t row, const char *txt)
{
  lcd.setCursor(0, row);
  uint8_t i = 0;
  for (; i < 16 && txt[i] != '\0'; ++i) lcd.print(txt[i]);
  for (; i < 16; ++i) lcd.print(' ');
}

// =========================
// Botones
// =========================

/**
 * @brief Inicializa una estructura de botón con antirrebote.
 */
void initButton(Button &b, uint8_t pin)
{
  b.pin = pin;
  b.stablePressed = false;
  b.lastRawPressed = false;
  b.lastDebounceMs = 0;
  b.pressStartMs = 0;
  b.shortEvent = false;
  b.longEvent = false;
  b.longHandled = false;
}

/**
 * @brief Actualiza el estado de un botón usando antirrebote por tiempo.
 */
void updateButton(Button &b, unsigned long nowMs)
{
  const bool rawPressed = (digitalRead(b.pin) == LOW);

  if (rawPressed != b.lastRawPressed)
  {
    b.lastRawPressed = rawPressed;
    b.lastDebounceMs = nowMs;
  }

  if ((nowMs - b.lastDebounceMs) >= 35UL && rawPressed != b.stablePressed)
  {
    b.stablePressed = rawPressed;

    if (b.stablePressed)
    {
      b.pressStartMs = nowMs;
      b.longHandled = false;
    }
    else if (!b.longHandled)
    {
      b.shortEvent = true;
    }
  }

  if (b.stablePressed && !b.longHandled && (nowMs - b.pressStartMs) >= 2000UL)
  {
    b.longEvent = true;
    b.longHandled = true;
  }
}

/**
 * @brief Consume el evento de pulsación corta del botón.
 */
bool takeShort(Button &b)
{
  if (!b.shortEvent) return false;
  b.shortEvent = false;
  return true;
}

/**
 * @brief Consume el evento de pulsación larga del botón.
 */
bool takeLong(Button &b)
{
  if (!b.longEvent) return false;
  b.longEvent = false;
  return true;
}

// =========================
// Tacómetro
// =========================

/**
 * @brief ISR del tacómetro IR.
 *
 * Captura el período entre pulsos y rechaza pulsos demasiado cortos.
 */
void isrTacho()
{
  const unsigned long nowUs = micros();
  const unsigned long periodUs = nowUs - g_lastEdgeUs;

  if (periodUs < 250UL) return;

  g_lastEdgeUs = nowUs;
  g_lastPeriodUs = periodUs;
}

/**
 * @brief Convierte período entre pulsos a RPM.
 */
float periodToRpm(unsigned long periodUs)
{
  if (periodUs == 0) return 0.0f;
  return 60000000.0f / ((float)TACHO_PPR * (float)periodUs);
}

/**
 * @brief Actualiza RPM cruda y filtrada.
 */
void sampleRpm(unsigned long nowUs)
{
  unsigned long edgeUs;
  unsigned long periodUs;

  noInterrupts();
  edgeUs = g_lastEdgeUs;
  periodUs = g_lastPeriodUs;
  interrupts();

  float raw = 0.0f;
  if (periodUs > 0 && (nowUs - edgeUs) <= TACHO_TIMEOUT_US)
  {
    raw = periodToRpm(periodUs);
  }

  g_rpmRaw = raw;

  if (raw <= 0.0f)
  {
    g_rpmFilt *= 0.85f;
    if (g_rpmFilt < 1.0f) g_rpmFilt = 0.0f;
  }
  else if (g_rpmFilt <= 0.0f)
  {
    g_rpmFilt = raw;
  }
  else
  {
    g_rpmFilt = RPM_ALPHA * raw + (1.0f - RPM_ALPHA) * g_rpmFilt;
  }
}

// =========================
// EEPROM / calibración
// =========================

/**
 * @brief Verifica si la tabla calibrada tiene coherencia mínima.
 */
bool calValid()
{
  if (cal.magic != EEPROM_MAGIC) return false;
  if (cal.version != EEPROM_VER) return false;
  if (cal.count < 2 || cal.count > CAL_POINTS) return false;
  if (cal.escUs[0] < ESC_MIN_US || cal.escUs[cal.count - 1] > ESC_MAX_US) return false;

  for (uint8_t i = 1; i < cal.count; ++i)
  {
    if (cal.escUs[i] < cal.escUs[i - 1]) return false;
    if (cal.rpm[i] < cal.rpm[i - 1]) return false;
  }

  return (cal.rpm[cal.count - 1] > cal.rpm[0]) && (cal.rpm[cal.count - 1] >= 500);
}

/**
 * @brief Carga calibración desde EEPROM.
 */
bool loadCal()
{
  EEPROM.get(0, cal);
  return calValid();
}

/**
 * @brief Guarda calibración en EEPROM.
 */
void saveCal()
{
  cal.magic = EEPROM_MAGIC;
  cal.version = EEPROM_VER;
  cal.count = CAL_POINTS;
  cal.minUs = CAL_START_US;
  cal.maxUs = CAL_END_US;
  EEPROM.put(0, cal);
}

/**
 * @brief Inicializa ganancias suaves a partir de la tabla calibrada.
 */
void tuneFromCal()
{
  const uint16_t rpmSpan = (cal.rpm[CAL_POINTS - 1] > cal.rpm[0]) ? (cal.rpm[CAL_POINTS - 1] - cal.rpm[0]) : 0;
  const uint16_t usSpan  = (cal.escUs[CAL_POINTS - 1] > cal.escUs[0]) ? (cal.escUs[CAL_POINTS - 1] - cal.escUs[0]) : 0;

  float usPerRpm = 0.02f;
  if (rpmSpan > 0 && usSpan > 0)
  {
    usPerRpm = (float)usSpan / (float)rpmSpan;
  }

  if (usPerRpm < 0.005f) usPerRpm = 0.005f;
  if (usPerRpm > 0.080f) usPerRpm = 0.080f;

  g_kp = usPerRpm * 1.00f;
  g_ki = usPerRpm * 0.08f;
  g_biasLearn = usPerRpm * 0.03f;

  g_targetMaxRpm = cal.rpm[CAL_POINTS - 1];
  if (g_targetMaxRpm < 1000) g_targetMaxRpm = 10000;
}

/**
 * @brief Devuelve el microsegundo de feedforward interpolado para una RPM objetivo.
 */
uint16_t lookupUs(uint16_t rpm)
{
  if (cal.count < 2)
  {
    uint16_t fallback = (uint16_t)(ESC_START_US + (float)rpm * 0.08f);
    if (fallback < ESC_MIN_US) fallback = ESC_MIN_US;
    if (fallback > ESC_MAX_US) fallback = ESC_MAX_US;
    return fallback;
  }

  if (rpm <= cal.rpm[0]) return cal.escUs[0];
  if (rpm >= cal.rpm[cal.count - 1]) return cal.escUs[cal.count - 1];

  for (uint8_t i = 1; i < cal.count; ++i)
  {
    if (rpm <= cal.rpm[i])
    {
      const uint16_t rpmLo = cal.rpm[i - 1];
      const uint16_t rpmHi = cal.rpm[i];
      const uint16_t usLo  = cal.escUs[i - 1];
      const uint16_t usHi  = cal.escUs[i];

      if (rpmHi <= rpmLo) return usHi;

      const float t = (float)(rpm - rpmLo) / (float)(rpmHi - rpmLo);
      return (uint16_t)((float)usLo + t * (float)(usHi - usLo) + 0.5f);
    }
  }

  return cal.escUs[cal.count - 1];
}

/**
 * @brief Convierte el potenciómetro en consigna de RPM y aplica cuantización.
 */
uint16_t readTargetRpm()
{
  const int rawPot = analogRead(PIN_POT);

  if (g_potFilt <= 0.0f)
  {
    g_potFilt = (float)rawPot;
  }
  else
  {
    g_potFilt = POT_ALPHA * (float)rawPot + (1.0f - POT_ALPHA) * g_potFilt;
  }

  uint16_t rpm = (uint16_t)((g_potFilt * (float)g_targetMaxRpm) / 1023.0f + 0.5f);
  rpm = (uint16_t)((rpm / TARGET_QUANTUM_RPM) * TARGET_QUANTUM_RPM);

  if (rpm < MIN_ACTIVE_RPM) rpm = 0;
  return rpm;
}

// =========================
// Estado y control
// =========================

/**
 * @brief Fuerza la salida mínima del ESC.
 */
void escMin()
{
  g_escUs = ESC_MIN_US;
  esc.writeMicroseconds(ESC_MIN_US);
}

/**
 * @brief Cambia de estado y deja trazabilidad por Serial.
 */
void setState(State next)
{
  if (next == g_state) return;

  Serial.print(F("[STATE] "));
  Serial.print(stateName(g_state));
  Serial.print(F(" -> "));
  Serial.println(stateName(next));

  g_state = next;
  g_stateMs = millis();
}

/**
 * @brief Entra en falla y corta el motor.
 */
void setFault(Fault f)
{
  g_fault = f;
  Serial.print(F("[FAULT] "));
  Serial.println(faultName(f));
  escMin();
  setState(ST_FAULT);
}

/**
 * @brief Limpia memoria del controlador PI y sesgo adaptativo.
 */
void resetControlMemory()
{
  g_integral = 0.0f;
  g_biasUs = 0.0f;
  g_prevTargetRpm = 0;
}

/**
 * @brief Calcula el punto actual de la auto-calibración.
 */
uint16_t calibrationUs(uint8_t idx)
{
  const float span = (float)(CAL_END_US - CAL_START_US);
  const float step = (CAL_POINTS > 1) ? span * (float)idx / (float)(CAL_POINTS - 1) : 0.0f;
  return (uint16_t)((float)CAL_START_US + step + 0.5f);
}

/**
 * @brief Inicia la auto-calibración.
 */
void startCalibration()
{
  Serial.println(F("[CAL] start"));
  g_calIndex = 0;
  g_calPointMs = millis();
  g_calSum = 0;
  g_calCount = 0;
  g_potFilt = 0.0f;
  g_rpmRaw = 0.0f;
  g_rpmFilt = 0.0f;
  resetControlMemory();

  for (uint8_t i = 0; i < CAL_POINTS; ++i)
  {
    cal.escUs[i] = 0;
    cal.rpm[i] = 0;
  }

  setState(ST_CALIBRATING);
}

/**
 * @brief Cierra la auto-calibración, guarda resultados y ajusta ganancias.
 */
void finishCalibration()
{
  for (uint8_t i = 1; i < CAL_POINTS; ++i)
  {
    if (cal.rpm[i] < cal.rpm[i - 1]) cal.rpm[i] = cal.rpm[i - 1];
  }

  cal.magic = EEPROM_MAGIC;
  cal.version = EEPROM_VER;
  cal.count = CAL_POINTS;
  cal.minUs = CAL_START_US;
  cal.maxUs = CAL_END_US;

  if (!calValid())
  {
    setFault(FAULT_CAL);
    return;
  }

  saveCal();
  tuneFromCal();

  Serial.println(F("[CAL] saved"));
  g_needCalibration = false;
  setState(ST_RUNNING);
}

/**
 * @brief Ejecuta un paso de auto-calibración.
 */
void processCalibration()
{
  const unsigned long nowMs = millis();

  if (g_calIndex >= CAL_POINTS)
  {
    finishCalibration();
    return;
  }

  const uint16_t us = calibrationUs(g_calIndex);
  esc.writeMicroseconds(us);

  const unsigned long elapsed = nowMs - g_calPointMs;

  if (elapsed >= CAL_SETTLE_MS)
  {
    g_calSum += (uint32_t)(g_rpmFilt + 0.5f);
    g_calCount++;
  }

  if (elapsed >= CAL_STEP_MS)
  {
    const uint16_t avg = (g_calCount > 0) ? (uint16_t)(g_calSum / g_calCount) : 0;

    cal.escUs[g_calIndex] = us;
    cal.rpm[g_calIndex] = avg;

    Serial.print(F("[CAL] point "));
    Serial.print(g_calIndex);
    Serial.print(F(" us="));
    Serial.print(us);
    Serial.print(F(" rpm="));
    Serial.println(avg);

    if (g_calIndex > 0)
    {
      if (avg < (cal.rpm[g_calIndex - 1] * 0.80f))
      {
        Serial.println(F("[CAL] RPM dropped"));
        Serial.println(F("[CAL] Reducing max PWM and restarting"));

        // Último punto estable
        uint16_t safeUs = cal.escUs[g_calIndex - 1];

        // Reducir límite superior
        CAL_END_US = safeUs;

        Serial.print(F("[CAL] New max PWM = "));
        Serial.println(CAL_END_US);

        // Reiniciar calibración
        g_calIndex = 0;
        g_calPointMs = millis();
        g_calSum = 0;
        g_calCount = 0;

        memset(&cal, 0, sizeof(cal));

        return;
      }
    }

    g_calIndex++;
    g_calPointMs = nowMs;
    g_calSum = 0;
    g_calCount = 0;
  }
}

/**
 * @brief Ejecuta el control normal de velocidad.
 */
void processRunning()
{
  g_targetRpm = readTargetRpm();

  if (g_targetRpm == 0)
  {
    resetControlMemory();
    escMin();
    return;
  }

  const float dt = (float)CONTROL_MS / 1000.0f;

  float ffUs = (float)lookupUs(g_targetRpm);

  // Histéresis simple: compensa si el setpoint viene subiendo o bajando.
  if (g_targetRpm > (g_prevTargetRpm + HYST_DEADBAND_RPM))
  {
    ffUs += 6.0f;
  }
  else if (g_prevTargetRpm > (g_targetRpm + HYST_DEADBAND_RPM))
  {
    ffUs -= 6.0f;
  }
  g_prevTargetRpm = g_targetRpm;

  const float error = (float)g_targetRpm - g_rpmFilt;

  g_integral += error * dt;
  if (g_integral > INTEGRAL_LIMIT_RPMS) g_integral = INTEGRAL_LIMIT_RPMS;
  if (g_integral < -INTEGRAL_LIMIT_RPMS) g_integral = -INTEGRAL_LIMIT_RPMS;

  // Sesgo lento para corregir carga persistente sin perseguir ruido.
  if (g_rpmFilt > 0.0f)
  {
    if (error > -500.0f && error < 500.0f)
    {
      g_biasUs += g_biasLearn * error * dt;
      if (g_biasUs > BIAS_LIMIT_US) g_biasUs = BIAS_LIMIT_US;
      if (g_biasUs < -BIAS_LIMIT_US) g_biasUs = -BIAS_LIMIT_US;
    }
  }

  float outUs = ffUs + (g_kp * error) + (g_ki * g_integral) + g_biasUs;

  if (outUs < (float)ESC_START_US) outUs = (float)ESC_START_US;
  if (outUs > (float)ESC_MAX_US) outUs = (float)ESC_MAX_US;

  g_escUs = (uint16_t)(outUs + 0.5f);
  esc.writeMicroseconds(g_escUs);

  unsigned long edgeUs;
  noInterrupts();
  edgeUs = g_lastEdgeUs;
  interrupts();

  if ((g_state == ST_RUNNING) && g_targetRpm > 200 && (micros() - edgeUs) > TACHO_TIMEOUT_US && g_rpmFilt < 20.0f)
  {
      setFault(FAULT_TACHO);
      return;
  }
}

/**
 * @brief Procesa los eventos de usuario y las transiciones de estado.
 */
void handleButtons()
{
  const unsigned long nowMs = millis();

  if (takeShort(btn1))
  {
    if (g_state == ST_IDLE)
    {
      setState(ST_ARMING);
      Serial.println(F("[USER] motor on"));
      g_lastEdgeUs = micros();
      g_lastPeriodUs = 0;
      g_rpmRaw = 0;
      g_rpmFilt = 0;
      escMin();
    }
    else if (g_state == ST_RUNNING || g_state == ST_CALIBRATING || g_state == ST_ARMING)
    {
      Serial.println(F("[USER] motor off"));
      g_needCalibration = g_needCalibration;
      escMin();
      setState(ST_IDLE);
    }
    
  }

  if (takeLong(btn1) && g_state == ST_FAULT)
  {
    Serial.println(F("[USER] clear fault"));
    g_fault = FAULT_NONE;
    setState(ST_IDLE);
  }

  if (takeShort(btn2) && (g_state == ST_RUNNING || g_state == ST_CALIBRATING || g_state == ST_ARMING))
  {
    escMin();
    Serial.println(F("[USER] Stop"));
    setFault(FAULT_STOP);
    setState(ST_FAULT);
  }
  
  if (takeLong(btn2))
  {
    Serial.println(F("[USER] recalibration requested"));
    g_needCalibration = true;
    startCalibration();
  }

  if (g_state == ST_ARMING && (nowMs - g_stateMs) >= ESC_ARM_MS)
  {
    if (g_needCalibration)
    {
      startCalibration();
    }
    else
    {
      g_lastEdgeUs = micros();
      g_lastPeriodUs = 0;
      g_rpmRaw = 0.0f;
      g_rpmFilt = 0.0f;
      setState(ST_RUNNING);
    }
  }
}

// =========================
// LCD / debug
// =========================

/**
 * @brief Muestra el estado actual en la LCD.
 */
void updateLcd()
{
  char l1[17];
  char l2[17];

  if (g_state == ST_IDLE)
  {
    snprintf(l1, sizeof(l1), "RPM: %u/%u", (uint16_t)g_rpmFilt, g_targetRpm);
    snprintf(l2, sizeof(l2), "Motor OFF");
  }
  else if (g_state == ST_ARMING)
  {
    const unsigned long left = (g_stateMs + ESC_ARM_MS > millis()) ? (g_stateMs + ESC_ARM_MS - millis()) : 0UL;
    snprintf(l1, sizeof(l1), "ARMING");
    snprintf(l2, sizeof(l2), "WAIT %lus", left / 1000UL);
  }
  else if (g_state == ST_CALIBRATING)
  {
    const uint16_t us = calibrationUs((g_calIndex < CAL_POINTS) ? g_calIndex : (CAL_POINTS - 1));
    const uint8_t pct = (uint8_t)((100UL * (unsigned long)g_calIndex) / (unsigned long)CAL_POINTS);
    snprintf(l1, sizeof(l1), "CAL %u/%u %u%%", g_calIndex + 1, CAL_POINTS, pct);
    snprintf(l2, sizeof(l2), "%uus %urpm", us, (uint16_t)g_rpmFilt);
  }
  else if (g_state == ST_RUNNING)
  {
    const int ff = (int)lookupUs(g_targetRpm);
    const int b  = (int)(g_biasUs < 0.0f ? -g_biasUs : g_biasUs);
    const char s = (g_biasUs >= 0.0f) ? '+' : '-';
    snprintf(l1, sizeof(l1), "RMP: %u/%u", (uint16_t)g_rpmFilt, g_targetRpm);
    snprintf(l2, sizeof(l2), "Bias%c%d", s, b);
  }
  else
  {
    if (g_fault == FAULT_TACHO)
    {
      snprintf(l1, sizeof(l1), "FAULT TACHO");
      snprintf(l2, sizeof(l2), "OK to clear");
    }
    else
    {
      snprintf(l1, sizeof(l1), "FAULT CAL");
      snprintf(l2, sizeof(l2), "OK to clear");
    }
  }

  lcdLine(0, l1);
  lcdLine(1, l2);
}

/**
 * @brief Emite trazas de depuración por Serial sin saturar la línea.
 */
void debugTick()
{
  if (millis() - g_lastDebugMs < 1000UL) return;
  g_lastDebugMs = millis();

  Serial.print(F("[DBG] st="));
  Serial.print(stateName(g_state));
  Serial.print(F(" tgt="));
  Serial.print(g_targetRpm);
  Serial.print(F(" rpm="));
  Serial.print((uint16_t)g_rpmFilt);
  Serial.print(F(" us="));
  Serial.print(g_escUs);
  Serial.print(F(" bias="));
  Serial.println(g_biasUs, 2);
}

// =========================
// Setup / loop
// =========================

/**
 * @brief Configuración inicial.
 */
void setup()
{
  Serial.begin(9600);
  Serial.println(F("\n--- RPM Controller boot ---"));

  pinMode(PIN_TACO, INPUT_PULLUP);
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);

  initButton(btn1, PIN_BTN1);
  initButton(btn2, PIN_BTN2);

  lcd.init();
  lcd.backlight();
  lcdLine(0, "BOOT...");
  lcdLine(1, "ARM ESC");

  esc.attach(PIN_ESC, ESC_MIN_US, ESC_MAX_US);
  escMin();

  delay(ESC_ARM_MS);

  g_lastEdgeUs = micros();
  g_lastPeriodUs = 0;

  attachInterrupt(digitalPinToInterrupt(PIN_TACO), isrTacho, FALLING);

  if (loadCal())
  {
    tuneFromCal();
    g_needCalibration = false;
    Serial.println(F("[EEPROM] calibration loaded"));
  }
  else
  {
    g_needCalibration = true;
    Serial.println(F("[EEPROM] no valid calibration"));
  }

  g_state = ST_IDLE;
  g_stateMs = millis();
  g_lastControlMs = g_stateMs;
  g_lastLcdMs = g_stateMs;
  g_lastDebugMs = g_stateMs;

  Serial.println(F("[READY] BTN1=ON/OFF, BTN2 long=RECAL"));
}

/**
 * @brief Bucle principal.
 */
void loop()
{
  const unsigned long nowMs = millis();
  const unsigned long nowUs = micros();

  updateButton(btn1, nowMs);
  updateButton(btn2, nowMs);

  handleButtons();

  if ((nowMs - g_lastControlMs) >= CONTROL_MS)
  {
    g_lastControlMs = nowMs;

    sampleRpm(nowUs);

    if (g_state == ST_CALIBRATING)
    {
      processCalibration();
    }
    else if (g_state == ST_RUNNING)
    {
      processRunning();
    }
    else if (g_state == ST_IDLE || g_state == ST_FAULT)
    {
      escMin();
    }
  }

  if ((nowMs - g_lastLcdMs) >= LCD_MS)
  {
    g_lastLcdMs = nowMs;
    updateLcd();
  }

  debugTick();
}