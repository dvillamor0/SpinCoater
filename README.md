# Controlador de RPM para ESC + BLDC (Arduino Nano)

## Descripción

Este proyecto implementa un controlador de velocidad (RPM) para un motor BLDC controlado mediante ESC utilizando un Arduino Nano.

El sistema está diseñado para que el usuario únicamente tenga que:

* Encender/apagar el motor.
* Seleccionar la velocidad mediante un potenciómetro.
* Ver RPM objetivo y RPM reales.
* Recalibrar cuando sea necesario.

Internamente el sistema:

* Aprende automáticamente la curva ESC → RPM.
* Guarda la calibración en EEPROM.
* Usa una tabla de feedforward interpolada.
* Corrige variaciones de carga mediante un controlador PI.
* Compensa histéresis.
* Aprende lentamente una corrección adaptativa para cargas permanentes.
* Detecta pérdida del tacómetro.

---

# Hardware

## Arduino Nano

### Entradas

| Pin | Función                 |
| --- | ----------------------- |
| D2  | Tacómetro IR            |
| D3  | Botón ON/OFF            |
| D4  | Botón STOP / Recalibrar |
| A0  | Potenciómetro           |

### Salidas

| Pin | Función |
| --- | ------- |
| D9  | ESC     |

### LCD I2C

| Pin LCD | Arduino |
| ------- | ------- |
| SDA     | A4      |
| SCL     | A5      |
| VCC     | 5V      |
| GND     | GND     |

---

# Botones

Ambos botones usan:

```cpp
INPUT_PULLUP
```

Por lo tanto:

```text
Pin ----- Pulsador ----- GND
```

Estado:

```text
Suelto   = HIGH
Presionado = LOW
```

No requieren resistencias externas.

---

# Máquina de estados

```text
             BTN1

         +--------+
         | IDLE   |
         +--------+
              |
              v
         +--------+
         |ARMING  |
         +--------+
              |
              |
              v
      +---------------+
      |CALIBRATING    |
      +---------------+
              |
              v
         +--------+
         |RUNNING |
         +--------+

 BTN2 corto -> STOP

              |
              v

         +--------+
         |FAULT   |
         +--------+

 BTN1 largo -> CLEAR
```

---

# Estados

## ST_IDLE

Motor apagado.

ESC:

```cpp
1000 us
```

LCD:

```text
RPM: xxx/yyy
Motor OFF
```

---

## ST_ARMING

Tiempo de armado del ESC.

Duración:

```cpp
ESC_ARM_MS = 3000 ms
```

LCD:

```text
ARMING
WAIT 3s
```

---

## ST_CALIBRATING

Se genera automáticamente una tabla:

```text
PWM → RPM
```

Durante la calibración:

```text
1000 us
1090 us
1181 us
...
2000 us
```

Cada punto se mantiene varios segundos.

---

## ST_RUNNING

Control normal.

LCD:

```text
RPM: 4500/5000
Bias+12
```

---

## ST_FAULT

Motor detenido.

Ejemplos:

```text
FAULT TACHO
FAULT CAL
FAULT STOP
```

---

# Auto-Calibración

## Objetivo

Medir:

```text
PWM -> RPM
```

del motor real.

---

## Duración

Actualmente:

```cpp
CAL_TOTAL_MS = 300000
```

equivale a:

```text
5 minutos
```

---

## Número de puntos

```cpp
CAL_POINTS = 12
```

Ejemplo:

| Punto | PWM  |
| ----- | ---- |
| 0     | 1000 |
| 1     | 1090 |
| 2     | 1181 |
| 3     | 1272 |
| ...   | ...  |
| 11    | 2000 |

---

# Detección automática de límite superior

Tu motor pierde sincronismo, cuando detecta una caída:

```text
[CAL] RPM dropped
```

el sistema:

1. Guarda el último punto estable.
2. Reduce CAL_END_US.
3. Reinicia calibración.
4. Aprende un nuevo máximo seguro.

Ejemplo:

```text
2000 us -> falla

nuevo máximo:

1827 us
```

---

# Feedforward

Tras calibrar se obtiene:

```text
RPM
 ^
 |
 |      *
 |    *
 |  *
 |*
 +-------------> PWM
```

La tabla queda almacenada en EEPROM.

---

## Interpolación

Para una RPM objetivo:

```cpp
lookupUs()
```

calcula:

$
u_{ff} = u_1 + \frac{rpm-rpm_1} {rpm_2-rpm_1} (u_2-u_1)
$

---

# Control PI

## Error

$
e(k) = RPM_{objetivo}-RPM_{real}
$

---

## Integral

$
I(k) = I(k-1)+e(k)\Delta t
$

---

## Salida PI

$
u_{PI} =K_p e(k)+K_i I(k)
$

---

# Histéresis

Compensa diferencias entre:

```text
Subiendo RPM
```

y

```text
Bajando RPM
```

---

Al aumentar:

```cpp
ffUs += 6
```

Al disminuir:

```cpp
ffUs -= 6
```

---

# Adaptación de carga

El PI corrige rápido.

Además existe:

```cpp
g_biasUs
```

que aprende lentamente.

---

## Actualización

$
bias(k) =bias(k-1)+\eta e(k)\Delta t
$

donde:

$
\eta = g_{biasLearn}
$

---

Ejemplo:

### Sin carga

```text
5000 RPM
1500 us
```

### Con carga

```text
5000 RPM
1550 us
```

Tras unos segundos:

```text
bias = +50 us
```

El controlador aprende la carga.

---

# Medición de RPM

La ISR mide el tiempo entre pulsos.

```cpp
attachInterrupt(
    digitalPinToInterrupt(PIN_TACO),
    isrTacho,
    FALLING
);
```

---

## Período

$
T = t_n - t_{n-1}
$

---

## RPM

$
RPM =
\frac{60,000,000}
{PPR\cdot T}
$

---

donde:

| Variable | Significado       |
| -------- | ----------------- |
| PPR      | Pulsos por vuelta |
| T        | microsegundos     |

---

# Filtro RPM

Filtro exponencial:

$
RPM_f(k)=
\alpha RPM(k)
+
(1-\alpha)RPM_f(k-1)
$

Con:

```cpp
RPM_ALPHA = 0.25
```

---

# Potenciómetro

También filtrado:

$
P_f(k)=
\alpha P(k)
+
(1-\alpha)P_f(k-1)
$

---

# Cuantización

Para evitar vibración:

```cpp
TARGET_QUANTUM_RPM = 20
```

---

Ejemplo:

```text
4321 RPM
```

se convierte en:

```text
4320 RPM
```

---

# EEPROM

La calibración se guarda automáticamente.

Estructura:

```cpp
CalibrationData
```

Contiene:

* PWM mínimo
* PWM máximo
* Tabla PWM
* Tabla RPM
* Versión
* Firma

---

# Protección de tacómetro

Actualmente el código verifica:

```cpp
if(
    g_state == ST_RUNNING &&
    g_targetRpm > 200 &&
    (micros()-edgeUs) > TACHO_TIMEOUT_US &&
    g_rpmFilt < 20
)
```

Si se cumple:

```cpp
setFault(FAULT_TACHO);
```

---

# Controles de usuario

## BTN1

Pulsación corta:

```text
IDLE -> ON
RUNNING -> OFF
```

Pulsación larga:

```text
FAULT -> CLEAR
```

---

## BTN2

Pulsación corta:

```text
STOP EMERGENCIA
```

Genera:

```text
FAULT_STOP
```

---

Pulsación larga:

```text
Recalibración completa
```