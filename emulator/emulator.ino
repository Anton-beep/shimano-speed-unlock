// =============================================================================
//  Hall-Sensor RPM Emulator  -  Arduino Pro Micro (ATmega32U4, 5V/16MHz)
// =============================================================================
//
//  WHAT THIS IS
//  ------------
//  Firmware that emulates a magnet-on-wheel rotation sensor. The board sits
//  inline where the real sensor would be and makes the external controller
//  ("the controller", the system under test) measure a commanded wheel RPM.
//  Bench-testing / spoofing tool.
//
//  PHYSICAL MODEL  (why pulse width changes with RPM)
//  --------------------------------------------------
//  A magnet rides on the wheel at some radius from the axle. Once per
//  revolution (per magnet) it sweeps past the fixed sensor. The sensor outputs
//  a pulse for as long as the magnet is inside its detection zone. So:
//
//    * PULSE PERIOD  = time between magnet passes = 60 / (RPM * magnets/rev).
//      Faster wheel -> pulses come closer together.
//    * PULSE WIDTH   = how long the magnet stays in the detection zone =
//      (detection-zone length) / (magnet's tangential speed). The magnet's
//      tangential speed is 2*pi*r*RPM/60, so faster wheel / bigger radius ->
//      magnet zips past faster -> SHORTER pulse.
//
//  INPUTS (edit below, re-flash):
//    TARGET_KMH         - GROUND SPEED to emulate (km/h). The thing you set.
//    WHEEL_DIAMETER_MM  - wheel outer diameter; converts km/h -> wheel RPM
//                         (RPM = v / (pi*D) * 60).
//    MAGNET_RADIUS_MM   - axle-center to magnet distance (sets tangential speed)
//    MAGNET_DETECT_MM   - length of the sensor's detection zone along the
//                         magnet's path. THIS sets pulse width. Unknown up
//                         front: calibrate it (see CALIBRATION below).
//    PULSES_PER_REV     - number of magnets on the wheel (usually 1).
//
//  NOTE: the controller counts EDGES, so the displayed speed is set by the
//  pulse RATE (period), which TARGET_KMH + WHEEL_DIAMETER_MM determine. Pulse
//  width (MAGNET_RADIUS_MM / MAGNET_DETECT_MM) only shapes the pulse; it does
//  not change the reading. If the controller's displayed speed is off by a
//  constant factor, trim WHEEL_DIAMETER_MM (or PULSES_PER_REV) to match.
//
//  CALIBRATION of MAGNET_DETECT_MM
//  -------------------------------
//  Capture the REAL sensor at a known RPM (your scope/ tool). Measure its pulse
//  width Tp (seconds) and the period. Then:
//      detect_mm = Tp * (2*pi*MAGNET_RADIUS_MM*RPM/60)
//  Plug that in and the emulator reproduces the real pulse width at any RPM.
//
//  ELECTRICAL MODEL
//  ----------------
//  The real sensor is a dry contact / open-collector switch: idle = line HIGH
//  (controller's pull-up), magnet present = line shorted toward GND. We emulate
//  with ONE pin. Two drive modes (see PUSH_PULL):
//    PUSH_PULL 1 - actively drive HIGH at rest, LOW at pulse. Guaranteed clean
//                  edges even if the controller has no pull-up.
//    PUSH_PULL 0 - open-collector: high-Z at rest (controller pull-up sets
//                  HIGH), drive LOW at pulse. Faithful to the dry contact.
//
//  WIRING
//  ------
//      SIGNAL_PIN (A2) ........ controller's signal sensor line
//      Pro Micro GND .......... controller GND line  (shared ground, required)
//  Flash over USB DISCONNECTED from the controller, then wire and run.
// =============================================================================

// ----------------------------- CONFIGURATION ---------------------------------
#define SIGNAL_PIN         A2      // pin emulating the sensor switch (wired to
                                   // controller's signal pin; GND is shared)

// --- desired output ---
#define TARGET_KMH         20.0    // GROUND SPEED to emulate (km/h) <-- set this

// --- wheel / magnet geometry ---
#define WHEEL_DIAMETER_MM  675.0   // wheel outer diameter (mm); converts km/h->RPM
#define MAGNET_RADIUS_MM   50.0    // axle center -> magnet distance (mm)
#define MAGNET_DETECT_MM   20.0    // sensor detection-zone length (mm); CALIBRATE
#define PULSES_PER_REV     1       // magnets on the wheel

// --- limits / drive ---
#define MAX_RPM            1000.0  // sanity clamp
#define MIN_PULSE_US       50UL    // floor so very fast spins still give a pulse
#define MAX_DUTY           0.90    // pulse never exceeds this fraction of period
#define PUSH_PULL          0       // 1=drive both rails, 0=open-collector
// -----------------------------------------------------------------------------

// Pre-computed phase durations in microseconds (filled in setup()).
static uint32_t g_pulseUs = 0;
static uint32_t g_restUs  = 0;
static bool     g_run     = false;

#if PUSH_PULL
static inline void lineConduct() { digitalWrite(SIGNAL_PIN, LOW);  } // pulse: 0V
static inline void lineOpen()    { digitalWrite(SIGNAL_PIN, HIGH); } // rest:  HIGH
#else
static inline void lineConduct() { pinMode(SIGNAL_PIN, OUTPUT); }    // drive LOW
static inline void lineOpen()    { pinMode(SIGNAL_PIN, INPUT);  }    // high-Z idle
#endif

// DEBUG pulse indicator on the Pro Micro RX LED (pin 17, active-LOW). There is
// NO D13 LED on a Pro Micro, so LED_BUILTIN is useless here.
#define LED_PIN 17
static inline void ledOn()  { digitalWrite(LED_PIN, LOW);  }
static inline void ledOff() { digitalWrite(LED_PIN, HIGH); }

// delayMicroseconds() is only reliable below ~16 ms. Split long waits into a
// millisecond delay() plus a microsecond remainder so any phase length works.
static inline void waitUs(uint32_t us) {
  if (us >= 1000UL) { delay(us / 1000UL); us %= 1000UL; }
  if (us) delayMicroseconds((unsigned int)us);
}

void setup() {
  digitalWrite(SIGNAL_PIN, LOW);
#if PUSH_PULL
  pinMode(SIGNAL_PIN, OUTPUT);   // stays OUTPUT; we toggle the level
#endif
  lineOpen();                    // start at rest (HIGH / high-Z)

  pinMode(LED_PIN, OUTPUT);
  ledOff();

  const float PI_F = 3.14159265f;

  // --- km/h -> wheel RPM ---
  // ground speed v (mm/s) = circumference * rev/s ; circumference = pi*D.
  // 1 km/h = 1e6 mm / 3600 s = 277.778 mm/s.
  if (TARGET_KMH <= 0.0f) {        // stop: hold idle, emit nothing
    g_run = false;
    return;
  }
  float v_ground_mm_s = (float)TARGET_KMH * (1000000.0f / 3600.0f);
  float circ_mm       = PI_F * WHEEL_DIAMETER_MM;
  float rpm           = v_ground_mm_s * 60.0f / circ_mm;
  if (rpm > MAX_RPM) rpm = MAX_RPM;

  // --- pulse period: time between magnet passes ---
  float pulses_per_sec = rpm * (float)PULSES_PER_REV / 60.0f;
  float period_us_f    = 1000000.0f / pulses_per_sec;

  // --- pulse width: detection-zone length / magnet tangential speed ---
  // tangential speed (mm/s) = 2*pi*r*RPM/60
  float v_mag_mm_s = 2.0f * PI_F * MAGNET_RADIUS_MM * rpm / 60.0f;
  float pulse_us_f = (v_mag_mm_s > 0.0f)
                       ? (MAGNET_DETECT_MM / v_mag_mm_s) * 1000000.0f
                       : period_us_f;

  // clamp: keep a rest gap, and never below the hardware pulse floor
  float cap = period_us_f * MAX_DUTY;
  if (pulse_us_f > cap) pulse_us_f = cap;

  uint32_t period_us = (uint32_t)(period_us_f + 0.5f);
  uint32_t pulse_us  = (uint32_t)(pulse_us_f + 0.5f);
  if (pulse_us < MIN_PULSE_US) pulse_us = MIN_PULSE_US;
  if (pulse_us >= period_us)   pulse_us = period_us / 2;
  if (pulse_us == 0)           pulse_us = 1;

  g_pulseUs = pulse_us;
  g_restUs  = period_us - pulse_us;
  g_run     = true;
}

void loop() {
  if (!g_run) return;          // stopped: line idles, no pulses

  lineConduct();  ledOn();     // magnet in detection zone -> pulse
  waitUs(g_pulseUs);
  lineOpen();     ledOff();    // magnet gone -> rest at idle
  waitUs(g_restUs);
}
