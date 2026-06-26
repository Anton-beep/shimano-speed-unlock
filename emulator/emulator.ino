// =============================================================================
//  Hall-Sensor RPM Emulator  -  Arduino Pro Micro (ATmega32U4, 5V/16MHz)
// =============================================================================
//
//  WHAT THIS IS
//  ------------
//  Firmware that emulates a two-wire Hall/reed rotation sensor. The board sits
//  inline where the real sensor would be and makes the external controller
//  ("the controller", i.e. the system under test) measure a commanded RPM.
//  This is a bench-testing / spoofing tool.
//
//  ELECTRICAL MODEL  (why we toggle pinMode instead of writing HIGH/LOW)
//  --------------------------------------------------------------------
//  The real sensor is a DRY CONTACT / OPEN-COLLECTOR switch, not a voltage
//  source. Verified by direct measurement (A0 + internal pull-up):
//      * No magnet -> circuit OPEN (high-Z). Controller's pull-up holds line
//        HIGH. This is the idle / resting state (line sits here most of time).
//      * Magnet present -> circuit CONDUCTS, shorting line to GND -> LOW.
//        One brief LOW pulse per magnet pass per revolution.
//  So normal operation = line HIGH at rest with short LOW pulses. The
//  controller counts those LOW edges and converts edge frequency to RPM.
//
//  We reproduce the switch with ONE pin, no transistor/opto/relay, by changing
//  pin MODE (not output voltage):
//      OPEN  (idle, "no magnet")  -> pinMode(INPUT)  : high-Z, NO pull-up.
//                                    Controller's pull-up sets the HIGH level.
//      CONDUCT (pulse, "magnet")  -> pinMode(OUTPUT) : drives LOW (port bit 0).
//  We NEVER digitalWrite(HIGH): driving HIGH would fight the controller's
//  pull-up and is electrically wrong. The PORT bit is forced to 0 once in
//  setup(), so flipping DDR alone gives: INPUT=high-Z idle, OUTPUT=LOW pulse.
//
//  WIRING
//  ------
//      SIGNAL_PIN (D9) ........ controller's POSITIVE sensor line
//      Pro Micro GND .......... controller 0V line  (shared ground, required)
//      battery negative ....... that same shared ground node
//  Power the Pro Micro from a floating battery pack; tie only the negatives
//  together at a single point. The pin pulls the positive line toward the
//  shared ground during a pulse.
//
//  HOW TO CHANGE RPM
//  -----------------
//  Edit TARGET_RPM (and PULSES_PER_REV if needed) below and re-flash. There is
//  no runtime control interface; the board emits the configured RPM at boot.
//
//  UNVERIFIED PRECONDITIONS - the human MUST confirm before wiring hardware
//  -----------------------------------------------------------------------
//   1. BOARD VARIANT vs LINE VOLTAGE. This sketch assumes a 5V/16MHz Pro Micro
//      whose GPIO tolerate the measured below-5V open-circuit line. On a
//      3.3V/8MHz Pro Micro the GPIO are NOT 5V-tolerant (abs max ~3.6V):
//      re-measure the open-circuit voltage, confirm it is <=3.3V, and if it
//      sits between 3.3V and 5V do NOT go pin-direct (use a series resistor +
//      clamp, level shifter, or opto-MOSFET). Confirm which board you have.
//      (On 8MHz the timer math below still works: ticks/us is derived from
//       F_CPU, so timing stays correct - but the voltage issue remains.)
//   2. SHORT-CIRCUIT CURRENT <= ~20 mA (ATmega32U4 per-pin recommendation;
//      abs max 40 mA). The pin sinks this whenever it holds the line LOW. The
//      ~1.5V-under-load reading suggests only a few mA; verify with a meter.
//   3. POLARITY: pin pulls the positive line toward shared ground; Pro Micro
//      GND ties to the controller's 0V.
//   4. SHARED GROUND: Pro Micro GND connected to controller 0V; battery
//      negative to that same node.
//
//  USB GROUND CAVEAT
//  -----------------
//  You still plug into a computer over USB to FLASH. Doing that while the board
//  is wired to the controller joins the computer's ground to the shared-ground
//  node and can create a ground loop / conflict. Workflow: flash first
//  (disconnected from controller), THEN wire to controller and run on battery.
//  This sketch never waits for Serial (would hang forever with no USB host).
//
//  QUICK VALIDATION
//  ----------------
//   * Scope SIGNAL_PIN with the controller (or a temp pull-up to VCC) attached:
//     rests HIGH, dips to 0V during brief LOW pulses. A bare high-Z pin with no
//     pull-up floats and won't read a clean HIGH - that is expected.
//   * With N=1, LOW-pulse frequency must equal TARGET_RPM/60 Hz.
// =============================================================================

// ----------------------------- CONFIGURATION ---------------------------------
// D9 = PB5 = OC1A, the Timer1 output-compare pin. That is fine here: we leave
// TCCR1A = 0 so the COM1A output is DISCONNECTED and D9 stays plain GPIO that we
// drive via pinMode. Timer1 is used only as an interrupt source, not as a PWM
// output, so the pin and the generation timer do not actually collide.
#define SIGNAL_PIN        9        // digital pin emulating the sensor switch (D9)
#define TARGET_RPM        30       // RPM to emit; edit and re-flash to change
#define PULSES_PER_REV    1        // N: LOW pulses per revolution
#define PULSE_WIDTH_US    250000  // nominal LOW-pulse width (microseconds)
#define PULSE_DUTY_CAP    0.50     // pulse width never exceeds this fraction of period
#define MAX_RPM           1000    // sanity clamp
// -----------------------------------------------------------------------------

// Timer1 runs in CTC mode with a /8 prescaler. ticksPerUs is derived from F_CPU
// so the math is correct on both 16MHz (=2) and 8MHz (=1) boards.
static const uint32_t TICKS_PER_US = (F_CPU / 8UL) / 1000000UL;

// State machine phases.
enum { STATE_REST, STATE_PULSE };

// Pre-computed phase durations in Timer1 ticks (written once in setup() before
// interrupts are enabled, then only read by the ISR).
static uint32_t g_pulseTicks = 0;
static uint32_t g_restTicks  = 0;

// ISR working state.
volatile uint8_t  g_state    = STATE_REST;
volatile uint32_t g_remTicks = 0;   // ticks left before the current phase ends

// Timer1 16-bit counter -> one compare interval is OCR1A+1 ticks. A phase may
// be longer than the 16-bit range (e.g. very low RPM), so we burn it down in
// chunks of up to 65536 ticks; only the final chunk ends the phase.
//
// CRITICAL: OCR1A is re-armed *inside* the ISR, after the compare match already
// reset TCNT1 to 0 and the ISR prologue/pinMode advanced it a few ticks. In CTC
// mode TOP=OCR1A, so if we ever load an OCR1A below the current TCNT1, the
// counter misses the match, runs to 0xFFFF, wraps, and only then matches ->
// ~65536 stray ticks (~33 ms) bolted onto that phase. If a phase consistently
// ended in a tiny tail, that error would repeat every cycle (~2x RPM error).
// Two guards prevent any sub-latency interval from ever being scheduled:
//   * CHUNK_FLOOR: a final chunk is never shorter than this (>> ISR latency).
//   * Split rule: when the remainder is in (65536, 131072], halve it instead of
//     emitting a full 65536 + a tiny tail. Each half is then >= 32768 ticks, so
//     the final <=65536 chunk that ends the phase is always comfortably large
//     (unless the whole phase was pathologically short, which CHUNK_FLOOR caps).
static const uint32_t CHUNK_FLOOR = 100;   // ticks; safely above ISR latency

static inline void armNextChunk() {
  if (g_remTicks > 65536UL) {
    uint32_t chunk = (g_remTicks <= 2UL * 65536UL) ? (g_remTicks / 2)  // avoid tiny tail
                                                   : 65536UL;
    OCR1A = (uint16_t)(chunk - 1);
    g_remTicks -= chunk;
  } else {
    uint32_t chunk = g_remTicks ? g_remTicks : 1;  // never schedule a zero interval
    if (chunk < CHUNK_FLOOR) chunk = CHUNK_FLOOR;  // floor the final interval
    OCR1A = (uint16_t)(chunk - 1);                 // final chunk ends the phase
    g_remTicks = 0;
  }
}

// Minimal ISR: either continue burning down the current phase, or flip the pin
// mode to start the next phase and reschedule. No serial / no blocking work.
ISR(TIMER1_COMPA_vect) {
  if (g_remTicks > 0) {          // more chunks left in this phase
    armNextChunk();
    return;
  }
  if (g_state == STATE_PULSE) {
    pinMode(SIGNAL_PIN, INPUT);  // OPEN / high-Z idle -> controller pull-up = HIGH
    g_state    = STATE_REST;
    g_remTicks = g_restTicks;
  } else {
    pinMode(SIGNAL_PIN, OUTPUT); // CONDUCT -> LOW (port bit already 0)
    g_state    = STATE_PULSE;
    g_remTicks = g_pulseTicks;
  }
  armNextChunk();
}

void setup() {
  // Force the output latch LOW once and rest in high-Z INPUT (the Pro Micro's
  // default at power-up). INPUT+PORT=0 -> high-Z, no pull-up -> idle HIGH via
  // controller. OUTPUT+PORT=0 -> driven LOW. So we only ever flip DDR; the line
  // is never driven HIGH and there is no spurious pulse at boot.
  digitalWrite(SIGNAL_PIN, LOW);
  pinMode(SIGNAL_PIN, INPUT);

  // ---- compute phase durations from the constants (once at startup) ----
  long rpm = TARGET_RPM;
  if (rpm <= 0) {
    // RPM = 0 / stop: hold open (high-Z, HIGH) continuously, emit no pulses.
    // This is the "healthy sensor, wheel stopped" signal. Timer never starts.
    // A negative TARGET_RPM also lands here: stop is the only sane reading of
    // "negative speed", so we treat it as stop rather than clamping to MAX_RPM.
    return;
  }
  if (rpm > MAX_RPM) rpm = MAX_RPM;          // clamp out-of-range RPM

  float freq_hz = (float)rpm * (float)PULSES_PER_REV / 60.0f;   // LOW pulses/sec
  uint32_t period_us = (uint32_t)(1000000.0f / freq_hz + 0.5f); // rest+pulse cycle
  if (period_us == 0) period_us = 1;

  // Fixed pulse width, but never let it approach the period (clamp to a
  // fraction of it) so high RPM still leaves a clean rest gap.
  uint32_t pulse_us = (uint32_t)PULSE_WIDTH_US;
  uint32_t cap_us   = (uint32_t)((float)period_us * PULSE_DUTY_CAP);
  if (pulse_us > cap_us) pulse_us = cap_us;
  if (pulse_us == 0)     pulse_us = 1;
  uint32_t rest_us = period_us - pulse_us;

  g_pulseTicks = pulse_us * TICKS_PER_US;
  g_restTicks  = rest_us  * TICKS_PER_US;
  if (g_pulseTicks == 0) g_pulseTicks = 1;
  if (g_restTicks  == 0) g_restTicks  = 1;

  // ---- Timer1: CTC (WGM12), /8 prescaler (CS11), compare-A interrupt ----
  noInterrupts();
  TCCR1A = 0;                       // normal port operation, CTC via TCCR1B
  TCCR1B = (1 << WGM12) | (1 << CS11);
  TCNT1  = 0;
  TIMSK1 = (1 << OCIE1A);           // enable Timer1 compare-A interrupt

  // Start in REST so the line idles HIGH first (no pulse at boot); the first
  // ISR after the rest interval begins the first LOW pulse.
  g_state    = STATE_REST;
  g_remTicks = g_restTicks;
  armNextChunk();
  interrupts();
}

void loop() {
  // Nothing to do: all pulse generation is interrupt-driven for timing
  // accuracy. The MCU may idle here. No Serial -> safe to run standalone.
}
