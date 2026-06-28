// =============================================================================
//  Half-Speed Repeater  -  Arduino (ATmega32U4, 5V/16MHz)
// =============================================================================
//
//  WHAT THIS IS
//  ------------
//  Reads the REAL magnet-on-wheel sensor (like scope/) and re-emits the pulse
//  train to the controller (like emulator/) at HALF the rate -> controller
//  displays HALF the true speed.
//
//  WHY HALF-RATE = HALF-SPEED
//  --------------------------
//  See emulator.ino: the controller counts EDGES; displayed speed is set by the
//  pulse RATE (period = 60 / (RPM * magnets/rev)). Drop the rate by 2 and the
//  controller reads half the ground speed -- independent of wheel diameter,
//  magnet geometry, or acceleration. We do it by RELAYING every 2nd input
//  pulse: copy that pulse's exact level (and therefore its exact width) to the
//  output, and swallow the pulse in between.
//
//  WIRING
//  ------
//    Sensor (2-wire reed / Hall switch), same as scope/:
//      sensor wire A --> A0      (PIN_SENSOR, internal pullup on)
//      sensor wire B --> GND
//    Controller line, same as emulator/:
//      A2 (SIGNAL_PIN) --> controller's signal sensor line
//      Arduino GND     --> controller GND   (shared ground, required)
//    Sensor GND, Arduino GND and controller GND must all be common.
//
//    Flash over USB, then wire sensor + controller and run.
//
//  ELECTRICAL MODEL (output) -- mirrors emulator.ino:
//    Real sensor = dry contact / open-collector: idle HIGH (controller pull-up),
//    magnet present = line pulled toward GND. PUSH_PULL 0 = open-collector
//    (high-Z idle, drive LOW on pulse); PUSH_PULL 1 = actively drive both rails.
// =============================================================================

// ----------------------------- CONFIGURATION ---------------------------------
const uint8_t PIN_SENSOR = A0;     // sensor input (idle ~1023 HIGH, magnet ~0 LOW)
#define SIGNAL_PIN        A2       // output to controller's signal pin
#define DIVISOR           2        // emit 1 output pulse per DIVISOR input pulses

// analogRead() thresholds with hysteresis -> reject contact bounce / noise.
const int TH_LOW  = 300;           // below this  -> magnet present (pulse, LOW)
const int TH_HIGH = 700;           // above this  -> idle (HIGH)

#define PUSH_PULL         0        // 1=drive both rails, 0=open-collector
// -----------------------------------------------------------------------------

// --- output line drive (identical convention to emulator.ino) ---
#if PUSH_PULL
static inline void lineConduct() { digitalWrite(SIGNAL_PIN, LOW);  } // pulse: 0V
static inline void lineOpen()    { digitalWrite(SIGNAL_PIN, HIGH); } // rest:  HIGH
#else
static inline void lineConduct() { pinMode(SIGNAL_PIN, OUTPUT); }    // drive LOW
static inline void lineOpen()    { pinMode(SIGNAL_PIN, INPUT);  }    // high-Z idle
#endif

// DEBUG: Pro Micro RX LED (pin 17, active-LOW); lit while relaying a pulse.
#define LED_PIN 17
static inline void ledOn()  { digitalWrite(LED_PIN, LOW);  }
static inline void ledOff() { digitalWrite(LED_PIN, HIGH); }

static bool     g_inPulse   = false;  // input currently LOW (magnet present)?
static uint16_t g_count     = 0;      // input pulses seen since last relay
static bool     g_relayNow  = false;  // is the CURRENT input pulse being relayed?

void setup() {
  pinMode(PIN_SENSOR, INPUT_PULLUP);  // same as scope/: weak pullup, inverted logic

  digitalWrite(SIGNAL_PIN, LOW);
#if PUSH_PULL
  pinMode(SIGNAL_PIN, OUTPUT);        // stays OUTPUT; we toggle the level
#endif
  lineOpen();                         // start at rest (HIGH / high-Z)

  pinMode(LED_PIN, OUTPUT);
  ledOff();
}

void loop() {
  // DIVIDE BY 2: relay every DIVISOR-th input pulse, swallow the rest.
  // Output rate = input rate / DIVISOR -> controller shows HALF the speed.
  int v = analogRead(PIN_SENSOR);     // 0..1023, inverted: LOW = magnet present

  if (!g_inPulse) {
    // idle -> falling edge (HIGH -> LOW = magnet present): pulse start
    if (v < TH_LOW) {
      g_inPulse = true;
      g_count++;
      if (g_count >= DIVISOR) {       // this is the Nth pulse -> relay it
        g_count    = 0;
        g_relayNow = true;
        lineConduct(); ledOn();       // mirror input level for exact width
      } else {
        g_relayNow = false;           // swallowed: output stays idle
      }
    }
  } else {
    // in pulse -> rising edge (LOW -> HIGH = magnet gone): pulse end
    if (v > TH_HIGH) {
      g_inPulse = false;
      if (g_relayNow) {
        lineOpen(); ledOff();         // end relayed pulse (same width as input)
        g_relayNow = false;
      }
    }
  }
}
