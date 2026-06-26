// Stream A0 samples over serial for oscilloscope view.
// 2-wire sensor (reed / Hall switch): A0 -> sensor -> GND, internal pullup on.
// Logic inverted: switch OPEN -> ~1023 (HIGH), magnet CLOSES -> ~0 (LOW).
const uint8_t PIN_SENSOR = A0;
const unsigned long BAUD = 115200;

// Target sample rate ~1 kHz. Lower SAMPLE_US = faster sampling.
const unsigned long SAMPLE_US = 1000;
unsigned long lastSample = 0;

void setup() {
  Serial.begin(BAUD);
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  while (!Serial) { ; }  // wait for USB CDC (ATmega32U4 needs this)
}

void loop() {
  unsigned long now = micros();
  if (now - lastSample >= SAMPLE_US) {
    lastSample += SAMPLE_US;
    int v = analogRead(PIN_SENSOR);   // 0..1023
    // CSV: micros,value  -> easy to plot and log
    Serial.print(now);
    Serial.print(',');
    Serial.println(v);
  }
}
