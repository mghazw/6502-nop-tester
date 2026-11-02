// W65C02S "NOP tester" for Arduino Mega 2560
//
// The Mega is the ONLY thing the CPU is connected to. It supplies the clock
// and reset, answers every read with $EA (NOP), serves a chosen reset vector,
// and checks that the CPU's address bus does exactly what a healthy CPU does:
//
//   reset sequence ... FFFC, FFFD, then vec, vec+1, vec+1, vec+2, vec+2, ...
//
// It runs three vectors ($EAEA, $5555, $AAAA). Together they put a 0 and a 1
// on every data line of the CPU, and counting up exercises the address lines.
//
// Wiring (CPU = W65C02S, DIP-40):
//   Mega 22,24,26 ... 52  -> A0..A15  (CPU pins 9-20 = A0-A11, 22-25 = A12-A15)
//   Mega A0..A7           -> D0..D7   (CPU pins 33,32,31,30,29,28,27,26)
//   Mega 3                <- RWB      (CPU pin 34)
//   Mega 7                -> PHI2     (CPU pin 37, clock input)
//   Mega 4                -> RESB     (CPU pin 40)
//   +5V: VDD pin 8, BE pin 36, RDY pin 2, IRQB pin 4, NMIB pin 6, SOB pin 38
//   GND: VSS pin 21, and the Mega GND. Add a 0.1uF capacitor across pins 8/21.
//   Nothing else on the bus: no ROM, RAM, 6522, UART or other Arduino.
//
// Serial monitor: 115200 baud. Send any character to run the tests again.

const uint8_t PIN_CLK  = 7;
const uint8_t PIN_RESB = 4;
const uint8_t PIN_RWB  = 3;

const unsigned int HALF_US = 20;    // PHI2 high/low time. ~10 kHz clock, fine for a W65C02S
const uint8_t RESET_CYCLES = 12;    // clock cycles with RESB held low
const uint8_t N = 56;               // cycles recorded after RESB is released

uint16_t logAddr[N];
uint8_t  logRw[N];

const uint16_t VECTORS[] = {0xEAEA, 0x5555, 0xAAAA};

uint16_t readAddress() {
  uint16_t a = 0;
  for (uint8_t i = 0; i < 16; i++) {
    if (digitalRead(22 + 2 * i)) a |= (uint16_t)1 << i;
  }
  return a;
}

void driveData(uint8_t v) { DDRF = 0xFF; PORTF = v; }   // A0..A7 = PF0..PF7 = D0..D7
void releaseData()        { DDRF = 0x00; PORTF = 0x00; }

uint8_t dataFor(uint16_t a, uint16_t vec) {
  if (a == 0xFFFC) return vec & 0xFF;
  if (a == 0xFFFD) return vec >> 8;
  return 0xEA;
}

// One full clock cycle. The address is valid for the whole of PHI2 high, so it
// is read in the middle of the high phase and the data is placed on the bus
// before PHI2 falls (the CPU latches data on the falling edge).
void cycle(uint16_t vec, int8_t slot) {
  digitalWrite(PIN_CLK, HIGH);
  delayMicroseconds(HALF_US);

  uint16_t a = readAddress();
  uint8_t  r = digitalRead(PIN_RWB) ? 1 : 0;
  if (r) driveData(dataFor(a, vec)); else releaseData();
  if (slot >= 0) { logAddr[slot] = a; logRw[slot] = r; }

  delayMicroseconds(HALF_US);
  digitalWrite(PIN_CLK, LOW);
  delayMicroseconds(5);              // hold data briefly after the falling edge
  releaseData();
  delayMicroseconds(HALF_US);
}

void printBin16(uint16_t v) {
  for (int8_t i = 15; i >= 0; i--) Serial.print((v >> i) & 1 ? '1' : '0');
}

void printHex4(uint16_t v) {
  for (int8_t i = 3; i >= 0; i--) Serial.print((v >> (i * 4)) & 0xF, HEX);
}

void dumpTrace() {
  Serial.println(F("  First recorded cycles (addr rw):"));
  for (uint8_t k = 0; k < 24 && k < N; k++) {
    Serial.print(F("    "));
    printHex4(logAddr[k]);
    Serial.println(logRw[k] ? F("  r") : F("  w"));
  }
}

bool analyse(uint16_t vec) {
  int i = -1;
  for (int k = 0; k < N - 2; k++) {
    if (logAddr[k] == 0xFFFC && logRw[k]) { i = k; break; }
  }
  if (i < 0) {
    Serial.println(F("  FAIL: the CPU never read the reset vector at FFFC."));
    Serial.println(F("        Check power, BE, RDY, RESB, clock and the control lines."));
    dumpTrace();
    return false;
  }
  if (logAddr[i + 1] != 0xFFFD) {
    Serial.print(F("  FAIL: after FFFC the CPU read "));
    printHex4(logAddr[i + 1]);
    Serial.println(F(" instead of FFFD (an address line is wrong)."));
    dumpTrace();
    return false;
  }

  uint16_t got = logAddr[i + 2];
  if (got != vec) {
    uint16_t diff = got ^ vec;
    Serial.print(F("  FAIL: vector "));
    printHex4(vec);
    Serial.print(F(" was fed, but the CPU jumped to "));
    printHex4(got);
    Serial.println(F("."));
    Serial.print(F("        expected ")); printBin16(vec);
    Serial.print(F("\n        got      ")); printBin16(got);
    Serial.print(F("\n        wrong    ")); printBin16(diff);
    Serial.println();
    for (uint8_t b = 0; b < 16; b++) {
      if (diff & ((uint16_t)1 << b)) {
        Serial.print(F("        -> CPU did not read data line D"));
        Serial.print(b & 7);
        Serial.println(b < 8 ? F(" correctly at FFFC") : F(" correctly at FFFD"));
      }
    }
    return false;
  }

  // The CPU is now executing NOPs: each address appears twice
  // (opcode fetch + dummy read), so address = vec + (k + 1) / 2.
  uint8_t checked = 0;
  for (int k = 0; i + 2 + k < N && k < 32; k++) {
    int idx = i + 2 + k;
    uint16_t expect = vec + (k + 1) / 2;
    if (logAddr[idx] != expect || !logRw[idx]) {
      Serial.print(F("  FAIL: NOP cycle "));
      Serial.print(k);
      Serial.print(F(": expected "));
      printHex4(expect);
      Serial.print(F(" r, got "));
      printHex4(logAddr[idx]);
      Serial.println(logRw[idx] ? F(" r") : F(" w"));
      uint16_t diff = logAddr[idx] ^ expect;
      for (uint8_t b = 0; b < 16; b++) {
        if (diff & ((uint16_t)1 << b)) {
          Serial.print(F("        -> address bit A"));
          Serial.print(b);
          Serial.println(F(" differs (check A line, CPU pin and Mega pin)"));
        }
      }
      return false;
    }
    checked++;
  }
  Serial.print(F("  PASS: reset vector read, jumped to "));
  printHex4(vec);
  Serial.print(F(", then "));
  Serial.print(checked);
  Serial.println(F(" correct NOP cycles."));
  return true;
}

bool runTest(uint16_t vec) {
  Serial.print(F("Test: reset vector $"));
  printHex4(vec);
  Serial.println();

  releaseData();
  digitalWrite(PIN_CLK, LOW);
  digitalWrite(PIN_RESB, LOW);
  delay(5);
  for (uint8_t n = 0; n < RESET_CYCLES; n++) cycle(vec, -1);

  digitalWrite(PIN_RESB, HIGH);
  for (uint8_t n = 0; n < N; n++) cycle(vec, n);

  return analyse(vec);
}

void runAll() {
  bool ok = true;
  for (uint8_t t = 0; t < sizeof(VECTORS) / sizeof(VECTORS[0]); t++) {
    ok &= runTest(VECTORS[t]);
    Serial.println();
  }
  Serial.println(ok ? F("RESULT: all tests passed. The CPU's logic looks fine.")
                    : F("RESULT: at least one test failed. See the details above."));
  Serial.println(F("Send any character to run again."));
}

void setup() {
  Serial.begin(115200);

  for (uint8_t pin = 22; pin <= 52; pin += 2) pinMode(pin, INPUT);
  releaseData();
  pinMode(PIN_RWB, INPUT);

  pinMode(PIN_CLK, OUTPUT);
  digitalWrite(PIN_CLK, LOW);
  pinMode(PIN_RESB, OUTPUT);
  digitalWrite(PIN_RESB, LOW);

  delay(500);
  Serial.println(F("W65C02S NOP tester"));
  runAll();
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    runAll();
  }
}
