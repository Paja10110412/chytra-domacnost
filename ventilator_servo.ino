/*
  Ventilátor ESP32 – ruční ovládání + automatika podle teploty (relé ON/OFF)
  + servo okna (GPIO33, nativní LEDC, bez další knihovny)
  + OLED displej (SSD1306 128x64, I2C) + teplota i vlhkost z DHT11 + web přímo na ESP32

  Deska:    ESP32 Dev Module (ESP32-WROOM-32), Arduino-ESP32 3.x
            (API ledcAttach/ledcWrite/ledcDetach s číslem pinu; ve verzi 2.x se nepřeloží).
  Knihovny (Správce knihoven v Arduino IDE):
            - "DHT sensor library" (Adafruit) + "Adafruit Unified Sensor" (závislost)
            - "Adafruit SSD1306" + "Adafruit GFX Library" + "Adafruit BusIO" (závislosti)
            WiFi, WebServer, Preferences a Wire jsou součástí balíčku Arduino-ESP32.
  Zapojení: relé IN   -> GPIO27 (aktivní úroveň: konstanta RELAY_ACTIVE_HIGH v CONFIG)
            DHT11 DATA -> GPIO26
            OLED SDA  -> GPIO21, OLED SCL -> GPIO22
            Servo signál -> GPIO33; servo napájet z regulovaných 5 V, NIKDY z 3V3 ani z GPIO
            VCC (DHT11, OLED) -> 3V3, všechny GND společné (zdroj, ESP32, relé, motor, servo).
  Web:      připoj se na Wi-Fi "Web_Control_Fan" a otevři http://192.168.4.1
            (HTML/CSS/JS jsou přímo v tomto souboru, internet není potřeba).

  Ventilátor:
  - Po startu je vždy ventilátor VYPNUTÝ a režim AUTO. Do paměti se ukládá teplotní hranice,
    hystereze a nastavení okna; režim ani ruční ON/OFF se po restartu neobnovují.
  - RUČNĚ: ON/OFF ovládá jen uživatel, automatika do ventilátoru nezasahuje.
    V AUTO jsou tlačítka ON/OFF zamčená a web vysvětlí proč.
  - Přepnutí AUTO -> RUČNĚ: ventilátor se vypne (ruční režim vždy začíná na OFF).
  - Přepnutí RUČNĚ -> AUTO: okamžitě se vyhodnotí poslední čerstvé měření;
    v pásmu mezi vypnutím a zapnutím zůstane stávající stav.
  - AUTO: zapne nad hranicí, vypne při (hranice - hystereze) nebo níže, mezi tím drží stav.
  - Výpadek čidla v AUTO: do SENSOR_STALE_MS se hodnota bere jako aktuální, potom do
    SENSOR_FAILSAFE_MS ventilátor drží poslední stav, potom se vypne a zůstane vypnutý,
    dokud čidlo nedodá platnou hodnotu. Ruční režim funguje dál.
  - Ruční ON se po MANUAL_LIMIT_MS (30 min) samo vypne; režim zůstane RUČNĚ.
    Nový stisk ON odpočet 30 min spustí znovu.
  - Čidlo se fyzicky čte jen v measure() každých SENSOR_INTERVAL_MS (3 s). Web i OLED zobrazují
    totéž uložené měření. Teplota a vlhkost mají vlastní platnost a stáří.

  Okno (servo):
  - Výchozí režim „S ventilátorem“: okno se otevře, když ventilátor zapne, a zavře, když vypne
    (AUTO i RUČNĚ, včetně bezpečnostního vypnutí). Žádná druhá teplotní automatika.
  - Tlačítka „Otevřít“ / „Zavřít“ přepíšou sledování na WINDOW_OVERRIDE_MS (10 min), potom se
    režim „S ventilátorem“ sám obnoví; nový ruční povel odpočet obnoví. Změna cíle za pohybu
    se uplatní okamžitě.
  - Úhly otevření/zavření (0–180° povelu) a rychlost (°/s) se nastavují na webu a ukládají
    do Preferences jen při potvrzené změně. Úhly lze prohodit (opačná orientace mechanismu).
  - Pohyb je neblokující a řízený podle millis(): poloha povelu se posouvá o rychlost x uplynulý
    čas, PWM se zapisuje jen při změně střídy. Bez snímače polohy se zobrazuje pouze POVEL
    (cíl, vydávaný úhel, délka impulzu), nikdy „změřená“ nebo „potvrzená“ poloha.
  - Po restartu ESP32 neví, kde servo fyzicky je. První povel (po SERVO_BOOT_QUIET_MS) proto
    servo vykoná plnou rychlostí serva skokem; nastavená rychlost platí až od dalšího pohybu.
  - Odstup SERVO_AFTER_RELAY_MS po každém přepnutí relé je jen neblokující úleva pro špičku
    proudu. Nenahrazuje dostatečně silný zdroj.
  - Chyba PWM serva zablokuje jen servo; web, měření a ventilátor běží dál a ESP32 inicializaci
    opakuje.

  Diagnostika: při startu se do Serial Monitoru (115200) vypíše důvod restartu; web ukazuje
  dobu běhu, důvod restartu, stáří měření, poslední důvod změny ventilátoru, stav serva, volnou
  paměť a nejdelší pauzu hlavní smyčky.
  Log: posledních 20 událostí (časy od spuštění ESP32, po restartu se mažou).
*/

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <DHT.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <esp_system.h>

// ==CONFIG-BEGIN== ---------------- NASTAVENÍ ----------------
const uint8_t RELAY_PIN = 27;
// !!! AKTIVNÍ ÚROVEŇ RELÉ: hodnota true je převzatá z přiloženého programu, není změřená ani
// ověřená. true = relé sepne při HIGH, false = relé sepne při LOW (řada modulů s optočlenem je
// aktivní LOW). Pokud se relé po startu samo sepne nebo je stav opačný než na webu, změň ji.
const bool RELAY_ACTIVE_HIGH = true;

// ---- Servo okna (GPIO33, nativní LEDC) ----
const uint8_t  SERVO_PIN = 33;
const uint32_t SERVO_HZ = 50;
const uint8_t  SERVO_BITS = 16;
// Délka impulzu pro povel 0° a 180°. Konzervativní výchozí rozsah 1000..2000 us: skutečný
// mechanický rozsah serva se může lišit (často je menší než 180°). Rozšiřuj opatrně a bez
// táhla (typicky 600..2400 us); servo nesmí tlačit do mechanického dorazu.
const int SERVO_MIN_US = 500, SERVO_MAX_US = 2500;
// Výchozí softwarové polohy a rychlost. Platí do prvního uložení z webu. Jsou to jen povely,
// nikoli známé polohy okna: musí se přizpůsobit skutečnému servu a mechanismu.
const int WINDOW_CLOSED_DEFAULT_DEG = 30;
const int WINDOW_OPEN_DEFAULT_DEG   = 120;
const int SERVO_SPEED_DEFAULT = 90;                 // °/s
const int SERVO_SPEED_MIN = 10, SERVO_SPEED_MAX = 180;
const int SERVO_ANGLE_MIN = 0, SERVO_ANGLE_MAX = 180;  // rozsah POVELU, ne záruka rozsahu serva
const int SERVO_MIN_SPAN_DEG = 5;                   // nejmenší rozdíl otevřeno/zavřeno
const uint32_t SERVO_UPDATE_MS = 20;                // krok polohy = jedna PWM perioda (50 Hz)
const uint32_t SERVO_MAX_DT_MS = 100;               // po zdržení smyčky se pohyb nedohání skokem
const uint32_t SERVO_AFTER_RELAY_MS = 400;          // neblokující odstup po přepnutí relé
const uint32_t SERVO_BOOT_QUIET_MS = 1500;          // po startu servo nedostává impulzy (start Wi-Fi)
const uint32_t SERVO_RETRY_MS = 10000;              // opakování inicializace PWM po chybě
const uint32_t WINDOW_OVERRIDE_MS = 10UL * 60UL * 1000UL;
static_assert(SERVO_HZ == 50 && SERVO_BITS == 16, "Casovani serva: 50 Hz, 16 bit");
static_assert(SERVO_MIN_US >= 500 && SERVO_MAX_US <= 2500 && SERVO_MIN_US < SERVO_MAX_US,
              "Neplatny rozsah impulzu serva");
static_assert(SERVO_ANGLE_MIN == 0 && SERVO_ANGLE_MAX == 180, "Rozsah povelu je 0..180");
static_assert(SERVO_SPEED_MIN > 0 && SERVO_SPEED_DEFAULT >= SERVO_SPEED_MIN &&
              SERVO_SPEED_DEFAULT <= SERVO_SPEED_MAX, "Neplatna rychlost serva");
static_assert(WINDOW_CLOSED_DEFAULT_DEG >= SERVO_ANGLE_MIN && WINDOW_CLOSED_DEFAULT_DEG <= SERVO_ANGLE_MAX &&
              WINDOW_OPEN_DEFAULT_DEG >= SERVO_ANGLE_MIN && WINDOW_OPEN_DEFAULT_DEG <= SERVO_ANGLE_MAX &&
              (WINDOW_OPEN_DEFAULT_DEG > WINDOW_CLOSED_DEFAULT_DEG ? WINDOW_OPEN_DEFAULT_DEG - WINDOW_CLOSED_DEFAULT_DEG
                                                                   : WINDOW_CLOSED_DEFAULT_DEG - WINDOW_OPEN_DEFAULT_DEG) >= SERVO_MIN_SPAN_DEG,
              "Neplatne vychozi uhly okna");
static_assert(SERVO_UPDATE_MS > 0 && SERVO_MAX_DT_MS >= SERVO_UPDATE_MS, "Neplatne casovani kroku serva");

const uint8_t DHT_PIN = 26;               // DHT11 DATA

const char* AP_SSID = "Web_Control_Fan";
const char* AP_PASSWORD = nullptr;        // nullptr = otevřená síť; jinak min. 8 znaků

// ---- OLED displej (I2C) ----
const bool     OLED_ENABLED = true;       // false = displej se vůbec nepoužije
const uint8_t  OLED_SDA_PIN = 21;
const uint8_t  OLED_SCL_PIN = 22;
const uint8_t  OLED_I2C_ADDR = 0x3C;      // nejčastěji 0x3C, někdy 0x3D
const bool     OLED_TRY_OTHER_ADDR = true;// když na OLED_I2C_ADDR nikdo neodpoví, zkusí 0x3C/0x3D
const uint8_t  OLED_WIDTH = 128;          // rozvržení je navržené jen pro 128 x 64
const uint8_t  OLED_HEIGHT = 64;
const uint8_t  OLED_ROTATION = 0;         // 0 = normálně, 2 = otočeno o 180 °
const uint32_t OLED_I2C_HZ = 400000;      // při problémech zkus 100000
const uint32_t OLED_REFRESH_MS = 250;     // jak často se kontroluje, zda se obsah změnil
const uint32_t OLED_RETRY_MS = 5000;      // pauza mezi pokusy o inicializaci displeje
const uint32_t OLED_FORCE_REDRAW_MS = 30000; // občasné překreslení i bez změny (oprava rušení)
const bool     OLED_SCAN_ON_BOOT = true;  // vypíše do Serial Monitoru nalezené I2C adresy

// DHT11 se smí číst nejvýše jednou za ~1 s, doporučeno 2 s a více.
const uint32_t SENSOR_INTERVAL_MS = 3000;
const uint32_t SENSOR_STALE_MS    = 10000;   // starší měření už není "aktuální"
const uint32_t SENSOR_FAILSAFE_MS = 60000;   // bez platného měření -> AUTO vypne motor
const uint32_t WEB_POLL_MS        = 3000;    // web jen čte poslední výsledek, neměří
static_assert(SENSOR_INTERVAL_MS >= 2000, "DHT11: interval mereni min. 2000 ms");
static_assert(WEB_POLL_MS >= 2000, "Web: obnoveni min. 2000 ms");
static_assert(SENSOR_STALE_MS > SENSOR_INTERVAL_MS, "STALE musi byt delsi nez interval");
static_assert(SENSOR_FAILSAFE_MS >= SENSOR_STALE_MS, "FAILSAFE musi byt >= STALE");
static_assert(RELAY_PIN != DHT_PIN && RELAY_PIN != OLED_SDA_PIN && RELAY_PIN != OLED_SCL_PIN &&
              DHT_PIN != OLED_SDA_PIN && DHT_PIN != OLED_SCL_PIN && OLED_SDA_PIN != OLED_SCL_PIN,
              "Konflikt pinu: relé, DHT11 a I2C musi mit ruzne GPIO");
static_assert(SERVO_PIN != RELAY_PIN && SERVO_PIN != DHT_PIN &&
              SERVO_PIN != OLED_SDA_PIN && SERVO_PIN != OLED_SCL_PIN, "Konflikt GPIO serva");
static_assert(OLED_WIDTH == 128 && OLED_HEIGHT == 64, "Rozvrzeni OLED je jen pro 128x64");

const int SENSOR_MIN_C = 0, SENSOR_MAX_C = 50;      // rozsah DHT11 (teplota)
const int HUM_MIN_PCT = 0, HUM_MAX_PCT = 100;       // rozumný rozsah vlhkosti (DHT11 uvádí 20–90 %)
const int THRESHOLD_MIN = 5, THRESHOLD_MAX = 45, THRESHOLD_DEFAULT = 25;
const int HYST_MIN = 1, HYST_MAX = 10, HYST_DEFAULT = 1;

const uint32_t MANUAL_LIMIT_MS = 30UL * 60UL * 1000UL;  // ruční ON se po této době samo vypne
const uint32_t HISTORY_STEP_MS = 30000;                 // jeden bod grafu
const int HISTORY_POINTS = 120;                         // 120 x 30 s = 60 min
const int LOG_SIZE = 20;                                // počet pamatovaných událostí
const uint32_t LOOP_WARN_MS = 250;                      // delší pauza hlavní smyčky se vypíše do Serialu
// ==CONFIG-END==

// ==LOGIC-BEGIN== (bez závislosti na Arduinu, proto jde testovat i na PC)
enum SensorState : uint8_t { SENSOR_WAITING, SENSOR_OK, SENSOR_STALE, SENSOR_ERROR };

SensorState sensorStateOf(bool haveGood, uint32_t ageMs, uint8_t failStreak) {
  if (haveGood) return ageMs <= SENSOR_STALE_MS ? SENSOR_OK : SENSOR_STALE;
  return failStreak > 0 ? SENSOR_ERROR : SENSOR_WAITING;
}

// Platné číslo v rozsahu (NaN a nekonečno propadnou).
bool readingInRange(float v, float lo, float hi) {
  return isfinite(v) && v >= lo && v <= hi;
}

// Je poslední platné měření ještě "aktuální"?
bool readingFresh(bool have, uint32_t ageMs) {
  return have && ageMs <= SENSOR_STALE_MS;
}

// Rozhodnutí automatiky. ageMs = stáří posledního platného měření teploty.
bool autoNextState(bool running, bool haveGood, int tempC, uint32_t ageMs,
                   int onAbove, int hyst) {
  if (!haveGood || ageMs > SENSOR_FAILSAFE_MS) return false;  // bezpečný stav
  if (ageMs > SENSOR_STALE_MS) return running;                // krátký výpadek: držet stav
  if (tempC > onAbove) return true;
  if (tempC <= onAbove - hyst) return false;
  return running;
}

bool settingsValid(int thr, int hyst) {
  return thr >= THRESHOLD_MIN && thr <= THRESHOLD_MAX &&
         hyst >= HYST_MIN && hyst <= HYST_MAX && thr - hyst >= 0;
}

// Ruční ON se po limitu samo vypne (jen v ručním režimu a při běžícím motoru).
bool manualLimitReached(bool manualMode, bool running, uint32_t runMs) {
  return manualMode && running && runMs >= MANUAL_LIMIT_MS;
}
uint32_t manualLeftMs(uint32_t runMs) {
  return runMs >= MANUAL_LIMIT_MS ? 0 : MANUAL_LIMIT_MS - runMs;
}

// ---- servo: čistá logika (bez hardwaru) ----
bool servoAngleValid(int deg) { return deg >= SERVO_ANGLE_MIN && deg <= SERVO_ANGLE_MAX; }

// nullptr = nastavení je v pořádku, jinak kód chyby, který web převede na vysvětlení.
const char* servoSettingsError(int closedDeg, int openDeg, int speedDps) {
  if (!servoAngleValid(closedDeg) || !servoAngleValid(openDeg)) return "bad_angle";
  if (abs(openDeg - closedDeg) < SERVO_MIN_SPAN_DEG) return "bad_span";
  if (speedDps < SERVO_SPEED_MIN || speedDps > SERVO_SPEED_MAX) return "bad_speed";
  return nullptr;
}

// Délka impulzu v mikrosekundách pro povelovaný úhel (NaN a hodnoty mimo rozsah se oříznou).
uint32_t servoPulseUs(float angleDeg) {
  if (!(angleDeg > (float)SERVO_ANGLE_MIN)) angleDeg = (float)SERVO_ANGLE_MIN;
  if (angleDeg > (float)SERVO_ANGLE_MAX) angleDeg = (float)SERVO_ANGLE_MAX;
  return (uint32_t)lroundf((float)SERVO_MIN_US + angleDeg * (float)(SERVO_MAX_US - SERVO_MIN_US) / 180.0f);
}

// Střída LEDC pro danou délku impulzu (perioda = 1 / SERVO_HZ, 2^SERVO_BITS kroků).
uint32_t servoDutyForUs(uint32_t us) {
  return (uint32_t)(((uint64_t)us * (1ULL << SERVO_BITS) * SERVO_HZ + 500000ULL) / 1000000ULL);
}

// Jeden krok pohybu povelu k cíli rychlostí speedDps za dtMs; nikdy nepřekročí cíl.
float servoStepToward(float pos, float target, int speedDps, uint32_t dtMs) {
  float step = (float)speedDps * (float)dtMs / 1000.0f;
  if (pos < target) return (target - pos <= step) ? target : pos + step;
  if (pos > target) return (pos - target <= step) ? target : pos - step;
  return pos;
}

// Důvod poslední změny stavu ventilátoru (texty jsou ve webu).
enum FanReason : uint8_t {
  FR_BOOT, FR_TEMP_ON, FR_TEMP_OFF, FR_MANUAL_ON, FR_MANUAL_OFF,
  FR_MODE_MANUAL, FR_MANUAL_TIMEOUT, FR_SENSOR_FAILSAFE
};

// Kódy událostí pro log (texty jsou ve webu).
enum EventCode : uint8_t {
  EV_BOOT, EV_MODE_AUTO, EV_MODE_MANUAL, EV_FAN_ON_MANUAL, EV_FAN_OFF_MANUAL,
  EV_FAN_ON_AUTO, EV_FAN_OFF_AUTO, EV_FAILSAFE, EV_MANUAL_TIMEOUT,
  EV_SENSOR_LOST, EV_SENSOR_BACK, EV_SETTINGS,
  EV_WINDOW_FOLLOW, EV_WINDOW_OPEN, EV_WINDOW_CLOSE, EV_SERVO_FAULT,
  EV_WINDOW_TIMEOUT, EV_SERVO_START, EV_SERVO_OK, EV_WINDOW_SETTINGS, EV_WINDOW_TEST,
  EV_NONE = 255
};
// ==LOGIC-END==

// Čitelné názvy událostí pro Serial Monitor (pořadí musí odpovídat enum EventCode).
const char* const EVENT_NAMES[] = {
  "start ESP32", "rezim AUTO", "rezim RUCNE", "ventilator ON rucne", "ventilator OFF rucne",
  "ventilator ON automatikou", "ventilator OFF automatikou", "ventilator OFF: chyba cidla",
  "ventilator OFF: limit rucniho behu", "cidlo bez platnych dat", "cidlo opet meri",
  "zmena nastaveni automatiky", "okno sleduje ventilator", "rucni povel: okno otevrit",
  "rucni povel: okno zavrit", "chyba PWM serva", "rucni volba okna skoncila",
  "servo: prvni povel po startu", "servo: PWM znovu funguje", "zmena nastaveni okna",
  "zkusebni povel okna"
};
static_assert(sizeof(EVENT_NAMES) / sizeof(EVENT_NAMES[0]) == EV_WINDOW_TEST + 1,
              "EVENT_NAMES neodpovida EventCode");

#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
WebServer server(80);
Preferences prefs;
Adafruit_SSD1306 oled(OLED_WIDTH, OLED_HEIGHT, &Wire, -1, OLED_I2C_HZ, OLED_I2C_HZ);

bool prefsReady = false;
bool motorOn = false;
bool autoMode = true;
bool servoReady = false;       // PWM (LEDC) je nastavené a funguje
bool servoAttached = false;    // pin je právě připojený k LEDC
bool servoStarted = false;     // po startu už byl vydán první povel
bool servoFaultLogged = false;
// 0 = sledovat ventilator, 1 = rucne otevrit, 2 = rucne zavrit
uint8_t windowMode = 0;
int windowTestDeg = -1;        // >= 0: zkušební (neuložený) úhel z webu místo uložené polohy
int windowClosedDeg = WINDOW_CLOSED_DEFAULT_DEG;
int windowOpenDeg = WINDOW_OPEN_DEFAULT_DEG;
int servoSpeedDps = SERVO_SPEED_DEFAULT;
float servoPos = WINDOW_CLOSED_DEFAULT_DEG;  // povel vydávaný servu, NE změřená poloha okna
int servoTarget = WINDOW_CLOSED_DEFAULT_DEG;
uint32_t servoCmdUs = 0, servoDutyNow = 0;
uint32_t windowManualSince = 0, servoLastUpdateMs = 0, servoRetryMs = 0, servoBootMs = 0;
uint32_t lastRelayChangeMs = 0;
bool relayChanged = false;
bool windowTargetOpen() { return windowMode == 0 ? motorOn : windowMode == 1; }
int windowDesiredAngle() {
  if (windowMode == 0) return motorOn ? windowOpenDeg : windowClosedDeg;
  if (windowTestDeg >= 0) return windowTestDeg;
  return windowMode == 1 ? windowOpenDeg : windowClosedDeg;
}
uint32_t windowRemainingMs(uint32_t now) {
  if (windowMode == 0) return 0;
  uint32_t age = now - windowManualSince;
  return age >= WINDOW_OVERRIDE_MS ? 0 : WINDOW_OVERRIDE_MS - age;
}
int thresholdC = THRESHOLD_DEFAULT;
int hysteresisC = HYST_DEFAULT;

// Teplota (řídí motor)
bool haveGood = false;
int lastTempC = 0;
uint32_t lastGoodMs = 0;
uint32_t lastAttemptMs = 0;
uint8_t failStreak = 0;

// Vlhkost (jen informativní, vlastní platnost a stáří)
bool haveHum = false;
int lastHumPct = 0;
uint32_t lastHumMs = 0;
uint8_t humFail = 0;

uint32_t manualOnSinceMs = 0;   // kdy byl naposledy stisknut ruční ON
bool sensorWasLost = false;

// Diagnostika
uint8_t fanReason = FR_BOOT;    // proč ventilátor naposledy změnil stav
uint32_t fanReasonMs = 0;
const char* resetReasonStr = "UNKNOWN";
uint32_t loopLastMs = 0, loopMaxGapMs = 0;

// OLED stav
bool oledOk = false;
uint8_t oledAddr = OLED_I2C_ADDR;
uint32_t oledLastTick = 0, oledLastTry = 0, oledLastDraw = 0;
char oledSig[80] = "";

struct LogEntry { uint32_t ms; uint8_t code; };
LogEntry logBuf[LOG_SIZE];
uint8_t logHead = 0, logCount = 0;
uint32_t logSeq = 0;            // zvyšuje se s každou událostí, web podle toho načítá log

const int8_t HIST_INVALID = -128;
int8_t histTemp[HISTORY_POINTS];
uint8_t histFan[HISTORY_POINTS];
uint16_t histHead = 0, histCount = 0;
uint32_t histSeq = 0, lastHistMs = 0;

// ---------------------------------------------------------------- log a historie
void logEvent(uint8_t code) {
  logBuf[logHead] = { millis(), code };
  logHead = (logHead + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
  logSeq++;
  Serial.printf("[UDALOST] %s (cas od startu %lu s)\n",
                code < sizeof(EVENT_NAMES) / sizeof(EVENT_NAMES[0]) ? EVENT_NAMES[code] : "?",
                (unsigned long)(millis() / 1000));
}

void histPush(bool tempFresh) {
  histTemp[histHead] = tempFresh ? (int8_t)lastTempC : HIST_INVALID;
  histFan[histHead] = motorOn ? 1 : 0;
  histHead = (histHead + 1) % HISTORY_POINTS;
  if (histCount < HISTORY_POINTS) histCount++;
  histSeq++;
}

// ---------------------------------------------------------------- servo okna
// ---------------------------------------------------------------- servo okna
// Bez snímače polohy se řídí jen POVEL. servoPos je poslední vydaný povel ve stupních,
// nikoli změřená poloha okna.
bool servoWrite(float angleDeg) {
  uint32_t us = servoPulseUs(angleDeg);
  uint32_t duty = servoDutyForUs(us);
  if (duty == servoDutyNow) return true;          // PWM se nepřepisuje zbytečně
  if (!ledcWrite(SERVO_PIN, duty)) return false;
  servoDutyNow = duty;
  servoCmdUs = us;
  return true;
}

void servoRelease() {                             // pin do klidu (LOW), bez impulzů
  if (servoAttached) { ledcDetach(SERVO_PIN); servoAttached = false; }
  pinMode(SERVO_PIN, OUTPUT);
  digitalWrite(SERVO_PIN, LOW);
  servoDutyNow = 0;
  servoCmdUs = 0;
}

bool servoAttach() {
  servoRelease();
  if (!ledcAttach(SERVO_PIN, SERVO_HZ, SERVO_BITS)) return false;
  servoAttached = true;
  if (!ledcWrite(SERVO_PIN, 0)) { servoRelease(); return false; }  // žádné impulzy před prvním povelem
  return true;
}

void servoFault(const char* why) {
  servoReady = false;
  servoStarted = false;                           // fyzická poloha je znovu neznámá
  servoRelease();
  servoRetryMs = millis();
  Serial.printf("[SERVO] chyba PWM (%s), fyzicka poloha okna je neznama, zkusim znovu za %lu s\n",
                why, (unsigned long)(SERVO_RETRY_MS / 1000));
  if (!servoFaultLogged) { servoFaultLogged = true; logEvent(EV_SERVO_FAULT); }
}

void servoInit() {
  servoBootMs = millis();
  if (servoAttach()) {
    servoReady = true;
    Serial.printf("[SERVO] GPIO%u, %lu Hz, %u bit, impulzy %d..%d us pro 0..180 stupnu, prvni povel az za %lu ms\n",
                  SERVO_PIN, (unsigned long)SERVO_HZ, (unsigned)SERVO_BITS, SERVO_MIN_US, SERVO_MAX_US,
                  (unsigned long)SERVO_BOOT_QUIET_MS);
  } else {
    servoFault("ledcAttach/ledcWrite");
  }
}

void servoService() {
  uint32_t now = millis();
  if (windowMode != 0 && windowRemainingMs(now) == 0) {   // konec ruční volby okna
    windowMode = 0;
    windowTestDeg = -1;
    logEvent(EV_WINDOW_TIMEOUT);
  }
  servoTarget = windowDesiredAngle();             // cíl se přepočítává vždy, změna platí hned

  if (!servoReady) {                              // chyba PWM: jen občasný nový pokus
    if (now - servoRetryMs >= SERVO_RETRY_MS) {
      servoRetryMs = now;
      if (servoAttach()) {
        servoReady = true;
        servoFaultLogged = false;
        logEvent(EV_SERVO_OK);
      }
    }
    return;
  }

  if (!servoStarted) {                            // první povel po startu: poloha je neznámá
    if (now - servoBootMs < SERVO_BOOT_QUIET_MS) return;   // po startu chvíli bez impulzů
    servoPos = (float)servoTarget;                // skok plnou rychlostí serva
    if (!servoWrite(servoPos)) { servoFault("ledcWrite"); return; }
    servoStarted = true;
    servoLastUpdateMs = now;
    Serial.printf("[SERVO] prvni povel %d stupnu (%lu us), poloha pred nim neznama\n",
                  servoTarget, (unsigned long)servoCmdUs);
    logEvent(EV_SERVO_START);
    return;
  }

  if (servoPos == (float)servoTarget) { servoLastUpdateMs = now; return; }   // nic k vykonání
  if (relayChanged && now - lastRelayChangeMs < SERVO_AFTER_RELAY_MS) {      // neblokující odstup
    servoLastUpdateMs = now;
    return;
  }
  uint32_t dt = now - servoLastUpdateMs;
  if (dt < SERVO_UPDATE_MS) return;
  servoLastUpdateMs = now;
  if (dt > SERVO_MAX_DT_MS) dt = SERVO_MAX_DT_MS;
  float next = servoStepToward(servoPos, (float)servoTarget, servoSpeedDps, dt);
  if (!servoWrite(next)) { servoFault("ledcWrite"); return; }
  servoPos = next;
}

// ---------------------------------------------------------------- relé
void setMotor(bool on, const char* why, uint8_t ev, uint8_t reason) {
  if (on == motorOn) return;
  motorOn = on;
  digitalWrite(RELAY_PIN, (on == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
  lastRelayChangeMs = millis();
  relayChanged = true;
  fanReason = reason;
  fanReasonMs = lastRelayChangeMs;
  Serial.printf("[RELE] %s (%s) -> GPIO%u = %s\n", on ? "ON" : "OFF", why, RELAY_PIN,
                (on == RELAY_ACTIVE_HIGH) ? "HIGH" : "LOW");
  if (ev != EV_NONE) logEvent(ev);
}

void applyAutomaticControl(const char* why) {
  if (!autoMode) return;  // v ručním režimu automatika nikdy nezasahuje
  uint32_t age = millis() - lastGoodMs;
  bool next = autoNextState(motorOn, haveGood, lastTempC, age, thresholdC, hysteresisC);
  if (next == motorOn) return;
  bool failsafe = !haveGood || age > SENSOR_FAILSAFE_MS;
  setMotor(next, why, next ? EV_FAN_ON_AUTO : (failsafe ? EV_FAILSAFE : EV_FAN_OFF_AUTO),
           next ? FR_TEMP_ON : (failsafe ? FR_SENSOR_FAILSAFE : FR_TEMP_OFF));
}

// Události čidla, limit ručního ON a historie pro graf. Voláno z loop().
void housekeeping() {
  uint32_t now = millis();
  SensorState ss = sensorStateOf(haveGood, now - lastGoodMs, failStreak);

  bool lostNow = (ss == SENSOR_STALE) || (ss == SENSOR_ERROR && failStreak >= 3);
  if (lostNow && !sensorWasLost) { sensorWasLost = true; logEvent(EV_SENSOR_LOST); }
  else if (ss == SENSOR_OK && sensorWasLost) { sensorWasLost = false; logEvent(EV_SENSOR_BACK); }

  if (manualLimitReached(!autoMode, motorOn, now - manualOnSinceMs))
    setMotor(false, "limit rucniho ON", EV_MANUAL_TIMEOUT, FR_MANUAL_TIMEOUT);

  if (now - lastHistMs >= HISTORY_STEP_MS) {
    lastHistMs = now;
    histPush(ss == SENSOR_OK);
  }
}

// ---------------------------------------------------------------- čidlo
// Jediné místo, kde se fyzicky čte DHT11. Teplota i vlhkost pocházejí z jednoho čtení;
// platnost každé veličiny se hodnotí zvlášť, takže chyba jedné nezneplatní druhou.
void measure() {
  uint32_t now = millis();
  bool ok = dht.read(true);                       // jedno fyzické čtení (odstup >= 3 s)
  float t = ok ? dht.readTemperature() : NAN;     // další volání vrací data z cache knihovny
  float h = ok ? dht.readHumidity() : NAN;

  if (readingInRange(t, SENSOR_MIN_C, SENSOR_MAX_C)) {
    lastTempC = (int)lroundf(t);  // DHT11 má rozlišení 1 °C
    lastGoodMs = now;
    haveGood = true;
    failStreak = 0;
    Serial.printf("[DHT11] teplota %d C\n", lastTempC);
  } else {
    if (failStreak < 255) failStreak++;
    Serial.printf("[DHT11] chyba cteni teploty (%u v rade)\n", failStreak);
  }

  if (readingInRange(h, HUM_MIN_PCT, HUM_MAX_PCT)) {
    lastHumPct = (int)lroundf(h);
    lastHumMs = now;
    haveHum = true;
    humFail = 0;
    Serial.printf("[DHT11] vlhkost %d %%\n", lastHumPct);
  } else {
    if (humFail < 255) humFail++;
    Serial.printf("[DHT11] chyba cteni vlhkosti (%u v rade)\n", humFail);
  }
}

// ---------------------------------------------------------------- paměť
void loadSettings() {
  int thr = THRESHOLD_DEFAULT, hy = HYST_DEFAULT;
  int wc = WINDOW_CLOSED_DEFAULT_DEG, wo = WINDOW_OPEN_DEFAULT_DEG, ws = SERVO_SPEED_DEFAULT;
  if (prefsReady) {
    thr = prefs.getUChar("thr", THRESHOLD_DEFAULT);
    hy = prefs.getUChar("hys", HYST_DEFAULT);
    wc = prefs.getUShort("wc", WINDOW_CLOSED_DEFAULT_DEG);
    wo = prefs.getUShort("wo", WINDOW_OPEN_DEFAULT_DEG);
    ws = prefs.getUShort("ws", SERVO_SPEED_DEFAULT);
  }
  if (!settingsValid(thr, hy)) { thr = THRESHOLD_DEFAULT; hy = HYST_DEFAULT; }
  if (servoSettingsError(wc, wo, ws)) {
    Serial.println("[PAMET] neplatne ulozene nastaveni okna, pouzity vychozi hodnoty");
    wc = WINDOW_CLOSED_DEFAULT_DEG; wo = WINDOW_OPEN_DEFAULT_DEG; ws = SERVO_SPEED_DEFAULT;
  }
  thresholdC = thr;
  hysteresisC = hy;
  windowClosedDeg = wc;
  windowOpenDeg = wo;
  servoSpeedDps = ws;
  servoPos = (float)wc;
  servoTarget = wc;
}

// Zápis do Preferences jen při potvrzené změně. RAM se mění až po úspěšném zápisu.
bool saveAutoSettings(int thr, int hy) {
  if (!prefsReady) return false;
  bool ok = true;
  if (thr != thresholdC) ok = prefs.putUChar("thr", (uint8_t)thr) == sizeof(uint8_t);
  if (ok && hy != hysteresisC) ok = prefs.putUChar("hys", (uint8_t)hy) == sizeof(uint8_t);
  if (!ok) {                                       // vrátit původní hodnoty (best effort)
    prefs.putUChar("thr", (uint8_t)thresholdC);
    prefs.putUChar("hys", (uint8_t)hysteresisC);
  }
  return ok;
}

bool saveWindowSettings(int c, int o, int s) {
  if (!prefsReady) return false;
  bool ok = prefs.putUShort("wc", (uint16_t)c) == sizeof(uint16_t);   // vždy celá trojice
  if (ok) ok = prefs.putUShort("wo", (uint16_t)o) == sizeof(uint16_t);
  if (ok) ok = prefs.putUShort("ws", (uint16_t)s) == sizeof(uint16_t);
  if (!ok) {
    prefs.putUShort("wc", (uint16_t)windowClosedDeg);
    prefs.putUShort("wo", (uint16_t)windowOpenDeg);
    prefs.putUShort("ws", (uint16_t)servoSpeedDps);
  }
  return ok;
}

// ---------------------------------------------------------------- OLED
bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void i2cScan() {
  Serial.print("[I2C] nalezena zarizeni na adrese:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) { Serial.printf(" 0x%02X", a); found++; }
  }
  Serial.println(found ? "" : " zadne (zkontroluj SDA/SCL, napajeni a GND)");
}

bool oledInit() {
  uint8_t a = OLED_I2C_ADDR;
  if (!i2cPresent(a)) {
    if (!OLED_TRY_OTHER_ADDR) return false;
    uint8_t alt = (a == 0x3C) ? 0x3D : 0x3C;
    if (!i2cPresent(alt)) return false;
    a = alt;
    Serial.printf("[OLED] na 0x%02X nic, pouzivam 0x%02X\n", OLED_I2C_ADDR, a);
  }
  if (!oled.begin(SSD1306_SWITCHCAPVCC, a, false, false)) return false;
  oledAddr = a;
  oled.setRotation(OLED_ROTATION);
  oled.setTextWrap(false);
  oled.clearDisplay();
  oled.display();
  return true;
}

// Teplota 0 = čeká, 1 = platná, 2 = chyba/zastaralá
void oledDraw(int tState, int tempC, bool humOk, int humPct, bool isAuto, bool isOn,
              int onAbove, int offAt, const char* winTxt) {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);

  // Záhlaví: režim vlevo, stav motoru vpravo (při ON inverzně)
  oled.setCursor(0, 1);
  oled.print(isAuto ? "AUTO" : "RUCNE");
  const char* m = isOn ? "MOTOR ON" : "MOTOR OFF";
  int w = (int)strlen(m) * 6;
  if (isOn) { oled.fillRect(64, 0, 64, 10, SSD1306_WHITE); oled.setTextColor(SSD1306_BLACK); }
  else oled.drawRect(64, 0, 64, 10, SSD1306_WHITE);
  oled.setCursor(64 + (64 - w) / 2, 1);
  oled.print(m);
  oled.setTextColor(SSD1306_WHITE);

  // Teplota
  if (tState == 1) {
    oled.setTextSize(3);
    oled.setCursor(0, 16);
    oled.print(tempC);
    int x = oled.getCursorX();
    oled.drawCircle(x + 3, 18, 2, SSD1306_WHITE);   // stupeň
    oled.setTextSize(2);
    oled.setCursor(x + 8, 16);
    oled.print("C");
  } else if (tState == 2) {
    oled.setTextSize(2);
    oled.setCursor(0, 16);
    oled.print("CHYBA");
    oled.setTextSize(1);
    oled.setCursor(0, 34);
    oled.print("cidlo DHT11");
  } else {
    oled.setTextSize(3);
    oled.setCursor(0, 16);
    oled.print("---");
  }

  // Vlhkost (nezávislá na teplotě)
  oled.setTextSize(1);
  oled.setCursor(76, 15);
  oled.print("Vlhkost");
  oled.setTextSize(2);
  oled.setCursor(76, 25);
  if (humOk) { oled.print(humPct); oled.print("%"); }
  else oled.print("--");

  oled.drawFastHLine(0, 42, OLED_WIDTH, SSD1306_WHITE);

  // Nastavené hranice (vypínací hodnota je skutečně vypočtená)
  oled.setTextSize(1);
  oled.setCursor(0, 46);
  oled.printf("Zapnout: >%dC", onAbove);
  oled.setCursor(0, 55);
  oled.printf("Vypnout: <=%dC", offAt);

  oled.setCursor(94, 46);
  oled.print("Okno:");
  oled.setCursor(94, 55);
  oled.print(winTxt);   // povel okna; hvězdička = ruční volba, ne měřená poloha
  oled.display();
}

// Voláno z loop(); nikdy nečeká a nikdy nesahá na čidlo.
void oledUpdate() {
  if (!OLED_ENABLED) return;
  uint32_t now = millis();
  if (now - oledLastTick < OLED_REFRESH_MS) return;
  oledLastTick = now;

  if (!oledOk) {
    if (now - oledLastTry < OLED_RETRY_MS) return;
    oledLastTry = now;
    oledOk = oledInit();
    if (!oledOk) return;
    oledSig[0] = '\0';  // vynutit první vykreslení
    Serial.printf("[OLED] pripojen na 0x%02X\n", oledAddr);
  }

  SensorState ss = sensorStateOf(haveGood, now - lastGoodMs, failStreak);
  int tState = ss == SENSOR_OK ? 1 : (ss == SENSOR_WAITING ? 0 : 2);
  bool humOk = readingFresh(haveHum, now - lastHumMs);
  int offAt = thresholdC - hysteresisC;

  // Stručný povel okna (max. 5 znaků): OTEV / ZAVR, hvězdička = ruční volba,
  // CEKA = po startu zatím bez povelu, CHYBA = chyba PWM serva.
  char winTxt[8];
  snprintf(winTxt, sizeof(winTxt), "%s%s",
           !servoReady ? "CHYBA" : (!servoStarted ? "CEKA" : (windowTargetOpen() ? "OTEV" : "ZAVR")),
           (servoReady && servoStarted && windowMode != 0) ? "*" : "");

  char sig[80];
  snprintf(sig, sizeof(sig), "%d|%d|%d|%d|%d|%d|%d|%s", tState, tState == 1 ? lastTempC : 0,
           humOk ? lastHumPct : -1, thresholdC, offAt, autoMode ? 1 : 0, motorOn ? 1 : 0, winTxt);

  bool force = (now - oledLastDraw >= OLED_FORCE_REDRAW_MS);
  if (!force && strcmp(sig, oledSig) == 0) return;   // nic se nezměnilo, nekreslit

  if (!i2cPresent(oledAddr)) {                       // displej zmizel: přestat ho zkoušet
    oledOk = false;
    oledLastTry = now;
    Serial.println("[OLED] displej neodpovida, zkusim znovu pozdeji");
    return;
  }
  oledDraw(tState, lastTempC, humOk, lastHumPct, autoMode, motorOn, thresholdC, offAt, winTxt);
  strncpy(oledSig, sig, sizeof(oledSig));
  oledSig[sizeof(oledSig) - 1] = '\0';
  oledLastDraw = now;
}

// ---------------------------------------------------------------- web
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="cs"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light dark">
<title>Ventilátor ESP32</title>
<style>
:root{--bg:#eef1f8;--card:#fff;--ink:#17203a;--mute:#566080;--line:#dfe4f0;--pri:#3b5bdb;--ok:#0b8a62;--bad:#cf3347;--warn:#a85f00;--soft:#f2f4fb;--btn-ink:#fff;--shadow:0 8px 30px rgba(23,32,58,.08)}
@media(prefers-color-scheme:dark){:root{--bg:#0f1424;--card:#181f36;--ink:#eef1fb;--mute:#a4aecb;--line:#2a3354;--pri:#8aa0ff;--ok:#3fd1a0;--bad:#ff7487;--warn:#f2b04a;--soft:#1f2745;--btn-ink:#0b1020;--shadow:none}}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--ink);font:16px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,Arial,sans-serif}
main{max-width:760px;margin:0 auto;display:grid;gap:14px}
header{display:flex;justify-content:space-between;align-items:center;gap:10px;flex-wrap:wrap}
h1{margin:0;font-size:24px}
.pill{padding:6px 14px;border-radius:99px;font-size:14px;font-weight:700;background:var(--soft);border:1px solid var(--line);color:var(--mute)}
.pill.ok{color:var(--ok)}.pill.bad{color:var(--bad)}
.card{background:var(--card);border:1px solid var(--line);border-radius:22px;padding:20px;box-shadow:var(--shadow)}
.card h2{margin:0 0 12px;font-size:13px;letter-spacing:.06em;text-transform:uppercase;color:var(--mute)}
.hero{display:grid;gap:14px;grid-template-columns:1fr}
@media(min-width:700px){.hero{grid-template-columns:3fr 2fr}}
.temp{font-size:clamp(64px,19vw,100px);font-weight:800;line-height:1;letter-spacing:-.03em}
.temp.stale{color:var(--mute)}
.hum{font-size:clamp(44px,13vw,64px);font-weight:800;line-height:1;letter-spacing:-.03em}
.hum.stale{color:var(--mute)}
.chip{display:inline-block;margin-top:12px;padding:5px 12px;border-radius:99px;font-size:14px;font-weight:700;background:var(--soft);color:var(--mute)}
.chip.ok{color:var(--ok)}.chip.warn{color:var(--warn)}.chip.bad{color:var(--bad)}
.note{margin:10px 0 0;color:var(--mute);font-size:14px}
.fancard{text-align:center}
.fanv{width:96px;height:96px;color:var(--mute)}
.fanv.on{color:var(--ok)}
.fanv.spin{animation:spin .9s linear infinite}
@keyframes spin{to{transform:rotate(360deg)}}
@media(prefers-reduced-motion:reduce){.fanv.spin{animation:none}}
.fanstate{font-size:26px;font-weight:800;margin-top:6px}
.pair{display:grid;grid-template-columns:1fr 1fr;gap:12px}
button{min-height:58px;border-radius:16px;border:2px solid var(--line);background:var(--soft);color:var(--ink);font:700 19px system-ui,sans-serif;cursor:pointer;touch-action:manipulation}
button.active{background:var(--pri);border-color:var(--pri);color:var(--btn-ink)}
#on.active{background:var(--ok);border-color:var(--ok)}
#off.active{background:var(--bad);border-color:var(--bad)}
button:disabled{opacity:.45;cursor:not-allowed}
button:focus-visible,input:focus-visible{outline:3px solid var(--pri);outline-offset:2px}
.big button{min-height:72px;font-size:24px}
.field{margin-bottom:14px}
label{display:block;font-weight:700;margin-bottom:6px}
.stepper{display:grid;grid-template-columns:58px 1fr 58px;gap:8px}
input{width:100%;min-height:58px;border-radius:16px;border:2px solid var(--line);background:var(--soft);color:var(--ink);font:800 26px system-ui,sans-serif;text-align:center}
.save{width:100%;margin-top:4px;background:var(--pri);border-color:var(--pri);color:var(--btn-ink)}
#msg{min-height:22px;margin:0;font-size:15px;font-weight:600;color:var(--ok)}
#msg.err{color:var(--bad)}
#chart{width:100%;height:auto;display:block}
.log{list-style:none;margin:0;padding:0}
.log li{display:flex;justify-content:space-between;gap:12px;padding:9px 0;border-bottom:1px solid var(--line);font-size:15px}
.log li:last-child{border-bottom:0}
.log li span:last-child{color:var(--mute);white-space:nowrap}
details{color:var(--mute);font-size:14px}
summary{cursor:pointer;padding:6px 0}
.triple{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:8px}
.triple button{font-size:15px;min-height:54px;padding:8px 4px}
#windowstate{font-size:24px;font-weight:800;margin:8px 0}
input[type=range]{min-height:44px;padding:0;border:0;background:transparent;accent-color:var(--pri);font-size:16px}
.rangepair{display:grid;grid-template-columns:1fr 104px;gap:12px;align-items:center}
.rangepair input[type=number]{font-size:22px}
.diag li span:last-child{white-space:normal;text-align:right}
</style></head><body><main>
<header><h1>Ventilátor ESP32</h1><span id="conn" class="pill">Připojuji…</span></header>

<section class="hero">
 <div class="card">
  <h2>Teplota</h2>
  <div id="temp" class="temp stale">—</div>
  <span id="chip" class="chip">Čekám na data</span>
  <p id="tempnote" class="note"></p>
  <p class="note">DHT11 měří s rozlišením 1 °C a přesností zhruba ±2 °C.</p>
 </div>
 <div class="card fancard">
  <h2>Ventilátor</h2>
  <svg id="fanv" class="fanv" viewBox="-50 -50 100 100" aria-hidden="true">
   <g fill="currentColor"><ellipse cx="0" cy="-24" rx="10" ry="22"/><ellipse cx="0" cy="-24" rx="10" ry="22" transform="rotate(120)"/><ellipse cx="0" cy="-24" rx="10" ry="22" transform="rotate(240)"/></g>
   <circle r="7" fill="var(--card)" stroke="currentColor" stroke-width="3"/>
  </svg>
  <div id="fanstate" class="fanstate">—</div>
  <p class="note">Stav potvrzený ESP32 (povel pro relé). Skutečné otáčení motoru se neměří.</p>
 </div>
</section>

<section class="card">
 <h2>Vlhkost</h2>
 <div id="hum" class="hum stale">—</div>
 <span id="humchip" class="chip">Čekám na data</span>
 <p id="humnote" class="note"></p>
 <p class="note">DHT11 měří relativní vlhkost s rozlišením 1 % a přesností zhruba ±5 %. Vlhkost je jen informativní, ventilátor řídí teplota.</p>
</section>

<section class="card">
 <h2>Teplota za poslední hodinu</h2>
 <svg id="chart" viewBox="0 0 360 190" role="img" aria-label="Graf teploty a stavu ventilátoru"></svg>
 <p id="chartnote" class="note"></p>
</section>

<section class="card">
 <h2>Režim</h2>
 <div class="pair big"><button id="auto" type="button">AUTO</button><button id="manual" type="button">RUČNĚ</button></div>
 <p id="modenote" class="note"></p>
</section>

<section class="card">
 <h2>Ruční ovládání</h2>
 <div class="pair big"><button id="on" type="button">ON</button><button id="off" type="button">OFF</button></div>
 <p id="locknote" class="note"></p>
</section>

<section class="card">
 <h2>Okno místnosti</h2>
 <div id="windowstate" role="status">Čekám na data…</div>
 <p id="windowdetail" class="note"></p>
 <div class="triple">
  <button id="wfollow" type="button">S ventilátorem</button>
  <button id="wopen" type="button">Otevřít</button>
  <button id="wclose" type="button">Zavřít</button>
 </div>
 <p id="windownote" class="note"></p>
 <p class="note">Zobrazuje se povel pro servo, skutečná poloha okna se neměří. Ruční volba platí 10 minut, potom se obnoví režim „S ventilátorem“. Ruční volba okna nemění režim ani stav ventilátoru.</p>
</section>

<form id="wsettings" class="card" novalidate>
 <h2>Nastavení okna a serva</h2>
 <div class="field"><label for="wclosedn">Úhel zavření (°)</label>
  <div class="rangepair"><input id="wclosedr" type="range" min="0" max="180" step="1" aria-label="Úhel zavření, posuvník"><input id="wclosedn" type="number" inputmode="numeric" min="0" max="180" step="1"></div></div>
 <div class="field"><label for="wopenn">Úhel otevření (°)</label>
  <div class="rangepair"><input id="wopenr" type="range" min="0" max="180" step="1" aria-label="Úhel otevření, posuvník"><input id="wopenn" type="number" inputmode="numeric" min="0" max="180" step="1"></div></div>
 <div class="field"><label for="wspeedn">Rychlost pohybu (°/s)</label>
  <div class="rangepair"><input id="wspeedr" type="range" min="10" max="180" step="1" aria-label="Rychlost pohybu, posuvník"><input id="wspeedn" type="number" inputmode="numeric" min="10" max="180" step="1"></div></div>
 <p id="wrule" class="note"></p>
 <button id="wsave" class="save" type="submit">Uložit nastavení</button>
 <div class="pair" style="margin-top:12px"><button id="wtestopen" type="button">Test: otevřít</button><button id="wtestclose" type="button">Test: zavřít</button></div>
 <p class="note">Rozsah 0–180° je rozsah povelu, nikoli záruka skutečného rozsahu serva. Výchozí 30° (zavřeno) a 120° (otevřeno) je jen začátek, který se musí přizpůsobit skutečnému servu a mechanismu. Opačnou orientaci nastavíš prohozením úhlů (například zavřeno 120°, otevřeno 30°).</p>
 <p class="note">První test proveď bez táhla okna a nepoužívej polohy, v nichž servo tlačí do dorazu. Posuvníky servem nehýbou; nastavení se použije až tlačítkem „Uložit nastavení“ (okno se pak přesune do nové polohy podle aktuálního povelu). „Test“ vyšle neuložený úhel z formuláře jako ruční povel na 10 minut. Po restartu ESP32 neví, kde servo fyzicky je: první povel po startu servo vykoná plnou vlastní rychlostí z neznámé polohy, nastavená rychlost platí až od dalšího pohybu.</p>
</form>

<form id="settings" class="card" novalidate>
 <h2>Nastavení automatiky</h2>
 <div class="field"><label for="thr">Zapnout nad teplotou (°C)</label>
  <div class="stepper"><button type="button" data-t="thr" data-d="-1" aria-label="Snížit hranici">−</button><input id="thr" type="number" inputmode="numeric" step="1"><button type="button" data-t="thr" data-d="1" aria-label="Zvýšit hranici">+</button></div></div>
 <div class="field"><label for="hys">Rozdíl pro vypnutí (°C)</label>
  <div class="stepper"><button type="button" data-t="hys" data-d="-1" aria-label="Snížit rozdíl">−</button><input id="hys" type="number" inputmode="numeric" step="1"><button type="button" data-t="hys" data-d="1" aria-label="Zvýšit rozdíl">+</button></div></div>
 <p id="rule" class="note"></p>
 <button id="save" class="save" type="submit">Uložit nastavení</button>
</form>

<section class="card">
 <h2>Poslední události</h2>
 <ul id="log" class="log"><li><span>Načítám…</span><span></span></li></ul>
 <p class="note">Časy se počítají od spuštění ESP32. Po restartu se seznam vymaže.</p>
</section>

<p id="msg" role="status" aria-live="polite"></p>
<details><summary>Diagnostika</summary><ul id="diag" class="log diag"></ul></details>
</main>
<script>
(function(){
'use strict';
var $=function(id){return document.getElementById(id)};
var ERR={auto_mode:'V režimu AUTO nelze ventilátor ovládat ručně. Přepni na RUČNĚ.',bad_value:'Neplatná hodnota, zkontroluj povolený rozsah.',save_failed:'ESP32 nastavení nepodařilo uložit do paměti.',bad_request:'ESP32 požadavek nepochopilo.',servo_fault:'PWM serva není dostupné. Zkontroluj diagnostiku, ESP32 inicializaci opakuje.',bad_angle:'Úhel musí být celé číslo 0–180°.',bad_span:'Úhly zavření a otevření jsou příliš blízko u sebe.',bad_speed:'Rychlost musí být celé číslo v povoleném rozsahu °/s.',status_overflow:'ESP32 nedokázalo sestavit odpověď.'};
var EVT={0:'ESP32 spuštěno',1:'Přepnuto na AUTO',2:'Přepnuto na RUČNĚ',3:'Ventilátor zapnut ručně',4:'Ventilátor vypnut ručně',5:'Ventilátor zapnut automatikou',6:'Ventilátor vypnut automatikou',7:'Ventilátor vypnut: čidlo dlouho bez platného měření',8:'Ventilátor vypnut: uplynul limit ručního zapnutí',9:'Čidlo přestalo dodávat platná data',10:'Čidlo znovu měří',11:'Změněno nastavení automatiky',12:'Okno sleduje ventilátor',13:'Ruční povel otevřít okno',14:'Ruční povel zavřít okno',15:'Chyba PWM serva',16:'Ruční volba okna skončila, okno sleduje ventilátor',17:'Servo: první povel po startu vydán',18:'PWM serva znovu funguje',19:'Změněno nastavení okna',20:'Zkušební povel okna (neuložený úhel)'};
var S=null,recvAt=0,lastOk=0,seq=0,applied=0,dirty=false,wdirty=false,pending=false,limitsSet=false,pollMs=3000,msgTimer=0;
var LOGD=null,logAt=0,logSeen=-1,logTry=0,logBusy=false,HIST=null,histSeen=-1,histTry=0,histBusy=false,chartKey='';

function fmtAge(s){s=Math.round(s);if(s<60)return s+' s';return Math.floor(s/60)+' min '+(s%60)+' s';}
function errText(e){
 if(e&&e.code==='bad_span'&&S&&S.servoLimits)return 'Úhly zavření a otevření se musí lišit alespoň o '+S.servoLimits.span+'°.';
 if(e&&e.code==='bad_speed'&&S&&S.servoLimits)return 'Rychlost musí být celé číslo '+S.servoLimits.sMin+'–'+S.servoLimits.sMax+' °/s.';
 if(e&&e.code&&ERR[e.code])return ERR[e.code];
 if(e&&e.name==='AbortError')return 'ESP32 neodpovědělo včas. Změna nebyla potvrzena.';
 return 'Nelze se spojit s ESP32 (Wi-Fi Web_Control_Fan). Změna nebyla potvrzena.';
}
function fmtClock(s){s=Math.max(0,Math.round(s));var r=s%60;return Math.floor(s/60)+':'+(r<10?'0':'')+r;}
function fmtAgo(s){return s<2?'právě teď':'před '+fmtAge(s);}
function getRaw(path){
 var ctl=new AbortController(),t=setTimeout(function(){ctl.abort();},4000);
 return fetch(path,{cache:'no-store',signal:ctl.signal}).then(function(r){return r.json();}).then(function(d){
  clearTimeout(t);if(!d.success)throw new Error('x');return d;
 },function(e){clearTimeout(t);throw e;});
}
function syncExtras(){
 if(!S)return;
 var now=Date.now(),want;
 if(S.logSeq!==logSeen&&!logBusy&&now-logTry>3000){
  logBusy=true;logTry=now;want=S.logSeq;
  getRaw('/api/log').then(function(d){LOGD=d.events;logAt=Date.now();logSeen=want;},function(){}).then(function(){logBusy=false;});
 }
 if(S.histSeq!==histSeen&&!histBusy&&now-histTry>3000){
  histBusy=true;histTry=now;want=S.histSeq;
  getRaw('/api/history').then(function(d){HIST=d;histSeen=want;chartKey='';},function(){}).then(function(){histBusy=false;});
 }
}
function drawChart(){
 var svg=$('chart'),W=360,H=190,L=34,R=8,T=10,B=26,pw=W-L-R,ph=H-T-B,out='';
 var n=HIST?HIST.temps.length:0;
 if(n<2){
  svg.innerHTML='<text x="'+(W/2)+'" y="'+(H/2)+'" text-anchor="middle" font-size="12" style="fill:var(--mute)">Zatím málo dat, bod přibývá každých '+(HIST?HIST.stepSec:30)+' s.</text>';
  $('chartnote').textContent='';return;
 }
 var cap=HIST.cap,thr=S.threshold,off=S.threshold-S.hysteresis,vals=[thr,off],i;
 for(i=0;i<n;i++)if(HIST.temps[i]!==null)vals.push(HIST.temps[i]);
 var lo=Math.floor(Math.min.apply(null,vals))-1,hi=Math.ceil(Math.max.apply(null,vals))+1;
 if(hi-lo<6){lo=Math.floor((hi+lo)/2-3);hi=lo+6;}
 var dx=pw/(cap-1);
 function X(k){return L+pw-(n-1-k)*dx;}
 function Y(v){return T+ph-(v-lo)/(hi-lo)*ph;}
 // zelené pozadí = povel ventilátor zapnutý
 i=0;
 while(i<n){
  if(HIST.fan.charAt(i)==='1'){var j=i;while(j+1<n&&HIST.fan.charAt(j+1)==='1')j++;
   out+='<rect x="'+(X(i)-dx/2).toFixed(1)+'" y="'+T+'" width="'+((j-i+1)*dx).toFixed(1)+'" height="'+ph+'" style="fill:var(--ok);opacity:.18"/>';i=j+1;}
  else i++;
 }
 out+='<line x1="'+L+'" y1="'+(T+ph)+'" x2="'+(W-R)+'" y2="'+(T+ph)+'" style="stroke:var(--line)" stroke-width="1"/>';
 out+='<line x1="'+L+'" y1="'+T+'" x2="'+L+'" y2="'+(T+ph)+'" style="stroke:var(--line)" stroke-width="1"/>';
 [[thr,'zap'],[off,'vyp']].forEach(function(l){
  out+='<line x1="'+L+'" y1="'+Y(l[0]).toFixed(1)+'" x2="'+(W-R)+'" y2="'+Y(l[0]).toFixed(1)+'" style="stroke:var(--mute)" stroke-width="1" stroke-dasharray="4 3"/>';
 });
 // čára teploty, přerušená tam, kde chybí platné měření
 var seg=[];
 function flush(){
  if(seg.length>1)out+='<polyline fill="none" stroke-width="2" stroke-linejoin="round" style="stroke:var(--pri)" points="'+seg.join(' ')+'"/>';
  else if(seg.length===1){var c=seg[0].split(',');out+='<circle cx="'+c[0]+'" cy="'+c[1]+'" r="2" style="fill:var(--pri)"/>';}
  seg=[];
 }
 for(i=0;i<n;i++){if(HIST.temps[i]===null)flush();else seg.push(X(i).toFixed(1)+','+Y(HIST.temps[i]).toFixed(1));}
 flush();
 out+='<text x="'+(L-4)+'" y="'+(T+8)+'" text-anchor="end" font-size="10" style="fill:var(--mute)">'+hi+'</text>';
 out+='<text x="'+(L-4)+'" y="'+(T+ph)+'" text-anchor="end" font-size="10" style="fill:var(--mute)">'+lo+'</text>';
 out+='<text x="'+L+'" y="'+(H-8)+'" font-size="10" style="fill:var(--mute)">před '+Math.round(cap*HIST.stepSec/60)+' min</text>';
 out+='<text x="'+(W-R)+'" y="'+(H-8)+'" text-anchor="end" font-size="10" style="fill:var(--mute)">teď</text>';
 svg.innerHTML=out;
 $('chartnote').textContent='Jeden bod = '+HIST.stepSec+' s, osa v °C. Horní přerušovaná čára = hranice zapnutí ('+thr+' °C), dolní = vypnutí ('+off+' °C). Zelené pozadí = povel pro ventilátor zapnut. Mezera v čáře = čidlo nedodávalo platná data.';
}
function drawLog(now){
 var ul=$('log');
 if(!LOGD)return;
 while(ul.firstChild)ul.removeChild(ul.firstChild);
 LOGD.forEach(function(ev){
  var li=document.createElement('li'),a=document.createElement('span'),b=document.createElement('span');
  var txt=EVT[ev.e]||'Událost '+ev.e;
  if(ev.e===8&&S)txt='Ventilátor vypnut: uplynul limit '+Math.round(S.manualLimitSec/60)+' min ručního zapnutí';
  a.textContent=txt;b.textContent=fmtAgo(ev.a+(now-logAt)/1000);
  li.appendChild(a);li.appendChild(b);ul.appendChild(li);
 });
}
function setMsg(t,isErr){
 var m=$('msg');m.textContent=t;m.className=isErr?'err':'';
 clearTimeout(msgTimer);if(!isErr&&t)msgTimer=setTimeout(function(){m.textContent='';},4000);
}
function api(path,body){
 var id=++seq,ctl=new AbortController(),t=setTimeout(function(){ctl.abort();},4000);
 var opt={cache:'no-store',signal:ctl.signal};
 if(body!==undefined){opt.method='POST';opt.headers={'Content-Type':'application/x-www-form-urlencoded'};opt.body=body;}
 return fetch(path,opt).then(function(r){
  return r.json().then(function(d){return {r:r,d:d};},function(){return {r:r,d:null};});
 }).then(function(x){
  clearTimeout(t);
  if(!x.d||!x.r.ok||!x.d.success){var e=new Error('http');e.code=x.d&&x.d.error;throw e;}
  lastOk=Date.now();
  if(id>applied){applied=id;S=x.d;recvAt=lastOk;pollMs=Math.max(2000,S.pollMs||3000);}
  return x.d;
 },function(e){clearTimeout(t);throw e;});
}
function cmd(path,body,okText,onOk){
 if(pending)return;
 pending=true;setMsg('',false);render();
 api(path,body).then(function(){if(onOk)onOk();setMsg(okText,false);},function(e){setMsg(errText(e),true);}).then(function(){pending=false;render();});
}
function formVals(){
 var t=parseInt($('thr').value,10),h=parseInt($('hys').value,10);
 var ok=S&&/^-?\d+$/.test($('thr').value)&&/^-?\d+$/.test($('hys').value)&&t>=S.limits.tMin&&t<=S.limits.tMax&&h>=S.limits.hMin&&h<=S.limits.hMax&&t-h>=0;
 return {t:t,h:h,ok:!!ok};
}
var FANR={0:'start ESP32 (ventilátor vypnutý)',1:'teplota nad hranicí zapnutí',2:'teplota na hranici vypnutí nebo níže',3:'ruční ON',4:'ruční OFF',5:'přepnutí na RUČNĚ (ventilátor vypnut)',6:'uplynul časový limit ručního běhu',7:'chyba čidla (bezpečnostní vypnutí)'};
var RST={POWERON:'zapnutí napájení',BROWNOUT:'propad napájecího napětí (brownout)',SW:'softwarový restart',PANIC:'pád programu (panic)',INT_WDT:'watchdog (přerušení)',TASK_WDT:'watchdog (úloha)',WDT:'watchdog',EXT:'vnější reset',DEEPSLEEP:'probuzení ze spánku',SDIO:'reset SDIO',UNKNOWN:'neznámý důvod'};
function fmtUp(s){s=Math.max(0,Math.floor(s));var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),m=Math.floor(s%3600/60);return (d?d+' d ':'')+(h||d?h+' h ':'')+m+' min '+(s%60)+' s';}
function wVals(){
 var L=S&&S.servoLimits;
 function iv(id){var v=$(id).value;return /^\d+$/.test(v)?parseInt(v,10):NaN;}
 var c=iv('wclosedn'),o=iv('wopenn'),s=iv('wspeedn');
 function okA(a){return !isNaN(a)&&!!L&&a>=L.aMin&&a<=L.aMax;}
 var okC=okA(c),okO=okA(o),okS=!isNaN(s)&&!!L&&s>=L.sMin&&s<=L.sMax;
 var okSpan=okC&&okO&&Math.abs(o-c)>=L.span;
 return {c:c,o:o,s:s,okC:okC,okO:okO,okS:okS,okSpan:okSpan,ok:okC&&okO&&okS&&okSpan};
}
function drawDiag(now,conn,fresh,age,hFresh,hAge){
 var rows=[],el=(now-recvAt)/1000,ul=$('diag');
 if(!conn)rows.push(['Spojení','Bez spojení s ESP32, údaje nejsou aktuální.']);
 else{
  rows.push(['Doba běhu od restartu',fmtUp(S.uptimeSec+el)]);
  rows.push(['Důvod posledního restartu',(RST[S.resetReason]||S.resetReason)+' ('+S.resetReason+')']);
  rows.push(['Teplota',S.lastTemp===null?'dosud žádné platné měření':S.lastTemp+' °C, stáří '+fmtAge(age)+(fresh?' (aktuální)':' (ZASTARALÉ)')+', chybných čtení v řadě: '+S.failStreak]);
  rows.push(['Vlhkost',S.lastHum===null?'dosud žádné platné měření':S.lastHum+' %, stáří '+fmtAge(hAge)+(hFresh?' (aktuální)':' (ZASTARALÉ)')+', chybných čtení v řadě: '+S.humFail]);
  rows.push(['Poslední změna ventilátoru',(FANR[S.fanReason]||'?')+', '+fmtAgo(S.fanReasonAgeSec+el)]);
  rows.push(['Servo GPIO33',!S.servoOk?'CHYBA řízení PWM, ESP32 zkouší znovu; web, měření i ventilátor běží dál':(S.servoStarted?'PWM v pořádku, první povel po startu vydán':'PWM v pořádku, čeká na první povel po startu')]);
  rows.push(['Povel serva',S.servoOk&&S.servoStarted?S.windowAngle+'° ('+S.windowCmdUs+' µs), cíl '+S.windowTarget+'°, bez zpětné vazby':'zatím nevydán']);
  rows.push(['Relé GPIO27','povel '+S.pinLevel+' ('+(S.fan==='on'?'zapnuto':'vypnuto')+'), aktivní při '+(S.activeHigh?'HIGH':'LOW')]);
  rows.push(['Volná paměť RAM',Math.round(S.heapFree/1024)+' kB, minimum od startu '+Math.round(S.heapMin/1024)+' kB']);
  rows.push(['Nejdelší pauza hlavní smyčky',S.loopMaxMs+' ms od startu']);
  rows.push(['Měření čidla','každé '+S.intervalSec+' s']);
  rows.push(['OLED displej',S.oled?'připojen':'nepřipojen nebo nedostupný']);
  rows.push(['Paměť nastavení',S.storage?'v pořádku':'nedostupná']);
 }
 while(ul.firstChild)ul.removeChild(ul.firstChild);
 rows.forEach(function(r){var li=document.createElement('li'),a=document.createElement('span'),b=document.createElement('span');a.textContent=r[0];b.textContent=r[1];li.appendChild(a);li.appendChild(b);ul.appendChild(li);});
}
function render(){
 var now=Date.now(),conn=!!S&&(now-lastOk<8000);
 $('conn').textContent=!S?'Připojuji…':(conn?'Připojeno k ESP32':'Bez spojení');
 $('conn').className='pill '+(conn?'ok':(S?'bad':''));
 if(!S)return;
 if(!limitsSet){
  $('thr').min=S.limits.tMin;$('thr').max=S.limits.tMax;$('hys').min=S.limits.hMin;$('hys').max=S.limits.hMax;
  $('thr').value=S.threshold;$('hys').value=S.hysteresis;
  var SL=S.servoLimits;
  [['wclosedr','wclosedn',SL.aMin,SL.aMax],['wopenr','wopenn',SL.aMin,SL.aMax],['wspeedr','wspeedn',SL.sMin,SL.sMax]].forEach(function(a){
   $(a[0]).min=a[2];$(a[0]).max=a[3];$(a[1]).min=a[2];$(a[1]).max=a[3];
  });
  limitsSet=true;
 }
 var auto=S.mode==='auto',on=S.fan==='on';
 var age=S.ageSec===null?null:S.ageSec+(now-recvAt)/1000;
 var fresh=conn&&age!==null&&age<=S.staleSec;

 // teplota a čidlo
 $('temp').textContent=fresh?S.lastTemp+' °C':'—';
 $('temp').className='temp'+(fresh?'':' stale');
 var cls,txt,note;
 if(!conn){cls='bad';txt='Stav čidla neznámý';note='Spojení s ESP32 je přerušeno, teplota není aktuální.';}
 else if(fresh){cls='ok';txt='Čidlo DHT11 v pořádku';note='Změřeno před '+fmtAge(age)+'.';}
 else if(S.sensor==='waiting'){cls='warn';txt='Čekám na první měření';note='Čidlo se po startu ustaluje.';}
 else{cls='bad';txt='Čidlo DHT11 neodpovídá';note=S.lastTemp!==null?'Měření je zastaralé. Poslední platná hodnota byla '+S.lastTemp+' °C před '+fmtAge(age)+'.':'Zatím není žádné platné měření. Zkontroluj zapojení čidla.';}
 $('chip').textContent=txt;$('chip').className='chip '+cls;$('tempnote').textContent=note;

 // vlhkost (vlastní stáří a platnost, jen informativní)
 var hAge=S.humAgeSec===null?null:S.humAgeSec+(now-recvAt)/1000;
 var hFresh=conn&&hAge!==null&&hAge<=S.staleSec;
 $('hum').textContent=hFresh?S.lastHum+' %':'—';
 $('hum').className='hum'+(hFresh?'':' stale');
 var hc,ht,hn;
 if(!conn){hc='bad';ht='Stav vlhkosti neznámý';hn='Spojení s ESP32 je přerušeno, vlhkost není aktuální.';}
 else if(hFresh){hc='ok';ht='Vlhkost změřena';hn='Změřeno před '+fmtAge(hAge)+'.';}
 else if(S.lastHum===null){
  if(S.humFail>0){hc='bad';ht='Vlhkost nelze změřit';hn='Zatím není žádné platné měření vlhkosti.';}
  else{hc='warn';ht='Čekám na první měření';hn='';}
 }
 else{hc='bad';ht='Měření vlhkosti je zastaralé';hn='Poslední platná hodnota byla '+S.lastHum+' % před '+fmtAge(hAge)+'.';}
 $('humchip').textContent=ht;$('humchip').className='chip '+hc;$('humnote').textContent=hn;

 // ventilátor
 $('fanstate').textContent=conn?(on?'ZAPNUTO':'VYPNUTO'):'NEZNÁMÝ STAV';
 $('fanv').setAttribute('class','fanv'+(conn&&on?' on spin':''));

 // režim
 $('auto').className=auto?'active':'';$('manual').className=auto?'':'active';
 $('auto').setAttribute('aria-pressed',auto);$('manual').setAttribute('aria-pressed',!auto);
 var off=S.threshold-S.hysteresis,mt;
 if(!auto)mt='RUČNĚ: ventilátor ovládáš jen ty, automatika do něj nezasahuje.';
 else if(!conn)mt='AUTO: bez spojení nelze stav ověřit. ESP32 řídí ventilátor samo.';
 else if(fresh)mt='AUTO: zapne nad '+S.threshold+' °C, vypne při '+off+' °C nebo níže, mezi tím drží stav.';
 else if(S.sensor==='waiting')mt='AUTO: čekám na první měření, ventilátor je vypnutý.';
 else if(age!==null&&age<=S.failsafeSec)mt='AUTO: měření je zastaralé, ventilátor drží poslední stav. Po '+S.failsafeSec+' s bez platného měření se vypne.';
 else mt='AUTO: bez platného měření déle než '+S.failsafeSec+' s, ventilátor je z bezpečnostních důvodů vypnutý.';
 $('modenote').textContent=mt;

 // ruční tlačítka
 var lock=!conn||auto||pending;
 $('on').disabled=lock;$('off').disabled=lock;
 $('on').className=(!auto&&on)?'active':'';$('off').className=(!auto&&!on)?'active':'';
 $('auto').disabled=!conn||pending;$('manual').disabled=!conn||pending;
 var lim=Math.round(S.manualLimitSec/60),ln;
 if(auto)ln='Tlačítka jsou zamčená, protože je zapnutý režim AUTO. Přepni na RUČNĚ.';
 else if(!conn)ln='Bez spojení s ESP32 nelze ovládat.';
 else if(on&&S.manualLeftSec!==null)ln='Ručně zapnutý ventilátor se po '+lim+' min sám vypne. Zbývá '+fmtClock(S.manualLeftSec-(now-recvAt)/1000)+'.';
 else ln='Ruční ovládání je odemčeno. Ručně zapnutý ventilátor se po '+lim+' min sám vypne.';
 $('locknote').textContent=ln;

 // okno: cíl a vydávaný povel servu, nikoli změřená poloha
 var wm=S.windowMode,wt=S.windowOpen,wsT,wd='';
 if(!conn)wsT='Stav neznámý';
 else if(!S.servoOk)wsT='Chyba PWM serva';
 else if(!S.servoStarted)wsT='Zatím bez povelu';
 else wsT='Povel: '+(wt?'OTEVŘÍT':'ZAVŘÍT');
 if(conn&&S.servoOk&&S.servoStarted)wd='Cílový úhel '+S.windowTarget+'°, aktuálně vydávaný povel '+S.windowAngle+'° ('+S.windowCmdUs+' µs)'+(S.windowMoving?', povel se mění rychlostí '+S.servoSpeed+' °/s.':', povel dosáhl cíle.');
 else if(conn&&S.servoOk)wd='Cílový úhel '+S.windowTarget+'°. První povel po startu servo dostane do několika sekund od zapnutí.';
 $('windowstate').textContent=wsT;$('windowdetail').textContent=wd;
 ['wfollow','wopen','wclose'].forEach(function(id,i){
  var b=$(id),active=wm===['follow','open','closed'][i];
  b.className=active?'active':'';b.setAttribute('aria-pressed',active);
  b.disabled=!conn||pending||(!S.servoOk&&i!==0);
 });
 var wn;
 if(!conn)wn='Bez spojení s ESP32 nelze ověřit stav ani ovládat okno.';
 else if(!S.servoOk)wn='ESP32 nedokáže vysílat PWM servu, fyzická poloha okna je neznámá. Ventilátor, měření i web fungují dál.';
 else{
  wn=wm==='follow'?'Režim „S ventilátorem“: okno sleduje výsledný stav ventilátoru v AUTO i RUČNĚ, včetně bezpečnostního vypnutí.':
   'Ruční volba: '+(S.windowTest?'zkušební úhel '+S.windowTarget+'°':(wm==='open'?'otevřít':'zavřít'))+'. Návrat k režimu „S ventilátorem“ za '+fmtClock(S.windowLeftSec-(now-recvAt)/1000)+'.';
  if(wm==='closed'&&on)wn+=' Pozor: ventilátor má povel ON a okno je ručně zavřené.';
  else if(wm!=='follow'&&auto&&!fresh)wn+=' Ruční volba okna platí i při chybě čidla.';
 }
 $('windownote').textContent=wn;

 // formulář – hodnoty se přepíšou jen když je uživatel právě needituje
 var ae=document.activeElement;
 if(!dirty&&ae!==$('thr')&&ae!==$('hys')){$('thr').value=S.threshold;$('hys').value=S.hysteresis;}
 var f=formVals();
 $('rule').textContent=dirty
  ?(f.ok?'Neuloženo: zapne nad '+f.t+' °C, vypne při '+(f.t-f.h)+' °C nebo níže.':'Neuloženo: hodnoty jsou mimo povolený rozsah ('+S.limits.tMin+'–'+S.limits.tMax+' °C, rozdíl '+S.limits.hMin+'–'+S.limits.hMax+' °C).')
  :'Uloženo v ESP32: zapne nad '+S.threshold+' °C, vypne při '+off+' °C nebo níže.';
 $('save').disabled=!conn||pending||!dirty;

 // nastavení okna: pole se přepíšou jen když je uživatel právě needituje
 var SL2=S.servoLimits,wae=document.activeElement,wids=['wclosedr','wclosedn','wopenr','wopenn','wspeedr','wspeedn'];
 if(!wdirty&&!(wae&&wids.indexOf(wae.id)>=0)){
  $('wclosedr').value=$('wclosedn').value=S.windowClosedDeg;
  $('wopenr').value=$('wopenn').value=S.windowOpenDeg;
  $('wspeedr').value=$('wspeedn').value=S.servoSpeed;
 }
 var wf=wVals(),wr;
 if(!conn)wr='Bez spojení s ESP32: nastavení nelze ověřit ani uložit.';
 else if(!wdirty)wr='Uloženo v ESP32: zavřeno '+S.windowClosedDeg+'°, otevřeno '+S.windowOpenDeg+'°, rychlost '+S.servoSpeed+' °/s.';
 else if(wf.ok)wr='Neuloženo: zavřeno '+wf.c+'°, otevřeno '+wf.o+'°, rychlost '+wf.s+' °/s. Servo se nepohne, dokud nestiskneš „Uložit nastavení“.';
 else if(!wf.okC||!wf.okO)wr='Neuloženo: úhly musí být celá čísla '+SL2.aMin+'–'+SL2.aMax+'°.';
 else if(!wf.okSpan)wr='Neuloženo: úhly zavření a otevření se musí lišit alespoň o '+SL2.span+'°.';
 else wr='Neuloženo: rychlost musí být celé číslo '+SL2.sMin+'–'+SL2.sMax+' °/s.';
 $('wrule').textContent=wr;
 $('wsave').disabled=!conn||pending||!wdirty||!wf.ok;
 $('wtestopen').disabled=!conn||pending||!S.servoOk||!wf.okO;
 $('wtestclose').disabled=!conn||pending||!S.servoOk||!wf.okC;

 syncExtras();
 drawLog(now);
 var ck=histSeen+'|'+S.threshold+'|'+S.hysteresis;
 if(ck!==chartKey&&HIST){chartKey=ck;drawChart();}
 else if(!HIST&&chartKey===''){chartKey='-';drawChart();}
 drawDiag(now,conn,fresh,age,hFresh,hAge);
}
function poll(){api('/api/status').then(function(){},function(){}).then(function(){render();setTimeout(poll,pollMs);});}

$('wfollow').onclick=function(){cmd('/api/window','mode=follow','Okno nyní sleduje ventilátor.');};
$('wopen').onclick=function(){cmd('/api/window','mode=open','ESP32 přijalo povel otevřít okno na 10 minut.');};
$('wclose').onclick=function(){cmd('/api/window','mode=closed','ESP32 přijalo povel zavřít okno na 10 minut.');};
$('wtestopen').onclick=function(){var v=wVals();if(!v.okO){setMsg('Úhel otevření musí být celé číslo 0–180°.',true);return;}cmd('/api/window','mode=open&deg='+v.o,'ESP32 přijalo zkušební povel otevřít ('+v.o+'°, neuloženo).');};
$('wtestclose').onclick=function(){var v=wVals();if(!v.okC){setMsg('Úhel zavření musí být celé číslo 0–180°.',true);return;}cmd('/api/window','mode=closed&deg='+v.c,'ESP32 přijalo zkušební povel zavřít ('+v.c+'°, neuloženo).');};
[['wclosedr','wclosedn'],['wopenr','wopenn'],['wspeedr','wspeedn']].forEach(function(p){
 $(p[0]).addEventListener('input',function(){$(p[1]).value=$(p[0]).value;wdirty=true;render();});
 $(p[1]).addEventListener('input',function(){var v=$(p[1]).value;if(/^\d+$/.test(v))$(p[0]).value=v;wdirty=true;render();});
});
$('wsettings').addEventListener('submit',function(e){
 e.preventDefault();
 var v=wVals();
 if(!v.ok){setMsg('Hodnoty jsou mimo povolený rozsah, nic se neuložilo.',true);return;}
 cmd('/api/winsettings','closed='+v.c+'&open='+v.o+'&speed='+v.s,'Nastavení okna uloženo.',function(){wdirty=false;});
});
$('auto').onclick=function(){cmd('/api/mode','mode=auto','Režim AUTO zapnut.');};
$('manual').onclick=function(){cmd('/api/mode','mode=manual','Režim RUČNĚ zapnut, ventilátor je vypnutý.');};
$('on').onclick=function(){cmd('/api/fan','state=on','ESP32 potvrdilo: ON.');};
$('off').onclick=function(){cmd('/api/fan','state=off','ESP32 potvrdilo: OFF.');};
['thr','hys'].forEach(function(id){$(id).addEventListener('input',function(){dirty=true;render();});});
Array.prototype.forEach.call(document.querySelectorAll('.stepper button'),function(b){
 b.onclick=function(){
  var inp=$(b.getAttribute('data-t')),v=parseInt(inp.value,10);
  if(isNaN(v))v=parseInt(inp.min,10);
  v=Math.min(parseInt(inp.max,10),Math.max(parseInt(inp.min,10),v+parseInt(b.getAttribute('data-d'),10)));
  inp.value=v;dirty=true;render();
 };
});
$('settings').addEventListener('submit',function(e){
 e.preventDefault();
 var f=formVals();
 if(!f.ok){setMsg('Hodnoty jsou mimo povolený rozsah, nic se neuložilo.',true);return;}
 cmd('/api/settings','threshold='+f.t+'&hysteresis='+f.h,'Nastavení uloženo.',function(){dirty=false;});
});
setInterval(render,1000);
poll();
})();
</script></body></html>
)rawliteral";

void sendJson(int code, const char* body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json; charset=utf-8", body);
}

void sendError(int code, const char* errCode) {
  char buf[80];
  snprintf(buf, sizeof(buf), "{\"success\":false,\"error\":\"%s\"}", errCode);
  Serial.printf("[HTTP] odmitnuto: %s\n", errCode);
  sendJson(code, buf);
}

void sendStatus() {
  uint32_t now = millis();
  uint32_t age = now - lastGoodMs;
  SensorState ss = sensorStateOf(haveGood, age, failStreak);
  const char* sensorTxt = ss == SENSOR_OK ? "ok" : ss == SENSOR_STALE ? "stale"
                        : ss == SENSOR_ERROR ? "error" : "waiting";
  char lastS[8] = "null", ageS[12] = "null";
  if (haveGood) {
    snprintf(lastS, sizeof(lastS), "%d", lastTempC);
    snprintf(ageS, sizeof(ageS), "%lu", (unsigned long)(age / 1000));
  }
  char humS[8] = "null", humAgeS[12] = "null";
  if (haveHum) {
    snprintf(humS, sizeof(humS), "%d", lastHumPct);
    snprintf(humAgeS, sizeof(humAgeS), "%lu", (unsigned long)((now - lastHumMs) / 1000));
  }
  bool level = (motorOn == RELAY_ACTIVE_HIGH);
  char leftS[12] = "null";
  if (!autoMode && motorOn)
    snprintf(leftS, sizeof(leftS), "%lu", (unsigned long)((manualLeftMs(now - manualOnSinceMs) + 999) / 1000));
  static char buf[2560];
  int n = snprintf(buf, sizeof(buf),
    "{\"success\":true,\"mode\":\"%s\",\"fan\":\"%s\",\"pinLevel\":\"%s\",\"activeHigh\":%s,"
    "\"sensor\":\"%s\",\"lastTemp\":%s,\"ageSec\":%s,\"failStreak\":%u,"
    "\"lastHum\":%s,\"humAgeSec\":%s,\"humFail\":%u,\"oled\":%s,"
    "\"threshold\":%d,\"hysteresis\":%d,"
    "\"limits\":{\"tMin\":%d,\"tMax\":%d,\"hMin\":%d,\"hMax\":%d},"
    "\"staleSec\":%lu,\"failsafeSec\":%lu,\"pollMs\":%lu,\"intervalSec\":%lu,\"storage\":%s,"
    "\"manualLeftSec\":%s,\"manualLimitSec\":%lu,\"logSeq\":%lu,\"histSeq\":%lu,"
    "\"servoOk\":%s,\"windowMode\":\"%s\",\"windowOpen\":%s,\"windowMoving\":%s,"
    "\"windowAngle\":%d,\"windowTarget\":%d,\"windowLeftSec\":%lu,\"uptimeSec\":%lu",
    autoMode ? "auto" : "manual", motorOn ? "on" : "off", level ? "HIGH" : "LOW",
    RELAY_ACTIVE_HIGH ? "true" : "false", sensorTxt, lastS, ageS, failStreak,
    humS, humAgeS, humFail, (OLED_ENABLED && oledOk) ? "true" : "false",
    thresholdC, hysteresisC, THRESHOLD_MIN, THRESHOLD_MAX, HYST_MIN, HYST_MAX,
    (unsigned long)(SENSOR_STALE_MS / 1000), (unsigned long)(SENSOR_FAILSAFE_MS / 1000),
    (unsigned long)WEB_POLL_MS, (unsigned long)(SENSOR_INTERVAL_MS / 1000),
    prefsReady ? "true" : "false", leftS, (unsigned long)(MANUAL_LIMIT_MS / 1000),
    (unsigned long)logSeq, (unsigned long)histSeq,
    servoReady ? "true" : "false", windowMode == 0 ? "follow" : (windowMode == 1 ? "open" : "closed"),
    windowTargetOpen() ? "true" : "false",
    (servoReady && servoStarted && servoPos != (float)windowDesiredAngle()) ? "true" : "false",
    (int)lroundf(servoPos), windowDesiredAngle(), (unsigned long)((windowRemainingMs(now) + 999) / 1000),
    (unsigned long)(now / 1000));
  if (n < 0 || n >= (int)sizeof(buf)) { sendError(500, "status_overflow"); return; }
  int m = snprintf(buf + n, sizeof(buf) - n,
    ",\"servoStarted\":%s,\"windowTest\":%s,\"windowClosedDeg\":%d,\"windowOpenDeg\":%d,\"servoSpeed\":%d,"
    "\"windowCmdUs\":%lu,\"servoDelayMs\":%lu,"
    "\"servoLimits\":{\"aMin\":%d,\"aMax\":%d,\"sMin\":%d,\"sMax\":%d,\"span\":%d},"
    "\"fanReason\":%u,\"fanReasonAgeSec\":%lu,\"resetReason\":\"%s\","
    "\"heapFree\":%lu,\"heapMin\":%lu,\"loopMaxMs\":%lu}",
    servoStarted ? "true" : "false", (windowMode != 0 && windowTestDeg >= 0) ? "true" : "false",
    windowClosedDeg, windowOpenDeg, servoSpeedDps,
    (unsigned long)servoCmdUs, (unsigned long)SERVO_AFTER_RELAY_MS,
    SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_SPEED_MIN, SERVO_SPEED_MAX, SERVO_MIN_SPAN_DEG,
    (unsigned)fanReason, (unsigned long)((now - fanReasonMs) / 1000), resetReasonStr,
    (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)loopMaxGapMs);
  if (m < 0 || n + m >= (int)sizeof(buf)) { sendError(500, "status_overflow"); return; }
  sendJson(200, buf);
}

void handleLog() {  // nejnovější událost první, "a" = stáří v sekundách, "e" = kód
  uint32_t now = millis();
  String j;
  j.reserve(700);
  j = "{\"success\":true,\"events\":[";
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t idx = (logHead + LOG_SIZE - 1 - i) % LOG_SIZE;
    if (i) j += ',';
    j += "{\"a\":";
    j += (unsigned long)((now - logBuf[idx].ms) / 1000);
    j += ",\"e\":";
    j += (int)logBuf[idx].code;
    j += '}';
  }
  j += "]}";
  sendJson(200, j.c_str());
}

void handleHistory() {  // nejstarší bod první; null = bez platného měření
  uint32_t now = millis();
  String j;
  j.reserve(1000);
  j = "{\"success\":true,\"cap\":";
  j += (int)HISTORY_POINTS;
  j += ",\"stepSec\":";
  j += (unsigned long)(HISTORY_STEP_MS / 1000);
  j += ",\"ageSec\":";
  j += (unsigned long)((now - lastHistMs) / 1000);
  j += ",\"temps\":[";
  for (uint16_t i = 0; i < histCount; i++) {
    uint16_t idx = (histHead + HISTORY_POINTS - histCount + i) % HISTORY_POINTS;
    if (i) j += ',';
    if (histTemp[idx] == HIST_INVALID) j += "null";
    else j += (int)histTemp[idx];
  }
  j += "],\"fan\":\"";
  for (uint16_t i = 0; i < histCount; i++) {
    uint16_t idx = (histHead + HISTORY_POINTS - histCount + i) % HISTORY_POINTS;
    j += histFan[idx] ? '1' : '0';
  }
  j += "\"}";
  sendJson(200, j.c_str());
}

bool intArg(const char* name, int& out) {
  if (!server.hasArg(name)) return false;
  String s = server.arg(name);
  const char* p = s.c_str();
  char* end = nullptr;
  long v = strtol(p, &end, 10);
  if (end == p || *end != '\0' || v < -1000 || v > 1000) return false;
  out = (int)v;
  return true;
}

void handleMode() {
  String m = server.arg("mode");
  Serial.printf("[HTTP] POST /api/mode mode=%s\n", m.c_str());
  if (m != "auto" && m != "manual") { sendError(400, "bad_request"); return; }
  bool wantAuto = (m == "auto");
  if (wantAuto != autoMode) {
    autoMode = wantAuto;
    logEvent(autoMode ? EV_MODE_AUTO : EV_MODE_MANUAL);
    if (autoMode) applyAutomaticControl("prepnuti na AUTO");
    else setMotor(false, "prepnuti na RUCNE", EV_FAN_OFF_MANUAL, FR_MODE_MANUAL);
    Serial.printf("[MODE] %s\n", autoMode ? "AUTO" : "RUCNE");
  }
  sendStatus();
}

void handleFan() {
  String s = server.arg("state");
  Serial.printf("[HTTP] POST /api/fan state=%s (rezim %s)\n", s.c_str(), autoMode ? "AUTO" : "RUCNE");
  if (autoMode) { sendError(409, "auto_mode"); return; }
  if (s != "on" && s != "off") { sendError(400, "bad_request"); return; }
  bool on = (s == "on");
  if (on) manualOnSinceMs = millis();  // každé ON spustí odpočet limitu znovu
  setMotor(on, "rucne z webu", on ? EV_FAN_ON_MANUAL : EV_FAN_OFF_MANUAL,
           on ? FR_MANUAL_ON : FR_MANUAL_OFF);
  sendStatus();
}

void handleSettings() {
  int thr, hy;
  if (!intArg("threshold", thr) || !intArg("hysteresis", hy) || !settingsValid(thr, hy)) {
    sendError(400, "bad_value"); return;
  }
  Serial.printf("[HTTP] POST /api/settings threshold=%d hysteresis=%d\n", thr, hy);
  if (thr != thresholdC || hy != hysteresisC) {  // zapisovat jen při změně
    if (!saveAutoSettings(thr, hy)) { sendError(500, "save_failed"); return; }
    thresholdC = thr;
    hysteresisC = hy;
    logEvent(EV_SETTINGS);
    applyAutomaticControl("zmena nastaveni");
  }
  sendStatus();
}

void handleWindow() {
  String m = server.arg("mode");
  if (m != "follow" && m != "open" && m != "closed") { sendError(400, "bad_request"); return; }
  int deg = -1;                                   // volitelný zkušební úhel (neuložený)
  if (server.hasArg("deg")) {
    if (m == "follow") { sendError(400, "bad_request"); return; }
    if (!intArg("deg", deg)) { sendError(400, "bad_value"); return; }
    if (!servoAngleValid(deg)) { sendError(400, "bad_angle"); return; }
  }
  if (!servoReady && m != "follow") { sendError(503, "servo_fault"); return; }
  uint8_t next = m == "follow" ? 0 : (m == "open" ? 1 : 2);
  Serial.printf("[HTTP] POST /api/window mode=%s deg=%d\n", m.c_str(), deg);
  if (next != 0) windowManualSince = millis();    // opakovaný povel obnoví 10minutový interval
  bool changed = (windowMode != next) || (next != 0 && windowTestDeg != deg);
  windowMode = next;
  windowTestDeg = (next == 0) ? -1 : deg;
  if (changed)
    logEvent(next == 0 ? EV_WINDOW_FOLLOW
             : (deg >= 0 ? EV_WINDOW_TEST : (next == 1 ? EV_WINDOW_OPEN : EV_WINDOW_CLOSE)));
  servoService();                                 // nový cíl platí okamžitě, i za pohybu
  sendStatus();
}

void handleWinSettings() {
  int c, o, s;
  if (!intArg("closed", c) || !intArg("open", o) || !intArg("speed", s)) {
    sendError(400, "bad_value"); return;
  }
  const char* err = servoSettingsError(c, o, s);
  if (err) { sendError(400, err); return; }
  Serial.printf("[HTTP] POST /api/winsettings zavreno=%d otevreno=%d rychlost=%d\n", c, o, s);
  if (c != windowClosedDeg || o != windowOpenDeg || s != servoSpeedDps) {  // zapisovat jen při změně
    if (!saveWindowSettings(c, o, s)) { sendError(500, "save_failed"); return; }
    windowClosedDeg = c;
    windowOpenDeg = o;
    servoSpeedDps = s;
    logEvent(EV_WINDOW_SETTINGS);
  }
  windowTestDeg = -1;                             // zkušební úhel končí, cíl je opět uložená poloha
  servoService();
  sendStatus();
}

// ---------------------------------------------------------------- diagnostika startu
const char* resetReasonText(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
  }
}

const char* resetReasonHint(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:  return "zapnuti napajeni";
    case ESP_RST_BROWNOUT: return "propad napajeciho napeti pod prah detektoru, podezreni na napajeni";
    case ESP_RST_PANIC:    return "pad programu (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog, nejaka cast programu se zasekla";
    case ESP_RST_SW:       return "softwarovy restart";
    case ESP_RST_EXT:      return "vnejsi reset (tlacitko EN)";
    default:               return "jiny nebo neznamy duvod";
  }
}

// ---------------------------------------------------------------- start
void setup() {
  Serial.begin(115200);
  // Relé: nejdřív úroveň „vypnuto“, teprve potom pin jako výstup (bez cvaknutí relé při startu).
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_HIGH ? LOW : HIGH);
  motorOn = false;

  esp_reset_reason_t rr = esp_reset_reason();
  resetReasonStr = resetReasonText(rr);
  Serial.printf("\n[BOOT] duvod restartu: %s (%s)\n", resetReasonStr, resetReasonHint(rr));
  Serial.printf("[BOOT] volna pamet %lu B\n", (unsigned long)ESP.getFreeHeap());

  prefsReady = prefs.begin("fan", false);
  loadSettings();
  Serial.printf("Nastaveni: zapnout nad %d C, vypnout pri %d C, pamet %s\n", thresholdC,
                thresholdC - hysteresisC, prefsReady ? "OK" : "CHYBA");
  Serial.printf("Okno: zavreno %d, otevreno %d (povel ve stupnich), rychlost %d st/s\n",
                windowClosedDeg, windowOpenDeg, servoSpeedDps);

  dht.begin();
  lastAttemptMs = millis();  // první měření až za SENSOR_INTERVAL_MS
  lastHistMs = millis();     // první bod grafu až za HISTORY_STEP_MS
  fanReason = FR_BOOT;
  fanReasonMs = millis();
  logEvent(EV_BOOT);
  servoInit();               // chyba serva nikdy nezastaví zbytek programu

  if (OLED_ENABLED) {        // chyba displeje nikdy nesmí zastavit zbytek programu
    Wire.begin(OLED_SDA_PIN, OLED_SCL_PIN);
    Wire.setClock(OLED_I2C_HZ);
    if (OLED_SCAN_ON_BOOT) i2cScan();
    oledOk = oledInit();
    oledLastTry = millis();
    Serial.printf("[OLED] %s\n", oledOk ? "inicializovan" : "nenalezen, bezim bez displeje");
  }

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.println("Wi-Fi AP se nespustila (heslo musi mit min. 8 znaku). Restart.");
    delay(3000);
    ESP.restart();
  }

  server.on("/", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);   // bez kopie stránky v RAM
  });
  server.on("/api/status", HTTP_GET, sendStatus);
  server.on("/api/window", HTTP_POST, handleWindow);
  server.on("/api/winsettings", HTTP_POST, handleWinSettings);
  server.on("/api/mode", HTTP_POST, handleMode);
  server.on("/api/fan", HTTP_POST, handleFan);
  server.on("/api/settings", HTTP_POST, handleSettings);
  server.on("/api/log", HTTP_GET, handleLog);
  server.on("/api/history", HTTP_GET, handleHistory);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.printf("Wi-Fi: %s, otevri http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
  uint32_t t = millis();
  if (loopLastMs != 0) {      // měření pauz hlavní smyčky (důkaz, zda něco blokuje)
    uint32_t gap = t - loopLastMs;
    if (gap > loopMaxGapMs) loopMaxGapMs = gap;
    if (gap >= LOOP_WARN_MS) Serial.printf("[LOOP] pauza %lu ms\n", (unsigned long)gap);
  }
  loopLastMs = t;

  server.handleClient();
  if (millis() - lastAttemptMs >= SENSOR_INTERVAL_MS) {
    lastAttemptMs = millis();
    measure();
  }
  applyAutomaticControl("AUTO");  // levné; zajistí i vypnutí po výpadku čidla
  housekeeping();
  servoService();
  oledUpdate();                   // jen zobrazuje uložené hodnoty, čidlo nečte
  delay(2);
}
