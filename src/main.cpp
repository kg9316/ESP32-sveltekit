/**
 *   ESP32 SvelteKit
 *
 *   A simple, secure and extensible framework for IoT projects for ESP32 platforms
 *   with responsive Sveltekit front-end built with TailwindCSS and DaisyUI.
 *   https://github.com/theelims/ESP32-sveltekit
 *
 *   Copyright (C) 2018 - 2023 rjwats
 *   Copyright (C) 2023 - 2024 theelims
 *
 *   All Rights Reserved. This software may be modified and distributed under
 *   the terms of the LGPL v3 license. See the LICENSE file for details.
 **/

#include <M5StamPLC.h>
#include <ESP32SvelteKit.h>
#include <LightMqttSettingsService.h>
#include <LightStateService.h>
#include <PsychicHttpServer.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "HX711.h"
// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 1

HX711 scale;

#define SERIAL_BAUD_RATE 115200

unsigned long lastByteTime = 0;
const unsigned long FRAME_TIMEOUT = 100; // ms

// Parser‑state
enum FrameState
{
    WAIT_SYNC, // venter på 0xFF
    READ_H,    // lese H_DATA
    READ_L,    // lese L_DATA
    READ_SUM   // lese checksum
};

FrameState state = WAIT_SYNC;
uint8_t hData, lData, sumData;

PsychicHttpServer server;

ESP32SvelteKit esp32sveltekit(&server, 120);

LightMqttSettingsService lightMqttSettingsService = LightMqttSettingsService(&server,
                                                                             &esp32sveltekit);

LightStateService lightStateService = LightStateService(&server,
                                                        &esp32sveltekit,
                                                        &lightMqttSettingsService);

// Shared state protected by mutexes
static uint16_t lastDistanceMm = 0; // 0 = out of range
static float lastWeight = 0.0f;
static SemaphoreHandle_t dataMutex;    // protects lastDistanceMm/lastWeight
static SemaphoreHandle_t displayMutex; // serializes LCD drawing
static SemaphoreHandle_t statusMutex;  // protects sequence status text
static volatile bool abortRequested = false;
static bool lastEndstop = false; // cached endstop state
// Endstop wiring assumptions (can adjust after field test)
static constexpr uint8_t ENDSTOP_INPUT_INDEX = 0; // DI1 (index 0)
static constexpr bool ENDSTOP_ACTIVE_LOW = false; // adjusted: endstop is active-low on this hardware

// Relay indices for clarity (must be declared before use)
static constexpr uint8_t RELAY_MOTOR_PWR = 0; // Relay 1
static constexpr uint8_t RELAY_POL_A = 1;     // Relay 2
static constexpr uint8_t RELAY_POL_B = 2;     // Relay 3
static constexpr uint8_t RELAY_FEEDER = 3;    // Relay 4

// Distance sensor timeout + manual feed indicator
static volatile bool distanceTimedOut = true;           // true until first good frame
static const uint32_t SENSOR_TIMEOUT_MS = 1000;         // no bytes for 1s => timeout
static volatile bool manualFeeding = false;             // true while feeder is held/run manually

// Task intervals
static const TickType_t WEIGHT_SAMPLE_TICKS = pdMS_TO_TICKS(100);  // 10 Hz
static const TickType_t TELEMETRY_TICKS = pdMS_TO_TICKS(500);      // 2 Hz
static const TickType_t DISPLAY_TICKS = pdMS_TO_TICKS(200);        // 5 Hz
static const TickType_t IP_DISPLAY_TICKS = pdMS_TO_TICKS(1000);    // 1 Hz

// Task handles (optional)
static TaskHandle_t weightTaskHandle = nullptr;
static TaskHandle_t distanceTaskHandle = nullptr;
static TaskHandle_t telemetryTaskHandle = nullptr;
static TaskHandle_t displayTaskHandle = nullptr;
static TaskHandle_t ipTaskHandle = nullptr;
static TaskHandle_t sequenceTaskHandle = nullptr;
static TaskHandle_t jogTaskHandle = nullptr;
static TaskHandle_t homeTaskHandle = nullptr;
static TaskHandle_t buttonTaskHandle = nullptr;
// One-shot tone task for boot/AP/STA sound cues
static TaskHandle_t toneTaskHandle = nullptr;
static TaskHandle_t endstopToneTaskHandle = nullptr; // background warning beeper
static volatile bool toneInUse = false;              // coordinates exclusive access to speaker
// Periodic auto-sequence timer
static TimerHandle_t sequenceTimer = nullptr;
static volatile uint32_t sequenceIntervalMin = 0; // 0 = off
static volatile TickType_t sequencePeriodTicks = 0; // cached period in ticks
static volatile TickType_t sequenceNextDueTick = 0; // next expected fire time
// Current LCD background color (used for connection status)
static uint16_t gBgColor = TFT_BLACK;
// Sequence status text shown on LCD (one line above IP)
static char gSequenceStatus[32] = "standby";

// Sequence phase tracking for dynamic countdowns on LCD
enum class SeqPhase : uint8_t { Idle = 0, Approach, Feed, Retract };
static volatile SeqPhase gSeqPhase = SeqPhase::Idle;
// Phase timing markers (ticks)
static volatile TickType_t approachStartTick = 0;
static volatile TickType_t approachMaxTicks = 0;
static volatile TickType_t feedEndTick = 0;           // absolute tick when feed ends
static volatile TickType_t retractStartTick = 0;
static volatile TickType_t retractMaxTicks = 0;


struct Kalman1D {
  float x = 0;   // estimate
  float p = 1;   // estimate variance
  float q = 0.01f; // process noise (try 0.001..0.1)
  float r = 4.0f;  // measurement noise variance (grams^2). try 1..50
  bool inited = false;

  float update(float z) {
    if (!inited) { x = z; p = 1; inited = true; return x; }
    // predict
    p = p + q;
    // update
    float k = p / (p + r);
    x = x + k * (z - x);
    p = (1 - k) * p;
    return x;
  }
};

static Kalman1D kf;

// Helpers to control the auto timer lifecycle
static inline void pauseAutoTimer()
{
    if (sequenceTimer)
    {
        xTimerStop(sequenceTimer, 0);
    }
    sequenceNextDueTick = 0;
}

static inline void resumeAutoTimer()
{
    if (sequenceTimer && sequenceIntervalMin > 0)
    {
        sequenceNextDueTick = xTaskGetTickCount() + sequencePeriodTicks;
        xTimerStart(sequenceTimer, 0);
    }
}

// Sequence control
struct SequenceConfig
{
    uint16_t target_mm;    // stop threshold in mm
    uint32_t feed_ms;      // relay 4 ON duration
    uint32_t retract_ms;   // relay 1 ON after releasing 2/3 (legacy)
    uint16_t return_target_mm; // absolute return target after feeding
};
static SequenceConfig currentSequence{300, 3000, 0, 300};
static volatile bool sequenceRunning = false;

void SequenceTask(void *param);
void JogTask(void *param);
void HomeTask(void *param);

// Task forward declarations
void WeightTask(void *param);
void DistanceTask(void *param);
void TelemetryTask(void *param);
void DisplayTask(void *param);
void IPStatusTask(void *param);
void ButtonTask(void *param);
void ToneTask(void *param);
static void EndstopToneTask(void *param);
// IO helpers
static uint8_t readInputsMask();

// Forward decls for sequence status helpers
static inline void setSequenceStatus(const char* text);
static inline void displaySequenceStatus();
static inline void displayScheduleCountdown();
static inline void startTone(int which);

static float median5(float a, float b, float c, float d, float e) {
  float v[5] = {a,b,c,d,e};
  // simple sort 5
  for (int i=0;i<5;i++) for (int j=i+1;j<5;j++) if (v[j] < v[i]) { float t=v[i]; v[i]=v[j]; v[j]=t; }
  return v[2];
}

struct WeightFilter {
  float ema = 0.0f;
  bool inited = false;
  float alpha = 0.15f;     // 0.05..0.3 (lower = smoother)
  float deadband = 2.0f;   // grams, set 0 to disable
  float lastOut = 0.0f;

  float update(float x) {
    if (!inited) { ema = x; lastOut = x; inited = true; return x; }
    ema = ema + alpha * (x - ema);
    float out = ema;
    if (deadband > 0.0f && fabsf(out - lastOut) < deadband) out = lastOut;
    else lastOut = out;
    return out;
  }
};

static WeightFilter wflt;



// Helper to (re)apply timer configuration from either UI event or persisted settings
void applySequenceIntervalFromSettings(int minutes)
{
    uint32_t min = (minutes < 0) ? 0U : (minutes > 60 ? 60U : (uint32_t)minutes);
    sequenceIntervalMin = min;
    // Stop and delete any previous timer
    if (sequenceTimer)
    {
        xTimerStop(sequenceTimer, 0);
        xTimerDelete(sequenceTimer, 0);
        sequenceTimer = nullptr;
    }
    if (sequenceIntervalMin > 0)
    {
        sequencePeriodTicks = pdMS_TO_TICKS(sequenceIntervalMin * 60U * 1000U);
        sequenceTimer = xTimerCreate("seqTimer", sequencePeriodTicks, pdTRUE, nullptr, [](TimerHandle_t) {
            // set next due tick at the time of this callback
            sequenceNextDueTick = xTaskGetTickCount() + sequencePeriodTicks;
            if (!sequenceRunning && !manualFeeding)
            {
                // Pause auto timer while a sequence is in progress
                pauseAutoTimer();
                sequenceRunning = true;
                abortRequested = false;
                if (sequenceTaskHandle)
                {
                    vTaskDelete(sequenceTaskHandle);
                    sequenceTaskHandle = nullptr;
                }
                xTaskCreatePinnedToCore(SequenceTask, "SequenceTask", 4096, nullptr, 3, &sequenceTaskHandle, 0);
            }
        });
        if (sequenceTimer)
        {
            // first fire will be one full period from now
            sequenceNextDueTick = xTaskGetTickCount() + sequencePeriodTicks;
            xTimerStart(sequenceTimer, 0);
        }
    }
    else
    {
        sequencePeriodTicks = 0;
        sequenceNextDueTick = 0;
    }
    // emit current schedule status
    JsonDocument s; s["interval_min"] = (int)sequenceIntervalMin; s["enabled"] = sequenceIntervalMin > 0; JsonObject o = s.as<JsonObject>();
    if (esp32sveltekit.getSocket()) esp32sveltekit.getSocket()->emitEvent("sequence_schedule_status", o);
}

void setup()
{
    // start serial and filesystem
    Serial.begin(SERIAL_BAUD_RATE);
    M5StamPLC.begin();

    kf.q = 0.001f;   
    kf.r = 64.0f;    

    // Less jitter: r ↑, q ↓
    // Faster response: q ↑, r ↓


    gBgColor = TFT_RED;
    M5StamPLC.Display.fillScreen(gBgColor);
    M5StamPLC.Display.setTextColor(TFT_WHITE, gBgColor);

    M5StamPLC.Display.setCursor(10, 10);
    M5StamPLC.Display.setTextColor(TFT_WHITE, gBgColor);
    M5StamPLC.Display.println("BOOT");


    // Konfigurer UART2 på EXT‑port
    // RX2 = GPIO41 (EXT pin 2), TX2 = GPIO40 (EXT pin 1; ubrukt OK)
    Serial2.begin(9600, SERIAL_8N1, /*RX*/ 4, /*TX*/ 5);
    if (!Serial2) {
        M5StamPLC.Display.println("Failed!");
    }
    else {
        M5StamPLC.Display.println("OK");
    }

    M5StamPLC.Display.println("Setup Scale");
    scale.begin(LOADCELL_SCK_PIN, LOADCELL_DOUT_PIN);

    unsigned long t0 = millis();
    while (!scale.is_ready() && (millis() - t0) < 1000) {
    delay(3);
    }

    if (scale.is_ready()) {
    scale.set_medavg_mode();   // stable but still responsive
    scale.tare(15);  // zero
    M5StamPLC.Display.println("Scale OK");
    } else {
    M5StamPLC.Display.println("Scale MISSING");
    }

    // start ESP32-SvelteKit
    esp32sveltekit.begin();

    // Register custom websocket event for demo telemetry (weight + distance)
    esp32sveltekit.getSocket()->registerEvent("demo");
    // Register sequence control event from UI
    esp32sveltekit.getSocket()->registerEvent("sequence");
    esp32sveltekit.getSocket()->registerEvent("sequence_status");
    esp32sveltekit.getSocket()->registerEvent("sequence_abort");
    // Auto schedule events
    esp32sveltekit.getSocket()->registerEvent("sequence_schedule");
    esp32sveltekit.getSocket()->registerEvent("sequence_schedule_get");
    esp32sveltekit.getSocket()->registerEvent("sequence_schedule_status");
    // Manual control events
    esp32sveltekit.getSocket()->registerEvent("jog");
    esp32sveltekit.getSocket()->registerEvent("jog_stop");
    // Manual feed now supports start/stop like jog
    esp32sveltekit.getSocket()->registerEvent("feed");
    esp32sveltekit.getSocket()->registerEvent("feed_stop");
    // Keep legacy for compatibility (one-shot pulse)
    esp32sveltekit.getSocket()->registerEvent("manual_feed");
    esp32sveltekit.getSocket()->registerEvent("home");

    // load the initial light settings
    lightStateService.begin();
    // start the light service
    lightMqttSettingsService.begin();

    // Force backend sequence config to use persisted values at boot
    {
        // Access persisted LightState
        const LightState& state = lightStateService.getCurrentState();
        // Set both sequence targets to persisted values
        currentSequence.return_target_mm = (uint16_t)(state.return_distance_cm * 10);
        // Use persisted lower (target) position in centimeters
        currentSequence.target_mm = (uint16_t)(state.target_distance_cm * 10);
    }

    // Handle incoming sequence commands
    esp32sveltekit.getSocket()->onEvent("sequence", [&](JsonObject &root, int originId) {
        if (sequenceRunning)
        {
            // Busy - ignore request
            return;
        }
        SequenceConfig cfg = currentSequence;
        if (root["target_distance_cm"].is<int>())
        {
            int cm = root["target_distance_cm"].as<int>();
            if (cm < 10)
                cm = 10;
            if (cm > 200)
                cm = 200;
            cfg.target_mm = (uint16_t)(cm * 10);
        }
        if (root["feed_seconds"].is<int>())
        {
            int s = root["feed_seconds"].as<int>();
            if (s < 0)
                s = 0;
            if (s > 120)
                s = 120;
            cfg.feed_ms = (uint32_t)s * 1000U;
        }
        if (root["return_distance_cm"].is<int>())
        {
            int cm = root["return_distance_cm"].as<int>();
            if (cm < 10) cm = 10;
            if (cm > 200) cm = 200;
            cfg.return_target_mm = (uint16_t)(cm * 10);
        }
        currentSequence = cfg;

    // Launch sequence task and pause auto timer while running
    pauseAutoTimer();
    sequenceRunning = true;
        // reset phase markers
        gSeqPhase = SeqPhase::Approach;
        approachStartTick = xTaskGetTickCount();
        approachMaxTicks = pdMS_TO_TICKS(1000*60*10);
        feedEndTick = 0;
        retractStartTick = retractMaxTicks = 0;
        abortRequested = false;
        if (sequenceTaskHandle)
        {
            vTaskDelete(sequenceTaskHandle);
            sequenceTaskHandle = nullptr;
        }
        xTaskCreatePinnedToCore(SequenceTask, "SequenceTask", 4096, nullptr, 3, &sequenceTaskHandle, 0);
    });

    // Abort event
    esp32sveltekit.getSocket()->onEvent("sequence_abort", [&](JsonObject &root, int originId) {
        abortRequested = true;
        // optional: reflect immediate phase clear if UI watches LCD countdown
        if (sequenceRunning) {
            gSeqPhase = SeqPhase::Idle;
            approachStartTick = approachMaxTicks = 0;
            feedEndTick = 0;
            retractStartTick = retractMaxTicks = 0;
        }
    });

    // Schedule set: expects { interval_min: number }
    esp32sveltekit.getSocket()->onEvent("sequence_schedule", [&](JsonObject &root, int originId) {
        int min = (int)(root["interval_min"] | 0);
        applySequenceIntervalFromSettings(min);
    });

    // Schedule get: reply with current status
    esp32sveltekit.getSocket()->onEvent("sequence_schedule_get", [&](JsonObject &root, int originId) {
        JsonDocument s; s["interval_min"] = (int)sequenceIntervalMin; s["enabled"] = sequenceIntervalMin > 0; JsonObject o = s.as<JsonObject>();
        esp32sveltekit.getSocket()->emitEvent("sequence_schedule_status", o);
    });

    // Jog start (dir: 'up' | 'down')
    esp32sveltekit.getSocket()->onEvent("jog", [&](JsonObject &root, int originId) {
        if (jogTaskHandle) { vTaskDelete(jogTaskHandle); jogTaskHandle = nullptr; }
        abortRequested = false;
        // dir 'up' means retract towards larger distance, 'down' means approach smaller distance
        String dir = root["dir"] | String("up");
        bool up = (dir == "up");
        xTaskCreatePinnedToCore(JogTask, up ? "JogUp" : "JogDown", 4096, (void*)(up ? 1 : 0), 3, &jogTaskHandle, 0);
    });

    // Jog stop
    esp32sveltekit.getSocket()->onEvent("jog_stop", [&](JsonObject &root, int originId) {
        abortRequested = true; // reuse abort flag to stop jog/other manual tasks
    });

    // Manual feed pulse (legacy)
    esp32sveltekit.getSocket()->onEvent("manual_feed", [&](JsonObject &root, int originId) {
        uint32_t seconds = (uint32_t)(root["seconds"] | 1);
        if (seconds > 10) seconds = 10;
        M5StamPLC.writePlcRelay(3, true);
        vTaskDelay(pdMS_TO_TICKS(seconds * 1000));
        M5StamPLC.writePlcRelay(3, false);
    });

    // Continuous feed start/stop
    esp32sveltekit.getSocket()->onEvent("feed", [&](JsonObject &root, int originId) {
        abortRequested = false;
        // turn on feeder relay
    manualFeeding = true;
    M5StamPLC.writePlcRelay(3, true);
    setSequenceStatus("manual feed");
    // Notify UI
    {
        JsonDocument sdoc;
        sdoc["phase"] = "manual_feed";
        sdoc["running"] = true;
        sdoc["distance_mm"] = (int)lastDistanceMm;
        sdoc["endstop"] = lastEndstop;
        sdoc["di_mask"] = (int)readInputsMask();
        JsonObject o = sdoc.as<JsonObject>();
        esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
    }
    });
    esp32sveltekit.getSocket()->onEvent("feed_stop", [&](JsonObject &root, int originId) {
        // turn off feeder relay
    manualFeeding = false;
    M5StamPLC.writePlcRelay(3, false);
    setSequenceStatus("standby");
    // Notify UI
    {
        JsonDocument sdoc;
        sdoc["phase"] = lastEndstop ? "endstop" : "standby";
        sdoc["running"] = false;
        sdoc["distance_mm"] = (int)lastDistanceMm;
        sdoc["endstop"] = lastEndstop;
        sdoc["di_mask"] = (int)readInputsMask();
        JsonObject o = sdoc.as<JsonObject>();
        esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
    }
    });

    // Return to home (target_distance_cm)
    esp32sveltekit.getSocket()->onEvent("home", [&](JsonObject &root, int originId) {
        if (sequenceRunning) return; // do not allow while sequence is active
        if (homeTaskHandle) { vTaskDelete(homeTaskHandle); homeTaskHandle = nullptr; }
        abortRequested = false;
        uint16_t cm = (uint16_t)(root["target_distance_cm"] | 30);
        if (cm < 10) cm = 10; if (cm > 200) cm = 200;
        uint16_t *target_mm = (uint16_t*)pvPortMalloc(sizeof(uint16_t));
        if (!target_mm) return;
        *target_mm = (uint16_t)(cm * 10);
        xTaskCreatePinnedToCore(HomeTask, "HomeTask", 4096, target_mm, 3, &homeTaskHandle, 0);
    });

    M5StamPLC.Display.setTextColor(TFT_WHITE, TFT_BLACK);
    M5StamPLC.Display.setTextSize(2);
    // Start with red background during boot/no IP
    

    // Boost speaker volume a bit for clearer cues (0-255 typical)
    M5.Speaker.setVolume(220);

    // Boot jingle (non-blocking)
    startTone(0);

    // Create mutexes
    dataMutex = xSemaphoreCreateMutex();
    displayMutex = xSemaphoreCreateMutex();
    statusMutex = xSemaphoreCreateMutex();
    setSequenceStatus("standby");

    // Create tasks (pin lightweight IO tasks to APP CPU core 0 to keep WiFi stack on core 1)
    xTaskCreatePinnedToCore(WeightTask, "WeightTask", 4096, nullptr, 2, &weightTaskHandle, 0);
    xTaskCreatePinnedToCore(DistanceTask, "DistanceTask", 4096, nullptr, 2, &distanceTaskHandle, 0);
    xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", 4096, nullptr, 1, &telemetryTaskHandle, 0);
    xTaskCreatePinnedToCore(DisplayTask, "DisplayTask", 4096, nullptr, 1, &displayTaskHandle, 0);
    xTaskCreatePinnedToCore(IPStatusTask, "IPStatusTask", 4096, nullptr, 1, &ipTaskHandle, 0);
    xTaskCreatePinnedToCore(ButtonTask, "ButtonTask", 4096, nullptr, 3, &buttonTaskHandle, 0);
}

// Distance validity bounds (in millimeters)
static constexpr uint16_t MIN_VALID_DISTANCE_MM = 22 * 10;   // 22 cm
static constexpr uint16_t MAX_VALID_DISTANCE_MM = 185 * 10;  // 185 cm

void displayDistance(uint16_t mm)
{
    // Apply validity filter: outside bounds => treat as out-of-range (0)
    uint16_t filtered = (mm >= MIN_VALID_DISTANCE_MM && mm <= MAX_VALID_DISTANCE_MM) ? mm : 0;

    if (displayMutex && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        M5StamPLC.Display.setCursor(10, 30);
        if (distanceTimedOut)
        {
            M5StamPLC.Display.print("Timeout          ");
        }
        else if (filtered == 0)
        {
            M5StamPLC.Display.print("Out of range     ");
        }
        else
        {
            // Vis som cm med én desimal, f.eks. 1953 mm → 195.3 cm
            float cm = filtered / 10.0f;
            M5StamPLC.Display.printf("%.1fcm            ", cm);
        }
        xSemaphoreGive(displayMutex);
    }

    // save latest distance
    lastDistanceMm = filtered;
}

void displayIPStatus()
{
    // Determine current IP and connectivity status
    bool staConnected = WiFi.isConnected();
    bool apMode = (WiFi.getMode() & WIFI_MODE_AP);
    IPAddress ip = staConnected ? WiFi.localIP() : (apMode ? WiFi.softAPIP() : IPAddress());

    // Consider it "ok" if STA is connected or AP mode is active with a valid IP
    IPAddress none(0, 0, 0, 0);
    bool ok = (staConnected && ip != none) || (apMode && ip != none);

    // Pick background color by status:
    // - Red: no IP (booting or disconnected)
    // - Blue: Access Point mode with IP
    // - Dark Green: Connected to a WiFi network (STA) with IP
    uint16_t desiredBg = gBgColor;
    if (staConnected && ip != none) {
        // darker green for better readability
        desiredBg = M5StamPLC.Display.color565(0, 120, 0);
    } else if (apMode && ip != none) {
        desiredBg = TFT_BLUE;
    } else {
        desiredBg = TFT_RED;
    }

    // Detect state transitions to play tones
    // 0 = no IP, 1 = AP mode, 2 = STA connected
    int currState = (staConnected && ip != none) ? 2 : ((apMode && ip != none) ? 1 : 0);
    static int prevState = -1;
    bool stateChanged = (currState != prevState);

    // Build the IP status line we want to show
    char line[64];
    snprintf(line, sizeof(line), "IP: %s %s", ip.toString().c_str(), ok ? "ok" : "...");

    // Cache last drawn content to avoid unnecessary redraws
    static char prevLine[64] = {0};
    bool contentChanged = (strncmp(line, prevLine, sizeof(prevLine)) != 0);

    // Draw atomically to avoid partial updates when other tasks are drawing
    if (displayMutex && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
        bool bgChanged = false;
        if (desiredBg != gBgColor) {
            M5StamPLC.Display.fillScreen(desiredBg);
            gBgColor = desiredBg;
            bgChanged = true;
        }

        // Redraw the line only if content or background changed
        if (bgChanged || contentChanged)
        {
            int16_t h = M5StamPLC.Display.fontHeight();
            int16_t y = M5StamPLC.Display.height() - h - 2;
            // Clear the row once to avoid per-glyph background blinking
            M5StamPLC.Display.fillRect(0, y, M5StamPLC.Display.width(), h + 2, gBgColor);
            // Draw text with explicit background for reliability
            M5StamPLC.Display.setTextColor(TFT_WHITE, gBgColor);
            M5StamPLC.Display.setCursor(10, y);
            M5StamPLC.Display.print(line);
            // Remember what we drew
            strncpy(prevLine, line, sizeof(prevLine) - 1);
            prevLine[sizeof(prevLine) - 1] = '\0';
        }
        xSemaphoreGive(displayMutex);
    }

    // Play tones after releasing display lock
    if (stateChanged)
    {
        if (currState == 1) {
            startTone(1); // AP tone
        } else if (currState == 2) {
            startTone(2); // STA connected tone
        }
        prevState = currState;
    }
}

// Update the global sequence status string (thread-safe)
static inline void setSequenceStatus(const char* text)
{
    if (!text) return;
    if (statusMutex && xSemaphoreTake(statusMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        size_t n = strnlen(text, sizeof(gSequenceStatus) - 1);
        memcpy(gSequenceStatus, text, n);
        gSequenceStatus[n] = '\0';
        xSemaphoreGive(statusMutex);
    }
}

// Draw the current sequence status on the line above the IP line
static inline void displaySequenceStatus()
{
    char line[64];
    char status[32];
    // Snapshot current status string
    if (statusMutex && xSemaphoreTake(statusMutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        size_t n = strnlen(gSequenceStatus, sizeof(status) - 1);
        memcpy(status, gSequenceStatus, n);
        status[n] = '\0';
        xSemaphoreGive(statusMutex);
    }
    else
    {
        strncpy(status, "...", sizeof(status));
        status[sizeof(status) - 1] = '\0';
    }

    // Base: Status text only (countdown is rendered on a separate line)
    snprintf(line, sizeof(line), "Status: %s", status);

    if (displayMutex && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        int16_t h = M5StamPLC.Display.fontHeight();
        int16_t y = M5StamPLC.Display.height() - (2 * h) - 4;
        // Clear the entire row to avoid overlapping with IP line below
        M5StamPLC.Display.fillRect(0, y, M5StamPLC.Display.width(), h + 2, gBgColor);
        M5StamPLC.Display.setTextColor(TFT_WHITE, gBgColor);
        M5StamPLC.Display.setCursor(10, y);
        M5StamPLC.Display.print(line);
        xSemaphoreGive(displayMutex);
    }
}

// Draw the schedule countdown on the line above the status line
static inline void displayScheduleCountdown()
{
    char line[32];
    line[0] = '\0';

    // During a running sequence, show phase-specific countdowns
    if (sequenceRunning)
    {
        TickType_t now = xTaskGetTickCount();
        if (gSeqPhase == SeqPhase::Approach && approachMaxTicks > 0)
        {
            int32_t left = (int32_t)((approachStartTick + approachMaxTicks) - now);
            if (left < 0) left = 0;
            uint32_t secs = (uint32_t)left * portTICK_PERIOD_MS / 1000U;
            uint32_t mm = secs / 60U;
            uint32_t ss = secs % 60U;
            snprintf(line, sizeof(line), "Max Appr: %02u:%02u", (unsigned)mm, (unsigned)ss);
        }
        else if (gSeqPhase == SeqPhase::Feed && feedEndTick > 0)
        {
            int32_t left = (int32_t)(feedEndTick - now);
            if (left < 0) left = 0;
            uint32_t secs = (uint32_t)left * portTICK_PERIOD_MS / 1000U;
            uint32_t mm = secs / 60U;
            uint32_t ss = secs % 60U;
            snprintf(line, sizeof(line), "Feed: %02u:%02u", (unsigned)mm, (unsigned)ss);
        }
        else if (gSeqPhase == SeqPhase::Retract && retractMaxTicks > 0)
        {
            int32_t left = (int32_t)((retractStartTick + retractMaxTicks) - now);
            if (left < 0) left = 0;
            uint32_t secs = (uint32_t)left * portTICK_PERIOD_MS / 1000U;
            uint32_t mm = secs / 60U;
            uint32_t ss = secs % 60U;
            snprintf(line, sizeof(line), "Max Home: %02u:%02u", (unsigned)mm, (unsigned)ss);


        }
    }

    // When idle and auto-schedule is enabled, show time until next run
    if (line[0] == '\0')
    {
        bool show = (sequenceIntervalMin > 0 && sequenceNextDueTick > 0 && !sequenceRunning);
        if (show)
        {
            TickType_t now = xTaskGetTickCount();
            int32_t ticksLeft = (int32_t)(sequenceNextDueTick - now);
            if (ticksLeft < 0) ticksLeft = 0;
            uint32_t secs = (uint32_t)ticksLeft * portTICK_PERIOD_MS / 1000U;
            uint32_t mm = secs / 60U;
            uint32_t ss = secs % 60U;
            snprintf(line, sizeof(line), "Nxt: %02u:%02u", (unsigned)mm, (unsigned)ss);
        }
    }

    if (displayMutex && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        int16_t h = M5StamPLC.Display.fontHeight();
        int16_t y = M5StamPLC.Display.height() - (3 * h) - 6;
        // Clear the entire row to avoid overlapping with status line below
        M5StamPLC.Display.fillRect(0, y, M5StamPLC.Display.width(), h + 2, gBgColor);
        M5StamPLC.Display.setTextColor(TFT_WHITE, gBgColor);
        M5StamPLC.Display.setCursor(10, y);
        if (line[0])
            M5StamPLC.Display.print(line);
        xSemaphoreGive(displayMutex);
    }
}




// Note: LightStateService only exposes led_on via the "led" event.
// Feed/retract seconds are UI-only controls sent with the same event and
// not stored server-side in LightState.

// Read endstop input (digital input 1)
static bool readEndstop()
{
    // M5StamPLC input channels are 0..7
    bool raw = M5StamPLC.readPlcInput(ENDSTOP_INPUT_INDEX);
    bool state = ENDSTOP_ACTIVE_LOW ? !raw : raw;
    lastEndstop = state;
    return state;
}

// Read a mask of digital inputs (up to 8)
static uint8_t readInputsMask()
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        bool v = M5StamPLC.readPlcInput(i);
        mask |= (uint8_t(v) << i);
    }
    return mask;
}

void loop()
{
    // Keep device services and button handling responsive on the Arduino loop
    M5StamPLC.update();
    // Poll endstop
    readEndstop();
    // Reflect endstop state on the LCD status when idle
    static bool prevEndstop = false;
    if (lastEndstop != prevEndstop)
    {
        prevEndstop = lastEndstop;
        bool idle = (!sequenceRunning && !manualFeeding && jogTaskHandle == nullptr && homeTaskHandle == nullptr);
        if (lastEndstop)
        {
            setSequenceStatus("endstop");
            // Start warning tone task if not already running
            if (!endstopToneTaskHandle)
            {
                xTaskCreatePinnedToCore(EndstopToneTask, "EndstopWarn", 2048, nullptr, 1, &endstopToneTaskHandle, 1);
            }
        }
        else if (idle)
        {
            setSequenceStatus("standby");
            // Stop warning tone task if running
            if (endstopToneTaskHandle)
            {
                vTaskDelete(endstopToneTaskHandle);
                endstopToneTaskHandle = nullptr;
            }
        }
    }
}

// ========================= Tasks ========================= //

void WeightTask(void *param)
{
  for (;;)
  {


    float filtered = scale.get_units(9);  // smooth input

    if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      lastWeight = filtered;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(WEIGHT_SAMPLE_TICKS);
  }
}

void DistanceTask(void *param)
{
    for (;;)
    {
        // Mark timeout if sensor has been silent for too long
        unsigned long nowCheck = millis();
        if ((nowCheck - lastByteTime) > SENSOR_TIMEOUT_MS && !distanceTimedOut)
        {
            distanceTimedOut = true;
            if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
            {
                lastDistanceMm = 0;
                xSemaphoreGive(dataMutex);
            }
            displayDistance(0);
        }

        while (Serial2.available())
        {
            uint8_t b = Serial2.read();
            unsigned long now = millis();

            // If it’s been too long since the last byte, reset
            if (now - lastByteTime > FRAME_TIMEOUT)
            {
                state = WAIT_SYNC;
                if (displayMutex && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    M5StamPLC.Display.setCursor(10, 30);
                    M5StamPLC.Display.print("Timeout ");
                    float secs = now / 1000.0f;
                    M5StamPLC.Display.print(secs, 1);
                    xSemaphoreGive(displayMutex);
                }
            }
            lastByteTime = now;

            switch (state)
            {
            case WAIT_SYNC:
                if (b == 0xFF)
                    state = READ_H;
                break;

            case READ_H:
                hData = b;
                state = READ_L;
                break;

            case READ_L:
                lData = b;
                state = READ_SUM;
                break;

            case READ_SUM:
                sumData = b;
                if (((0xFF + hData + lData) & 0xFF) == sumData)
                {
                    uint16_t distance = (uint16_t(hData) << 8) | lData;
                    // Update shared distance and draw
                    if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
                    {
                        lastDistanceMm = distance;
                        xSemaphoreGive(dataMutex);
                    }
                    distanceTimedOut = false; // got a good frame
                    displayDistance(distance);

                    // flush remaining bytes
                    while (Serial2.available())
                    {
                        Serial2.read();
                    }
                }
                state = WAIT_SYNC;
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void TelemetryTask(void *param)
{
    for (;;)
    {
        float weightCopy;
        uint16_t distanceCopy;
        if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            weightCopy = lastWeight;
            distanceCopy = lastDistanceMm;
            xSemaphoreGive(dataMutex);
        }

        JsonDocument doc;
        doc["weight_g"] = (int)weightCopy;
        doc["distance_mm"] = (int)distanceCopy;
    doc["endstop"] = lastEndstop;
    doc["di_mask"] = (int)readInputsMask();
        // include remaining seconds until next scheduled start, when enabled
        if (sequenceIntervalMin > 0 && sequenceNextDueTick > 0)
        {
            TickType_t now = xTaskGetTickCount();
            int32_t ticksLeft = (int32_t)(sequenceNextDueTick - now);
            if (ticksLeft < 0) ticksLeft = 0;
            uint32_t msLeft = (uint32_t)ticksLeft * portTICK_PERIOD_MS;
            doc["next_start_in_s"] = (int)(msLeft / 1000U);
        }
    JsonObject obj = doc.as<JsonObject>();
        esp32sveltekit.getSocket()->emitEvent("demo", obj);

        vTaskDelay(TELEMETRY_TICKS);
    }
}

void DisplayTask(void *param)
{
    for (;;)
    {
        float weightCopy;
        uint16_t distanceCopy;
        if (dataMutex && xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
        {
            weightCopy = lastWeight;
            distanceCopy = lastDistanceMm;
            xSemaphoreGive(dataMutex);
        }

        if (displayMutex && xSemaphoreTake(displayMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            M5StamPLC.Display.setCursor(10, 10);
            M5StamPLC.Display.print("Weight: ");
            M5StamPLC.Display.print(weightCopy, 0);
            M5StamPLC.Display.print(" g     ");
            xSemaphoreGive(displayMutex);
        }

        // Draw distance using helper (handles mutex inside)
        displayDistance(distanceCopy);

    // Draw sequence status and countdown lines above the IP line
    displayScheduleCountdown();
    displaySequenceStatus();

        vTaskDelay(DISPLAY_TICKS);
    }
}

void IPStatusTask(void *param)
{
    for (;;)
    {
        displayIPStatus();
        // Detect client connection changes and jingle
        static unsigned int prevClients = 0;
        unsigned int clients = esp32sveltekit.getSocket()->getConnectedClients();
        if (clients != prevClients)
        {
            if (clients > prevClients)
            {
                startTone(3); // client connected
            }
            else
            {
                startTone(4); // client disconnected
            }
            prevClients = clients;
        }
        vTaskDelay(IP_DISPLAY_TICKS);
    }
}

// Hold-to-run buttons: A=jog up, B=jog down, C=feed
void ButtonTask(void *param)
{
    bool aPrev = false, bPrev = false, cPrev = false;
    for (;;)
    {
        // Update M5 buttons state
        M5StamPLC.update();
        bool a = M5StamPLC.BtnA.isPressed();
        bool b = M5StamPLC.BtnB.isPressed();
        bool c = M5StamPLC.BtnC.isPressed();

        // A: jog up while held
        if (a && !aPrev)
        {
            if (!sequenceRunning)
            {
                abortRequested = false;
                // Stop any ongoing jog task first
                if (jogTaskHandle) { abortRequested = true; vTaskDelay(pdMS_TO_TICKS(30)); }
                xTaskCreatePinnedToCore(JogTask, "JogUpBtn", 4096, (void*)1, 3, &jogTaskHandle, 0);
                setSequenceStatus("jog up");
            }
        }
        if (!a && aPrev)
        {
            // release -> stop jogging
            abortRequested = true;
        }

        // B: jog down while held
        if (b && !bPrev)
        {
            if (!sequenceRunning)
            {
                abortRequested = false;
                if (jogTaskHandle) { abortRequested = true; vTaskDelay(pdMS_TO_TICKS(30)); }
                xTaskCreatePinnedToCore(JogTask, "JogDownBtn", 4096, (void*)0, 3, &jogTaskHandle, 0);
                setSequenceStatus("jog down");
            }
        }
        if (!b && bPrev)
        {
            abortRequested = true;
        }

    // C: feed while held
        if (c && !cPrev)
        {
            if (!sequenceRunning)
            {
        manualFeeding = true;
        M5StamPLC.writePlcRelay(RELAY_FEEDER, true);
                setSequenceStatus("manual feed");
                // UI event: manual feed running
                {
                    JsonDocument sdoc; sdoc["phase"] = "manual_feed"; sdoc["running"] = true; sdoc["distance_mm"] = (int)lastDistanceMm; sdoc["endstop"] = lastEndstop; sdoc["di_mask"] = (int)readInputsMask(); JsonObject o = sdoc.as<JsonObject>(); esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
                }
            }
        }
        if (!c && cPrev)
        {
        manualFeeding = false;
        M5StamPLC.writePlcRelay(RELAY_FEEDER, false);
            setSequenceStatus("standby");
            // UI event: manual feed stopped
            {
                JsonDocument sdoc; sdoc["phase"] = lastEndstop ? "endstop" : "standby"; sdoc["running"] = false; sdoc["distance_mm"] = (int)lastDistanceMm; sdoc["endstop"] = lastEndstop; sdoc["di_mask"] = (int)readInputsMask(); JsonObject o = sdoc.as<JsonObject>(); esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
            }
        }

        aPrev = a; bPrev = b; cPrev = c;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// (reverted) readDistanceFiltered helper removed

void SequenceTask(void *param)
{
    // Snapshot config at start
    SequenceConfig cfg = currentSequence;
    auto emit_status = [&](const char* phase, const char* reason){
        JsonDocument sdoc;
        sdoc["phase"] = phase;
        sdoc["running"] = sequenceRunning;
        sdoc["distance_mm"] = (int)lastDistanceMm;
        sdoc["endstop"] = lastEndstop;
        sdoc["di_mask"] = (int)readInputsMask();
        sdoc["target_down_mm"] = (int)cfg.target_mm;
        sdoc["return_target_mm"] = (int)cfg.return_target_mm;
        if (reason) sdoc["reason"] = reason;
        JsonObject o = sdoc.as<JsonObject>();
        esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
    };

    emit_status("approach", nullptr);
    setSequenceStatus("approach");
    // Mark phase and safety timeout window
    gSeqPhase = SeqPhase::Approach;

    // Step 1: Engage polarity relays (2 and 3)
    M5StamPLC.writePlcRelay(RELAY_POL_A, true);
    M5StamPLC.writePlcRelay(RELAY_POL_B, true);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Step 2: Power motor (relay 1)
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, true);

    // Step 3: Run until distance within tolerance band of target
    const uint16_t target = cfg.target_mm;
    const uint16_t tol = 10; // 10 mm tolerance
    const TickType_t maxApproach = pdMS_TO_TICKS(1000*60*5); // 5m safety timeout
    TickType_t startTick = xTaskGetTickCount();
    approachStartTick = startTick;
    approachMaxTicks = maxApproach;
    for (;;)
    {
        if (abortRequested) {
            emit_status("aborted", "user abort");
            break;
        }
        if (readEndstop()) {
            emit_status("endstop", "hit during approach");
            break;
        }
    uint16_t mm = lastDistanceMm;
        if (mm != 0)
        {
            if ((mm >= (target >= tol ? target - tol : 0)) && (mm <= (uint16_t)(target + tol)))
            {
                break; // reached target band
            }
        }
        if ((xTaskGetTickCount() - startTick) > maxApproach)
        {
            break; // timeout
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // Stop motor
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, false);

    // Step 4: Run feeder for feed_ms
    if (cfg.feed_ms > 0 && !abortRequested && !readEndstop())
    {
        emit_status("feed", nullptr);
        setSequenceStatus("feed");
        gSeqPhase = SeqPhase::Feed;
        feedEndTick = xTaskGetTickCount() + pdMS_TO_TICKS(cfg.feed_ms);
        M5StamPLC.writePlcRelay(RELAY_FEEDER, true);
        vTaskDelay(pdMS_TO_TICKS(cfg.feed_ms));
        M5StamPLC.writePlcRelay(RELAY_FEEDER, false);
        feedEndTick = 0; // clear
    }

    // Step 5: Release polarity relays (2 and 3)
    M5StamPLC.writePlcRelay(RELAY_POL_A, false);
    M5StamPLC.writePlcRelay(RELAY_POL_B, false);
    vTaskDelay(pdMS_TO_TICKS(200));

    // Step 6: Retract to return_target_mm
    if (!abortRequested)
    {
        emit_status("retract", nullptr);
        setSequenceStatus("retract");
        gSeqPhase = SeqPhase::Retract;
        const uint16_t retTarget = cfg.return_target_mm;
        const TickType_t maxRetract = pdMS_TO_TICKS(1000*60*5); // 5m safety timeout
        TickType_t retStart = xTaskGetTickCount();
        retractStartTick = retStart;
        retractMaxTicks = maxRetract;
        // Decide direction based on current distance vs. return target
        uint16_t cur = lastDistanceMm;
        bool goDown = false; // down = approach (distance decreases)
        if (cur == 0)
        {
            // if unknown, prefer retract/up to get to a safe open position
            goDown = false;
        }
        else
        {
            // if we are greater than target, we need to go down (decrease distance)
            goDown = (cur > retTarget);
        }
        // Set polarity: down => both on, up => both off
        M5StamPLC.writePlcRelay(RELAY_POL_A, goDown);
        M5StamPLC.writePlcRelay(RELAY_POL_B, goDown);
        vTaskDelay(pdMS_TO_TICKS(100));
        M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, true);
        for(;;) {
            if (abortRequested) { emit_status("aborted", "user abort"); break; }
            if (readEndstop()) { emit_status("endstop", "hit during retract"); break; }
        uint16_t mm = lastDistanceMm;
            if (mm != 0) {
                if ((mm >= (retTarget >= tol ? retTarget - tol : 0)) && (mm <= (uint16_t)(retTarget + tol))) {
                    break;
                }
            }
            if ((xTaskGetTickCount() - retStart) > maxRetract) { break; }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, false);
        // Restore polarity off
        M5StamPLC.writePlcRelay(RELAY_POL_A, false);
        M5StamPLC.writePlcRelay(RELAY_POL_B, false);
    }

    sequenceRunning = false;
    // On success, report standby for consistency across UI and LCD
    emit_status(abortRequested ? "aborted" : (lastEndstop ? "endstop" : "standby"), nullptr);
    setSequenceStatus(abortRequested ? "aborted" : (lastEndstop ? "endstop" : "standby"));
    // Manage endstop warning tone based on current state
    if (lastEndstop)
    {
        if (!endstopToneTaskHandle)
            xTaskCreatePinnedToCore(EndstopToneTask, "EndstopWarn", 2048, nullptr, 1, &endstopToneTaskHandle, 1);
    }
    else if (endstopToneTaskHandle)
    {
        vTaskDelete(endstopToneTaskHandle);
        endstopToneTaskHandle = nullptr;
    }
    // Clear phase markers
    gSeqPhase = SeqPhase::Idle;
    approachStartTick = approachMaxTicks = 0;
    feedEndTick = 0;
    retractStartTick = retractMaxTicks = 0;
    // Resume auto-timer after sequence completes
    resumeAutoTimer();
    sequenceTaskHandle = nullptr;
    vTaskDelete(NULL);
}

// Jog implementation: param is (void*)1 for up, (void*)0 for down
void JogTask(void *param)
{
    bool up = (param == (void*)1);
    auto emit_status = [&](const char* phase){
        JsonDocument sdoc; sdoc["phase"] = phase; sdoc["running"] = true; sdoc["distance_mm"] = (int)lastDistanceMm; sdoc["endstop"] = lastEndstop; sdoc["di_mask"] = (int)readInputsMask(); JsonObject o = sdoc.as<JsonObject>(); esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
    };
    emit_status(up ? "jog_up" : "jog_down");
    setSequenceStatus(up ? "jog up" : "jog down");
    // Set polarity for direction (both relays on is our "down"/approach per sequence; use off for up)
    M5StamPLC.writePlcRelay(RELAY_POL_A, up ? false : true);
    M5StamPLC.writePlcRelay(RELAY_POL_B, up ? false : true);
    vTaskDelay(pdMS_TO_TICKS(100));
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, true);
    while (!abortRequested)
    {
        if (readEndstop()) break;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, false);
    // restore polarity off when done jogging
    M5StamPLC.writePlcRelay(RELAY_POL_A, false);
    M5StamPLC.writePlcRelay(RELAY_POL_B, false);
    abortRequested = false;
    setSequenceStatus("standby");
    JsonDocument sdoc; sdoc["phase"] = "standby"; sdoc["running"] = false; sdoc["distance_mm"] = (int)lastDistanceMm; sdoc["endstop"] = lastEndstop; sdoc["di_mask"] = (int)readInputsMask(); JsonObject o = sdoc.as<JsonObject>(); esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
    jogTaskHandle = nullptr; vTaskDelete(NULL);
}

// Home implementation: param is pointer to target mm
void HomeTask(void *param)
{
    uint16_t target = *(uint16_t*)param; vPortFree(param);
    auto emit_status = [&](const char* phase){ JsonDocument sdoc; sdoc["phase"] = phase; sdoc["running"] = true; sdoc["distance_mm"] = (int)lastDistanceMm; sdoc["endstop"] = lastEndstop; sdoc["di_mask"] = (int)readInputsMask(); sdoc["return_target_mm"] = (int)target; JsonObject o = sdoc.as<JsonObject>(); esp32sveltekit.getSocket()->emitEvent("sequence_status", o); };
    emit_status("home");
    setSequenceStatus("home");
    // We'll move towards target; decide direction based on current distance
    uint16_t mm = lastDistanceMm;
    bool needDown = (mm == 0) ? true : (mm > target); // if unknown, default to down
    // Set polarity for direction
    M5StamPLC.writePlcRelay(RELAY_POL_A, needDown);
    M5StamPLC.writePlcRelay(RELAY_POL_B, needDown);
    vTaskDelay(pdMS_TO_TICKS(100));
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, true);
    const uint16_t tol = 5;
    TickType_t start = xTaskGetTickCount();
    const TickType_t maxT = pdMS_TO_TICKS(30000);
    while (!abortRequested)
    {
        if (readEndstop()) break;
    uint16_t d = lastDistanceMm;
        if (d != 0 && d >= (target >= tol ? target - tol : 0) && d <= (uint16_t)(target + tol)) break;
        if ((xTaskGetTickCount() - start) > maxT) break;
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    M5StamPLC.writePlcRelay(RELAY_MOTOR_PWR, false);
    M5StamPLC.writePlcRelay(RELAY_POL_A, false);
    M5StamPLC.writePlcRelay(RELAY_POL_B, false);
    JsonDocument sdoc; sdoc["phase"] = abortRequested ? "aborted" : (lastEndstop ? "endstop" : "standby"); sdoc["running"] = false; sdoc["distance_mm"] = (int)lastDistanceMm; sdoc["endstop"] = lastEndstop; sdoc["di_mask"] = (int)readInputsMask(); sdoc["return_target_mm"] = (int)target; JsonObject o = sdoc.as<JsonObject>(); esp32sveltekit.getSocket()->emitEvent("sequence_status", o);
    setSequenceStatus(abortRequested ? "aborted" : (lastEndstop ? "endstop" : "standby"));
    abortRequested = false;
    homeTaskHandle = nullptr; vTaskDelete(NULL);
}

// ========================= Tone Playback ========================= //

static inline void startTone(int which)
{
    // Stop any existing tone task to avoid overlap
    if (toneTaskHandle) {
        vTaskDelete(toneTaskHandle);
        toneTaskHandle = nullptr;
    }
    xTaskCreatePinnedToCore(ToneTask, "ToneTask", 2048, (void*)which, 1, &toneTaskHandle, 1);
}

void ToneTask(void *param)
{
    int which = (int)param;
    // Simple sequences using M5.Speaker.tone(freq, ms)
    auto beep = [&](uint16_t f, uint16_t ms, uint16_t gap){
        toneInUse = true;
        M5.Speaker.tone(f, ms);
        vTaskDelay(pdMS_TO_TICKS(ms + gap));
        toneInUse = false;
    };

    switch (which)
    {
    case 0: // Boot jingle (two short beeps)
        beep(880, 80, 60);
        beep(880, 100, 0);
        break;
    case 1: // AP mode
        beep(880, 100, 40);
        beep(1047, 100, 40);
        beep(1319, 160, 0);
        break;
    case 2: // STA connected (ascending chime)
        beep(1319, 120, 40);
        beep(1760, 160, 0);
        break;
    case 3: // Client connected (short up chirp)
        beep(1200, 80, 30);
        beep(1500, 90, 0);
        break;
    case 4: // Client disconnected (short down chirp)
        beep(900, 90, 30);
        beep(700, 80, 0);
        break;
    default:
        break;
    }

    toneTaskHandle = nullptr;
    vTaskDelete(NULL);
}

// Background endstop warning: short periodic beeps while endstop is active
static void EndstopToneTask(void *param)
{
    const TickType_t period = pdMS_TO_TICKS(1000); // once per second
    const uint16_t freq = 600;
    const uint16_t durMs = 120;
    for (;;)
    {
        // Exit when endstop clears
        if (!lastEndstop) break;
        // Skip if another tone is playing
        if (!toneInUse)
        {
            toneInUse = true;
            M5.Speaker.tone(freq, durMs);
            vTaskDelay(pdMS_TO_TICKS(durMs));
            toneInUse = false;
        }
        vTaskDelay(period - pdMS_TO_TICKS(durMs));
    }
    endstopToneTaskHandle = nullptr;
    vTaskDelete(NULL);
}
