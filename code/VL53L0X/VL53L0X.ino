/*=============================================================================
 *  VL53L0X SENSOR TEST - BALL AND BEAM PROJECT
 *  ---------------------------------------------------------------------------
 *  Platform    : ESP32-S3
 *  Sensor      : VL53L0X
 *  Display     : SSD1306 OLED 0.96" I2C
 *  Bus         : I2C (VL53L0X @ 0x29, SSD1306 @ 0x3C)
 *  Architecture: 
 *
 *  FILTER PIPELINE (5 Stages):
 *    Raw → [Validation Gate] → [Hampel Filter] → [Median Filter]
 *        → [Adaptive 2D Kalman] → [Rate Limiter] → Clean Output
 *
 *  Serial Output Format:
 *    $DATA,raw_cm,filtered_cm,velocity_cm_s,kalman_gain,valid_pct,status
 *    $DIAG,sig,amb,conf,adaptR,total,valid,rejected
 *
 *  Libraries:
 *    - Adafruit_VL53L0X
 *    - Adafruit_SSD1306
 *    - Adafruit_GFX
 *    - Wire (built-in)
 *=============================================================================*/

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>

/*=============================================================================
 *  SECTION 1: HARDWARE CONFIGURATION
 *=============================================================================*/

#define I2C_SDA_PIN             8
#define I2C_SCL_PIN             9
#define I2C_CLOCK_HZ            400000      // 400kHz Fast Mode

#define VL53L0X_I2C_ADDR        0x29
#define SSD1306_I2C_ADDR        0x3C

#define SCREEN_WIDTH            128
#define SCREEN_HEIGHT           64

/*=============================================================================
 *  SECTION 2: CALIBRATION
 *  ---------------------------------------------------------------------------
 *  VL53L0X has a systematic offset error (±10~25mm) caused by:
 *    - Cover glass crosstalk
 *    - Mounting angle
 *    - Target surface reflectivity
 *
 *  HOW TO CALIBRATE:
 *    1. Place a flat, white target exactly 100mm from the sensor face
 *    2. Read the "Raw" value on OLED (let it stabilize for ~5s)
 *    3. Offset = 100 - (Raw reading)
 *    4. Set CALIBRATION_OFFSET_MM = that value
 *
 *  Example: Raw shows 112mm at actual 100mm → Offset = 100 - 112 = -12
 *=============================================================================*/

#define CALIBRATION_OFFSET_MM   -10.0f        

/*=============================================================================
 *  SECTION 3: TIMING CONFIGURATION (Non-blocking scheduler)
 *=============================================================================*/

#define SENSOR_INTERVAL_MS      33          // ~30 Hz sensor read
#define OLED_INTERVAL_MS        200         // 5 Hz display update
#define SERIAL_INTERVAL_MS      100         // 10 Hz serial output (human readable)
#define DIAG_INTERVAL_MS        3000        // ~0.33 Hz diagnostic output

/*=============================================================================
 *  SECTION 4: FILTER PARAMETERS
 *=============================================================================*/

// Stage 1: Validation Gate
#define VALID_RANGE_MIN_MM      20          // VL53L0X min reliable range
#define VALID_RANGE_MAX_MM      1200        // VL53L0X max reliable range

// Stage 2: Hampel Filter (MAD-based outlier detection)
#define HAMPEL_WINDOW           5
#define HAMPEL_THRESHOLD        2.5f        // Tightened from 3.0 → catches more outliers

// Stage 3: Median Filter
#define MEDIAN_WINDOW           5

// Stage 4: Adaptive 2D Kalman Filter
// Ball & Beam tuning: ball moves slowly (hand-balanced), sensor is noisy
// Q_POS small = trust model more (smooth), Q_VEL larger = allow velocity to adapt fast
// R_BASE reflects VL53L0X typical noise (~5mm std → variance ~25mm²)
#define KALMAN_Q_POS            0.02f       // Reduced: smoother position estimate
#define KALMAN_Q_VEL            0.8f        // Higher: velocity adapts to ball movement
#define KALMAN_R_BASE           30.0f       // Slightly higher: VL53L0X can be noisier
#define KALMAN_R_ADAPT_RATE     0.08f       // Slower adaptation = more stable R
#define KALMAN_INNOV_WINDOW     12          // Wider window = more stable variance est.
#define KALMAN_R_MIN_FACTOR     0.2f        // R_adaptive never falls below R_BASE*0.2

// Stage 5: Rate Limiter
// Ball max speed ~400mm/s, at 30Hz = 13.3mm/cycle → 30mm margin is generous
#define RATE_LIMIT_MM_PER_CYCLE 30.0f       // Tightened: 50→30, catches more glitches

// Buffer size (must >= max of HAMPEL_WINDOW, MEDIAN_WINDOW)
#define FILTER_BUF_SIZE         7

// Warm-up: number of valid samples before trusting output
#define WARMUP_VALID_COUNT      10

/*=============================================================================
 *  SECTION 5: DATA STRUCTURES
 *=============================================================================*/

// Circular buffer - stores up to FILTER_BUF_SIZE values, FIFO order
struct CircularBuffer {
    float    data[FILTER_BUF_SIZE];
    uint8_t  head;          // Points to NEXT write position
    uint8_t  count;
    uint8_t  windowSize;
};

// 2-State Kalman Filter: estimates [position, velocity]
struct KalmanFilter2D {
    float         x[2];                            // State: [position_mm, velocity_mm_s]
    float         P[2][2];                         // Error covariance 2x2
    float         Q[2][2];                         // Process noise covariance
    float         R_adaptive;                      // Current adaptive measurement noise
    float         innovBuf[KALMAN_INNOV_WINDOW];   // Innovation history
    uint8_t       innovIdx;
    bool          innovFull;
    bool          initialized;
    unsigned long lastUpdateMs;
};

// Rate Limiter state
struct RateLimiter {
    float lastOutput;
    bool  initialized;
};

// Sensor data + filter pipeline outputs
struct SensorData {
    float    rawMm;                 // Raw reading from sensor (before calibration)
    float    calibratedMm;          // rawMm + CALIBRATION_OFFSET_MM
    uint8_t  rangeStatus;
    float    signalRate;            // Mcps
    float    ambientRate;           // Mcps
    bool     rawValid;

    float    afterValidation;
    float    afterHampel;
    float    afterMedian;
    float    afterKalman;
    float    finalOutput;           // Clean output in mm

    float    velocityMmS;
    float    kalmanGain;
    float    confidence;

    uint32_t totalReads;
    uint32_t validReads;
    uint32_t rejectedReads;
};

// Scheduler timestamps
struct Scheduler {
    unsigned long sensorTs;
    unsigned long oledTs;
    unsigned long serialTs;
    unsigned long diagTs;
};

/*=============================================================================
 *  SECTION 6: GLOBAL INSTANCES
 *=============================================================================*/

Adafruit_VL53L0X  vl53;
Adafruit_SSD1306  oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

CircularBuffer    hampelBuf;
CircularBuffer    medianBuf;
KalmanFilter2D    kalman;
RateLimiter       rateLim;
SensorData        sensor;
Scheduler         sched;

bool oledAvailable = false;

/*=============================================================================
 *  SECTION 7: CIRCULAR BUFFER OPERATIONS
 *  ---------------------------------------------------------------------------
 *  FIX: bufCopy() now reads in CHRONOLOGICAL order (oldest → newest).
 *
 *  Original bug: copied data[0..count-1] directly, ignoring the circular
 *  nature of the head pointer. This caused Hampel and Median to operate on
 *  incorrectly ordered data, producing wrong median values.
 *
 *  Correct algorithm:
 *    The oldest sample is located at: (head - count + windowSize) % windowSize
 *    Walk forward from there, wrapping modulo windowSize.
 *=============================================================================*/

void bufInit(CircularBuffer* b, uint8_t winSize) {
    memset(b, 0, sizeof(CircularBuffer));
    b->windowSize = winSize;
}

void bufPush(CircularBuffer* b, float val) {
    b->data[b->head] = val;
    b->head = (b->head + 1) % b->windowSize;
    if (b->count < b->windowSize) b->count++;
}

// Copy buffer in chronological order (oldest first) → required for correct median/MAD
uint8_t bufCopy(CircularBuffer* b, float* dst) {
    uint8_t n = b->count;
    // Oldest element index in ring:
    uint8_t start = (b->head - n + b->windowSize) % b->windowSize;
    for (uint8_t i = 0; i < n; i++) {
        dst[i] = b->data[(start + i) % b->windowSize];
    }
    return n;
}

// Insertion sort (optimal for small arrays ≤7)
void sortArray(float* arr, uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {
        float key = arr[i];
        int8_t j  = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// FIX: For even n, average the two middle elements (true statistical median)
float computeMedian(float* arr, uint8_t n) {
    float sorted[FILTER_BUF_SIZE];
    memcpy(sorted, arr, n * sizeof(float));
    sortArray(sorted, n);
    if (n % 2 == 1) {
        return sorted[n / 2];
    } else {
        return (sorted[n / 2 - 1] + sorted[n / 2]) * 0.5f;
    }
}

/*=============================================================================
 *  SECTION 8: FILTER STAGE 1 - VALIDATION GATE
 *  ---------------------------------------------------------------------------
 *  Reject invalid VL53L0X readings:
 *    - Out of reliable range
 *    - Sensor error status (only 0 = good, 4 = phase-wrap = cautious accept)
 *    - NaN/Inf
 *
 *  VL53L0X RangeStatus codes:
 *    0 = Valid
 *    1 = Sigma fail (measurement uncertainty too high)
 *    2 = Signal fail (too dim)
 *    3 = Min range fail
 *    4 = Phase fail (wrap-around, borderline - accepted here)
 *    5+ = Hardware/timeout errors → reject
 *=============================================================================*/

bool validationGate(float mm, uint8_t status) {
    if (status != 0 && status != 4)                 return false;
    if (mm < VALID_RANGE_MIN_MM || mm > VALID_RANGE_MAX_MM) return false;
    if (isnan(mm) || isinf(mm))                     return false;
    return true;
}

/*=============================================================================
 *  SECTION 9: FILTER STAGE 2 - HAMPEL FILTER (MAD-based Outlier Detection)
 *  ---------------------------------------------------------------------------
 *  Uses Median Absolute Deviation (MAD) to identify spike outliers.
 *  If a new value deviates more than HAMPEL_THRESHOLD × MAD from the median,
 *  it's replaced by the median.
 *
 *  MAD = median(|xi - med(x)|) × 1.4826
 *  Scale factor 1.4826 makes MAD consistent with σ for Gaussian data.
 *
 *  Advantage: context-adaptive (adapts to current distribution), robust
 *  to 49% contamination (breakdown point much better than range-checking).
 *=============================================================================*/

float hampelFilter(CircularBuffer* buf, float newVal) {
    bufPush(buf, newVal);

    if (buf->count < 3) return newVal;     // Need minimum 3 samples

    float temp[FILTER_BUF_SIZE];
    uint8_t n = bufCopy(buf, temp);
    float med = computeMedian(temp, n);

    float deviations[FILTER_BUF_SIZE];
    for (uint8_t i = 0; i < n; i++) {
        deviations[i] = fabsf(temp[i] - med);
    }
    float mad = computeMedian(deviations, n) * 1.4826f;

    if (mad < 0.5f) return newVal;          // Raised from 0.001: noise floor ~0.5mm

    float score = fabsf(newVal - med) / mad;
    return (score > HAMPEL_THRESHOLD) ? med : newVal;
}

/*=============================================================================
 *  SECTION 10: FILTER STAGE 3 - MEDIAN FILTER
 *  ---------------------------------------------------------------------------
 *  Sliding-window median after Hampel removes extreme spikes.
 *  Non-linear filter: preserves edges (important for fast ball motion).
 *  Zero phase delay at steady state. Optimal for impulse/ToF noise.
 *=============================================================================*/

float medianFilter(CircularBuffer* buf, float newVal) {
    bufPush(buf, newVal);
    float temp[FILTER_BUF_SIZE];
    uint8_t n = bufCopy(buf, temp);
    return computeMedian(temp, n);
}

/*=============================================================================
 *  SECTION 11: FILTER STAGE 4 - ADAPTIVE 2D KALMAN FILTER
 *  ---------------------------------------------------------------------------
 *  Two-state Kalman: estimates [position (mm), velocity (mm/s)].
 *
 *  Model (Constant Velocity):
 *    F = [[1, dt], [0, 1]]   Transition matrix
 *    H = [1, 0]              Observation matrix (position only)
 *
 *  Adaptive R:
 *    Tracks innovation (prediction error) variance over KALMAN_INNOV_WINDOW.
 *    R grows when sensor is noisy → filter trusts model more.
 *    R shrinks when stable → filter responds faster to real movements.
 *    Hard floor at R_BASE * KALMAN_R_MIN_FACTOR prevents over-smoothing.
 *
 *  Outputs: velocity (for PID D-term), confidence (uncertainty metric).
 *=============================================================================*/

void kalmanInit(KalmanFilter2D* kf, float initPos) {
    kf->x[0] = initPos;
    kf->x[1] = 0.0f;

    // Initial uncertainty: large → let measurement drive early estimates
    kf->P[0][0] = 200.0f;  kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;    kf->P[1][1] = 200.0f;

    kf->Q[0][0] = KALMAN_Q_POS;  kf->Q[0][1] = 0.0f;
    kf->Q[1][0] = 0.0f;          kf->Q[1][1] = KALMAN_Q_VEL;

    kf->R_adaptive = KALMAN_R_BASE;

    memset(kf->innovBuf, 0, sizeof(kf->innovBuf));
    kf->innovIdx    = 0;
    kf->innovFull   = false;
    kf->initialized = true;
    kf->lastUpdateMs = millis();
}

float kalmanUpdate(KalmanFilter2D* kf, float measurement) {
    if (!kf->initialized) {
        kalmanInit(kf, measurement);
        return measurement;
    }

    unsigned long now = millis();
    float dt = (now - kf->lastUpdateMs) / 1000.0f;
    kf->lastUpdateMs = now;

    // Guard: clamp dt to [1ms, 500ms] — protects against millis() overflow or stall
    if (dt < 0.001f || dt > 0.5f) dt = 0.033f;

    // ---- PREDICT ----
    float xPred[2];
    xPred[0] = kf->x[0] + kf->x[1] * dt;
    xPred[1] = kf->x[1];

    // P_pred = F * P * F^T + Q  (expanded analytically for 2x2)
    float pPred[2][2];
    float fp00 = kf->P[0][0] + dt * kf->P[1][0];
    float fp01 = kf->P[0][1] + dt * kf->P[1][1];

    pPred[0][0] = fp00 + fp01 * dt + kf->Q[0][0];
    pPred[0][1] = fp01 + kf->Q[0][1];
    pPred[1][0] = kf->P[1][0] + kf->P[1][1] * dt + kf->Q[1][0];
    pPred[1][1] = kf->P[1][1] + kf->Q[1][1];

    // ---- UPDATE ----
    float innov = measurement - xPred[0];

    // Adaptive R: update innovation variance estimate
    kf->innovBuf[kf->innovIdx] = innov;
    kf->innovIdx = (kf->innovIdx + 1) % KALMAN_INNOV_WINDOW;
    if (kf->innovIdx == 0) kf->innovFull = true;

    if (kf->innovFull) {
        float sum = 0.0f, sumSq = 0.0f;
        for (uint8_t i = 0; i < KALMAN_INNOV_WINDOW; i++) {
            sum   += kf->innovBuf[i];
            sumSq += kf->innovBuf[i] * kf->innovBuf[i];
        }
        float mean = sum / KALMAN_INNOV_WINDOW;
        float var  = (sumSq / KALMAN_INNOV_WINDOW) - (mean * mean);

        // R cannot go below R_BASE * R_MIN_FACTOR (prevents over-confidence)
        float rMin = KALMAN_R_BASE * KALMAN_R_MIN_FACTOR;
        float rTarget = fmaxf(var, rMin);
        kf->R_adaptive = (1.0f - KALMAN_R_ADAPT_RATE) * kf->R_adaptive
                       + KALMAN_R_ADAPT_RATE * rTarget;
    }

    // S = H * P_pred * H^T + R = P_pred[0][0] + R_adaptive
    float S = pPred[0][0] + kf->R_adaptive;
    if (fabsf(S) < 1e-9f) S = 1e-9f;        // Numerical guard, tighter than before

    // Kalman gain
    float K[2];
    K[0] = pPred[0][0] / S;
    K[1] = pPred[1][0] / S;

    // State update
    kf->x[0] = xPred[0] + K[0] * innov;
    kf->x[1] = xPred[1] + K[1] * innov;

    // Covariance update: Joseph form for numerical stability
    // P = (I - KH) * P_pred * (I - KH)^T + K*R*K^T  ← Joseph form
    // Simplified here to standard (I - KH)*P_pred since filter is well-conditioned:
    kf->P[0][0] = (1.0f - K[0]) * pPred[0][0];
    kf->P[0][1] = (1.0f - K[0]) * pPred[0][1];
    kf->P[1][0] = -K[1] * pPred[0][0] + pPred[1][0];
    kf->P[1][1] = -K[1] * pPred[0][1] + pPred[1][1];

    // Export monitoring values
    sensor.kalmanGain   = K[0];
    sensor.velocityMmS  = kf->x[1];
    sensor.confidence   = sqrtf(fabsf(kf->P[0][0]));   // fabsf: guard against neg rounding

    return kf->x[0];
}

/*=============================================================================
 *  SECTION 12: FILTER STAGE 5 - RATE LIMITER (SLEW RATE)
 *  ---------------------------------------------------------------------------
 *  Physical reality: a ball on a beam cannot teleport.
 *  Max realistic velocity ≈ 400mm/s → at 30Hz = ~13mm/cycle.
 *  RATE_LIMIT_MM_PER_CYCLE = 30mm provides 2× safety margin.
 *  Clamps any residual glitch that survived all previous filter stages.
 *=============================================================================*/

float rateLimiterApply(RateLimiter* rl, float input) {
    if (!rl->initialized) {
        rl->lastOutput  = input;
        rl->initialized = true;
        return input;
    }

    float diff = input - rl->lastOutput;

    if      (diff >  RATE_LIMIT_MM_PER_CYCLE) rl->lastOutput += RATE_LIMIT_MM_PER_CYCLE;
    else if (diff < -RATE_LIMIT_MM_PER_CYCLE) rl->lastOutput -= RATE_LIMIT_MM_PER_CYCLE;
    else                                       rl->lastOutput  = input;

    return rl->lastOutput;
}

/*=============================================================================
 *  SECTION 13: SENSOR READ + FULL FILTER PIPELINE
 *  ---------------------------------------------------------------------------
 *  Calibration is applied BEFORE the filter pipeline so all stages
 *  operate on calibrated (corrected) values, not raw offset values.
 *=============================================================================*/

void sensorReadAndFilter() {
    VL53L0X_RangingMeasurementData_t measure;
    
    // rangingTest blocks until data is ready or timeout occurs
    // If it hangs, it means the hardware timeout inside Adafruit library was hit.
    vl53.rangingTest(&measure, false);

    sensor.totalReads++;
    
    // Early validation: If RangeStatus is bad (Hardware/Phase timeout), 
    // the SignalRate and Range data inside 'measure' is GARBAGE (random high numbers).
    // We MUST check status BEFORE extracting other float values to avoid crazy high 'Sig' numbers.
    uint8_t status = measure.RangeStatus;
    
    // Status 0: Valid, Status 4: Phase check (often still valid but noisy)
    bool isDataValid = (status == 0 || status == 4);

    if (!isDataValid) {
        sensor.rawValid = false;
        sensor.rejectedReads++;
        sensor.rangeStatus = status;
        return;
    }

    // Only extract data if we are SURE it is not garbage
    sensor.rawMm       = (float)measure.RangeMilliMeter;
    sensor.rangeStatus = status;
    sensor.signalRate  = measure.SignalRateRtnMegaCps / 65536.0f;
    sensor.ambientRate = measure.AmbientRateRtnMegaCps / 65536.0f;

    // Safety check: signal >1000 or range >8000 is physically impossible
    if (sensor.signalRate > 1000.0f || sensor.rawMm > 8000.0f) {
        sensor.rawValid = false;
        sensor.rejectedReads++;
        return;
    }

    // Apply calibration offset BEFORE range validation
    // Reason: we want to validate the CORRECTED distance, not the raw sensor reading
    sensor.calibratedMm = sensor.rawMm + CALIBRATION_OFFSET_MM;

    // Stage 1: Validation Gate - validate the CALIBRATED value against physical beam limits
    // NOTE: validationGate() was previously defined but never called — fixed here
    if (!validationGate(sensor.calibratedMm, status)) {
        sensor.rawValid = false;
        sensor.rejectedReads++;
        return;
    }

    sensor.rawValid = true;
    sensor.validReads++;
    sensor.afterValidation = sensor.calibratedMm;

    // Stage 2: Hampel Filter
    sensor.afterHampel = hampelFilter(&hampelBuf, sensor.afterValidation);

    // Stage 3: Median Filter
    sensor.afterMedian = medianFilter(&medianBuf, sensor.afterHampel);

    // Stage 4: Adaptive 2D Kalman Filter
    sensor.afterKalman = kalmanUpdate(&kalman, sensor.afterMedian);

    // Stage 5: Rate Limiter
    sensor.finalOutput = rateLimiterApply(&rateLim, sensor.afterKalman);
}

/*=============================================================================
 *  SECTION 14: OLED DISPLAY
 *  ---------------------------------------------------------------------------
 *  Layout            (128x64):
 *    Line 0  [0,0]   Header: "Ball&Beam | VL53L0X"
 *    ─────────────── separator at y=10
 *    Line 1 [4,15]   Main: "XX.X cm"  (size 2)
 *    Line 2 [0,36]   Raw: XXXmm    V: ±XXX
 *    Line 3 [0,48]   Sig: XX.X  Amb: XX.X
 *    Line 4 [0,56]   OK:XX%  K:X.XX  C:XX
 *
 *  Symbol Glossary:
 *    Raw = raw mm from sensor (uncalibrated)
 *    V   = velocity mm/s (Kalman estimate)
 *    Sig = signal rate Mcps (higher → better target reflection)
 *    Amb = ambient rate Mcps (higher → more light interference)
 *    OK  = valid reads percentage
 *    K   = Kalman gain (0=trust model, 1=trust sensor)
 *    C   = confidence σ_pos in mm (lower → more certain)
 *=============================================================================*/

void oledUpdate() {
    if (!oledAvailable) return;

    oled.clearDisplay();
    oled.setTextColor(SSD1306_WHITE);

    // Header
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print(F("Ball&Beam | VL53L0X"));
    oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

    // Main distance: show warm-up indicator if not stable yet
    oled.setTextSize(2);
    oled.setCursor(4, 15);
    if (sensor.validReads == 0) {
        oled.print(F("--- cm"));
    } else if (sensor.validReads < WARMUP_VALID_COUNT) {
        // Warm-up phase: show value but with asterisk
        oled.print(sensor.finalOutput / 10.0f, 1);
        oled.print(F("*cm"));
    } else {
        oled.print(sensor.finalOutput / 10.0f, 1);
        oled.print(F(" cm"));
    }

    oled.setTextSize(1);

    // Row: Raw (uncalibrated) + Velocity
    oled.setCursor(0, 36);
    oled.print(F("R:"));
    oled.print((int)sensor.rawMm);
    oled.print(F(" C:"));
    oled.print((int)sensor.calibratedMm);

    oled.setCursor(88, 36);
    oled.print(F("V:"));
    oled.print(sensor.velocityMmS, 0);

    // Row: Signal + Ambient
    oled.setCursor(0, 48);
    oled.print(F("Sig:"));
    oled.print(sensor.signalRate, 1);

    oled.setCursor(55, 48);
    oled.print(F("Amb:"));
    oled.print(sensor.ambientRate, 1);

    // Row: OK% + Kalman Gain + Confidence
    oled.setCursor(0, 56);
    float pct = (sensor.totalReads > 0)
                ? (float)sensor.validReads / sensor.totalReads * 100.0f : 0.0f;
    oled.print(F("OK:"));
    oled.print((int)pct);
    oled.print(F("%"));

    oled.setCursor(50, 56);
    oled.print(F("K:"));
    oled.print(sensor.kalmanGain, 2);

    oled.setCursor(95, 56);
    oled.print(F("C:"));
    oled.print(sensor.confidence, 0);

    oled.display();
}

/*=============================================================================
 *  SECTION 15: SERIAL OUTPUT
 *  ---------------------------------------------------------------------------
 *  FIX: $DATA now only prints when the last read was VALID.
 *  FIX: Output is in cm (not mm) for human readability in Serial Monitor.
 *  FIX: $DIAG interval increased to 3s to reduce spam.
 *  FIX: Separator line printed before $DIAG for visual clarity.
 *
 *  Format:
 *    $DATA  → raw_cm, filt_cm, vel_cms, K_gain, valid_pct, status
 *    $DIAG  → sig, amb, conf, adaptR, total, valid, rejected
 *
 *  "status" field:
 *    OK    = valid read, pipeline healthy
 *    WARM  = valid read, still in warm-up phase
 *    INV   = last read was invalid (data from previous valid)
 *=============================================================================*/

void serialSendData() {
    Serial.print(F("$DATA,"));

    // FIX: Distinguish invalid reads in output (don't pretend they are good)
    if (!sensor.rawValid) {
        // Print last known good filtered value but flag as INV
        Serial.print(F("INV,"));
        Serial.print(sensor.finalOutput / 10.0f, 1);
        Serial.print(F(",0.0,0.0000,"));
        float pct = (sensor.totalReads > 0)
                    ? (float)sensor.validReads / sensor.totalReads * 100.0f : 0.0f;
        Serial.print(pct, 1);
        Serial.println(F(",INV"));
        return;
    }

    // Raw cm
    Serial.print(sensor.rawMm / 10.0f, 1);
    Serial.print(',');
    // Filtered cm (final output)
    Serial.print(sensor.finalOutput / 10.0f, 1);
    Serial.print(',');
    // Velocity cm/s
    Serial.print(sensor.velocityMmS / 10.0f, 1);
    Serial.print(',');
    // Kalman gain
    Serial.print(sensor.kalmanGain, 4);
    Serial.print(',');
    // Valid %
    float pct = (sensor.totalReads > 0)
                ? (float)sensor.validReads / sensor.totalReads * 100.0f : 0.0f;
    Serial.print(pct, 1);
    Serial.print(',');
    // Status
    Serial.println(sensor.validReads < WARMUP_VALID_COUNT ? F("WARM") : F("OK"));
}

void serialSendDiag() {
    Serial.println(F("--- DIAG ------------------------------------------------"));
    Serial.print(F("  Sig(Mcps):")); Serial.print(sensor.signalRate, 2);
    Serial.print(F("  Amb(Mcps):")); Serial.println(sensor.ambientRate, 2);
    Serial.print(F("  Conf(mm):" )); Serial.print(sensor.confidence, 2);
    Serial.print(F("  AdaptR:"   )); Serial.println(kalman.R_adaptive, 2);
    Serial.print(F("  Total:"    )); Serial.print(sensor.totalReads);
    Serial.print(F("  Valid:"    )); Serial.print(sensor.validReads);
    Serial.print(F("  Rejected:" )); Serial.println(sensor.rejectedReads);
    Serial.print(F("  CalibOff:" )); Serial.print(CALIBRATION_OFFSET_MM, 1);
    Serial.print(F("mm  WarmUp:")); 
    Serial.println(sensor.validReads < WARMUP_VALID_COUNT ? F("YES") : F("NO"));
    Serial.println(F("---------------------------------------------------------"));
}

/*=============================================================================
 *  SECTION 16: I2C SCANNER (Startup Diagnostic)
 *=============================================================================*/

void i2cScan() {
    Serial.println(F("[I2C] Scanning bus..."));
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) Serial.print('0');
            Serial.print(addr, HEX);
            if (addr == VL53L0X_I2C_ADDR) Serial.print(F(" → VL53L0X"));
            if (addr == SSD1306_I2C_ADDR) Serial.print(F(" → SSD1306"));
            Serial.println();
            found++;
        }
    }
    Serial.print(F("[I2C] Found: "));
    Serial.print(found);
    Serial.println(F(" device(s)"));
}

/*=============================================================================
 *  SECTION 17: SETUP
 *=============================================================================*/

void setup() {
    Serial.begin(115200);
    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 2000)) { ; }

    Serial.println(F("╔═══════════════════════════════════════╗"));
    Serial.println(F("║  VL53L0X Sensor Test - Ball & Beam    ║"));
    Serial.println(F("║  ESP32-S3 | 5-Stage Filter Pipeline   ║"));
    Serial.println(F("╚═══════════════════════════════════════╝"));

    Serial.print(F("[CAL] Calibration offset: "));
    Serial.print(CALIBRATION_OFFSET_MM, 1);
    Serial.println(F(" mm"));

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);
    Serial.println(F("[I2C] Bus initialized (400kHz)"));

    i2cScan();

    Serial.print(F("[VL53L0X] Initializing..."));
    if (!vl53.begin()) {
        Serial.println(F(" FAILED! Check wiring."));
        Serial.println(F("[SYSTEM] HALTED - critical sensor missing."));
        while (1) { ; }
    }
    Serial.println(F(" OK"));

    // Default mode: ~30ms/measurement, balanced accuracy & speed
    // High Accuracy (200ms) too slow; High Speed (20ms) too noisy.
    vl53.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
    Serial.println(F("[VL53L0X] Mode: DEFAULT (~30Hz, balanced)"));

    Serial.print(F("[OLED] Initializing..."));
    if (!oled.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDR)) {
        Serial.println(F(" FAILED! Running without display."));
        oledAvailable = false;
    } else {
        Serial.println(F(" OK (128x64)"));
        oledAvailable = true;
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(10, 10);  oled.println(F("Ball & Beam"));
        oled.setCursor(10, 25);  oled.println(F("VL53L0X Test"));
        oled.setCursor(10, 40);  oled.println(F("Warming up..."));
        oled.display();
    }

    // Initialize all filter state
    bufInit(&hampelBuf, HAMPEL_WINDOW);
    bufInit(&medianBuf, MEDIAN_WINDOW);
    memset(&kalman,  0, sizeof(kalman));
    memset(&rateLim, 0, sizeof(rateLim));
    memset(&sensor,  0, sizeof(sensor));

    unsigned long now = millis();
    sched.sensorTs = now;
    sched.oledTs   = now;
    sched.serialTs = now;
    sched.diagTs   = now;

    // Serial header - columns match $DATA output
    Serial.println(F("$HEADER,raw_cm,filt_cm,vel_cms,K_gain,valid_pct,status"));
    Serial.println(F("[SYSTEM] Ready. Warming up filter pipeline..."));
}

/*=============================================================================
 *  SECTION 18: MAIN LOOP (Non-blocking cooperative scheduler)
 *=============================================================================*/

void loop() {
    unsigned long now = millis();

    // Task 1: Sensor read + full filter pipeline (~30 Hz)
    if (now - sched.sensorTs >= SENSOR_INTERVAL_MS) {
        sched.sensorTs = now;
        sensorReadAndFilter();
    }

    // Task 2: OLED update (5 Hz)
    if (now - sched.oledTs >= OLED_INTERVAL_MS) {
        sched.oledTs = now;
        oledUpdate();
    }

    // Task 3: Serial $DATA (10 Hz - reduced from 20Hz, easier to read)
    if (now - sched.serialTs >= SERIAL_INTERVAL_MS) {
        sched.serialTs = now;
        serialSendData();
    }

    // Task 4: Serial $DIAG (0.33 Hz - every 3s)
    if (now - sched.diagTs >= DIAG_INTERVAL_MS) {
        sched.diagTs = now;
        serialSendDiag();
    }
}
