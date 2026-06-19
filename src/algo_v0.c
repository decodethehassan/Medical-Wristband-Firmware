#include "algo_v0.h"
#include "ppg_validation.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#include <stdlib.h>

#if PPG_VALIDATION_MODE || PPG_FULLFW_PPG_STREAM_ENABLE
#include "ble_log_service.h"
#endif

LOG_MODULE_REGISTER(algo_v0, LOG_LEVEL_INF);

/* =========================
 * Version Zero parameters (from Algo V0 doc)
 * - Fast loop: 5 seconds
 * - Slow loop: 60 seconds
 * ========================= */
#define FAST_LOOP_SEC               5
#define SLOW_LOOP_SEC               60
#define FAST_WINDOWS_PER_MIN        12
#define HR_BLOCKS_PER_MIN           6
#define HR_BLOCK_SEC                10

/* Expected sampling rates per doc (defaults) */
#define ACC_FS_HZ_DEFAULT           32
#define PPG_FS_HZ_DEFAULT           100
#define EDA_FS_HZ_DEFAULT           4
#define TEMP_FS_HZ_DEFAULT          4

/* --- Artifact thresholds --- */
#define TH_ACCEL_VAR_G2             0.02f
#define TH_SMA_G                    0.20f
#define TH_PPG_SQI                  0.50f
#define TH_CLIP_FRAC                0.05f

/* MAX30101 is 18-bit raw */
#define PPG_ADC_MIN                 0u
#define PPG_ADC_MAX                 0x3FFFFu
#define PPG_SAT_EPS                 1u

/* --- EDA QC (doc is in µS) --- */
#define EDA_MIN_US                  0.0f
#define EDA_MAX_US                  100.0f
#define EDA_MAX_DELTA_US            3.0f
#define EDA_FLAT_DELTA_US           0.001f
#define EDA_FLAT_TIME_SEC           5

/* --- Temp QC --- */
#define TEMP_MIN_C                  20.0f
#define TEMP_MAX_C                  40.0f
#define TEMP_MAX_DPERMIN_C          0.2f
#define TEMP_SLOPE_HIST_MIN         5

/* --- Activity thresholds (median SMA over minute) --- */
#define SMA_REST_MAX_G              0.05f
#define SMA_LOW_MAX_G               0.15f
#define SMA_WALK_MAX_G              0.40f

/* --- HRV gating --- */
#define HRV_MIN_CONT_CLEAN_SEC      20

/* --- Beat detection --- */
#define PEAK_REFRACTORY_MS          600
#define IBI_MIN_MS                  600
#define IBI_MAX_MS                  1500
#define IBI_OUTLIER_FRAC            0.20f
#define IBI_MAX_PER_MIN             512
#define IBI_MAX_PER_10S             64
#define ACC_MIN_SAMPLES_5S          8
#define PPG_MIN_SAMPLES_5S          8

/* --- Contact placement / optical transient protection --- */
#define PPG_RAW_STEP_RESET_ADC      1500.0f
#define PPG_SETTLE_AFTER_RESET_MS   3000

/* =========================
 * Internal state
 * ========================= */

static algo_v0_minute_cb_t g_minute_cb;

/* If you want, you can tune this without touching sensor tasks */
static float g_eda_mv_to_uS = 1.0f;

/* Sampling-rate trackers (estimated from timestamps) */
static float g_acc_fs_hz = ACC_FS_HZ_DEFAULT;
static float g_ppg_fs_hz = PPG_FS_HZ_DEFAULT;
static float g_eda_fs_hz = EDA_FS_HZ_DEFAULT;
static float g_temp_fs_hz = TEMP_FS_HZ_DEFAULT;

/* ---------- helpers ---------- */
static inline float f_abs(float x) { return x < 0.0f ? -x : x; }
static inline float clamp01(float x) { return MIN(1.0f, MAX(0.0f, x)); }

static int cmp_float(const void *a, const void *b)
{
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

static float median_float(float *arr, int n)
{
    if (n <= 0) return NAN;
    qsort(arr, n, sizeof(float), cmp_float);
    if (n & 1) return arr[n / 2];
    return 0.5f * (arr[n / 2 - 1] + arr[n / 2]);
}

static float mean_float(const float *arr, int n)
{
    if (n <= 0) return NAN;
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += arr[i];
    return sum / (float)n;
}

static float std_float(const float *arr, int n)
{
    if (n <= 1) return NAN;
    float mu = mean_float(arr, n);
    float acc = 0.0f;
    for (int i = 0; i < n; i++) {
        float d = arr[i] - mu;
        acc += d * d;
    }
    return sqrtf(acc / (float)(n - 1));
}

static float one_pole_alpha(float fc_hz, float fs_hz)
{
    float fs = MAX(fs_hz, 0.1f);
    float dt = 1.0f / fs;
    float rc = 1.0f / (2.0f * (float)M_PI * MAX(fc_hz, 0.0001f));
    return dt / (rc + dt);
}

static void lowpass_sequence(const float *in, float *out, int n, float fc_hz, float fs_hz)
{
    if (n <= 0) return;

    float a = one_pole_alpha(fc_hz, fs_hz);
    out[0] = in[0];
    for (int i = 1; i < n; i++) {
        out[i] = out[i - 1] + a * (in[i] - out[i - 1]);
    }
}

/* Running variance via Welford */
typedef struct {
    int32_t n;
    float mean;
    float m2;
} welford_t;

static void wf_reset(welford_t *w) { memset(w, 0, sizeof(*w)); }

static void wf_push(welford_t *w, float x)
{
    w->n++;
    float d = x - w->mean;
    w->mean += d / (float)w->n;
    float d2 = x - w->mean;
    w->m2 += d * d2;
}

static float wf_var(const welford_t *w)
{
    if (w->n < 2) return 0.0f;
    return w->m2 / (float)(w->n - 1);
}

/* =========================
 * Buffers for 5s and 60s logic
 * ========================= */

/* 5s window accumulators from ACC
 * NOTE:
 * The document writes formulas using ax/ay/az directly, but with raw accelerometer
 * data the 0.20 g SMA threshold becomes impractical because gravity dominates.
 * So we use body/linear acceleration (gravity removed) before applying the document
 * thresholds. This keeps the V0 rules usable on the current sensor feed.
 */
static welford_t g_mag_wf_5s;
static float     g_sma_sum_5s;
static int       g_acc_count_5s;

/* Track last ACC timestamp to estimate fs */
static int64_t   g_last_acc_t = -1;

/* Minute storage: 12 slots of 5s */
static uint8_t   g_artifact_5s[FAST_WINDOWS_PER_MIN];
static uint8_t   g_acc_valid_5s[FAST_WINDOWS_PER_MIN];
static uint8_t   g_ppg_valid_5s[FAST_WINDOWS_PER_MIN];
static float     g_sma_5s[FAST_WINDOWS_PER_MIN];
static float     g_ppg_sqi_5s[FAST_WINDOWS_PER_MIN];
static uint8_t   g_ppg_sat_5s[FAST_WINDOWS_PER_MIN];
static float     g_clip_frac_5s[FAST_WINDOWS_PER_MIN];
static int       g_5s_idx;

/* PPG quality accumulators for current 5s */
static bool      g_ppg_saturation_5s;
static uint32_t  g_ppg_clip_cnt_5s;
static uint32_t  g_ppg_total_cnt_5s;
static float     g_ppg_energy_band_5s;
static float     g_ppg_energy_total_5s;

/* Beat detection state on IR channel */
static int64_t   g_last_ppg_t = -1;
static float     g_ppg_hp;
static float     g_ppg_lp;
static float     g_ppg_mean_ewma;
static float     g_ppg_var_ewma;
static float     g_ppg_x_prev;
static float     g_ppg_y_prev1;
static float     g_ppg_y_prev2;
static int64_t   g_ppg_t_prev1 = -1;
static int64_t   g_ppg_t_prev2 = -1;
static int64_t   g_last_peak_t = -1000000;

#if PPG_VALIDATION_MODE || PPG_FULLFW_PPG_STREAM_ENABLE
static uint32_t  g_ppg_val_seq;
#endif

/* Last completed 5-second PPG quality gate.
 * qok=1 means the last PPG window passed SQI/motion/clipping checks.
 * Live HR/IBI reporting is suppressed when qok=0 so false peaks do not look valid.
 */
static bool      g_last_ppg_quality_ok = false;
static float     g_last_ppg_sqi_live = NAN;
static bool      g_last_ppg_artifact_live = true;

/* Contact/placement transient handling.
 * A sudden raw jump happens when finger/wrist first touches the optical window.
 * Resetting the filter and ignoring peaks during settling prevents huge spikes
 * from becoming fake HR/IBI values or destabilizing BLE streaming.
 */
static float     g_ppg_last_raw = NAN;
static int64_t   g_ppg_ignore_until_ms = 0;

/* Store clean IBIs during the current minute, plus their end timestamps */
static uint16_t  g_ibi_ms[IBI_MAX_PER_MIN];
static int64_t   g_ibi_t_ms[IBI_MAX_PER_MIN];
static int       g_ibi_n;

/* Continuous IBI QC state (must persist across minute boundaries) */
static uint16_t  g_last5_ibi[5];
static int       g_last5_n;

/* HR 10s subwindows (6 blocks) */
static float     g_hr10[HR_BLOCKS_PER_MIN];
static bool      g_hr10_valid[HR_BLOCKS_PER_MIN];

/* EDA minute buffers */
#define EDA_MAX_SAMPLES_PER_MIN  512
static float     g_eda_uS[EDA_MAX_SAMPLES_PER_MIN];
static float     g_eda_clean[EDA_MAX_SAMPLES_PER_MIN];
static float     g_eda_1hz[EDA_MAX_SAMPLES_PER_MIN];
static float     g_eda_tonic[EDA_MAX_SAMPLES_PER_MIN];
static float     g_eda_phasic[EDA_MAX_SAMPLES_PER_MIN];
static int       g_eda_n;
static bool      g_eda_flatline;
static int       g_eda_flat_cnt;
static int64_t   g_last_eda_t = -1;
static float     g_last_eda_uS = NAN;

/* Temp minute buffers */
#define TEMP_MAX_SAMPLES_PER_MIN  256
static float     g_temp_c[TEMP_MAX_SAMPLES_PER_MIN];
static float     g_temp_scratch[TEMP_MAX_SAMPLES_PER_MIN];
static int       g_temp_n;

/* For temp slope QC */
static float     g_last_temp_c = NAN;
static int64_t   g_last_temp_t = -1;

/* Rolling 5-minute temp feature */
static float     g_temp60_hist[TEMP_SLOPE_HIST_MIN];
static int64_t   g_temp60_hist_t[TEMP_SLOPE_HIST_MIN];
static int       g_temp60_hist_n;

/* Minute counters */
static int64_t   g_minute_start_t = -1;
static int64_t   g_next_fast_t = -1;
static int64_t   g_next_slow_t = -1;

static bool      g_seen_imu;
static bool      g_seen_ppg;
static bool      g_algo_armed;

/* Sleep state */
static int       g_sleep_restlow_streak_min;
static int       g_sleep_wake_streak_min;
static bool      g_sleep_likely;

/* Gravity removal state */
#define GRAV_TAU_SEC  1.0f
static float g_gx = 0.0f, g_gy = 0.0f, g_gz = 1.0f;

/* Activity enum */
typedef enum {
    ACT_REST = 0,
    ACT_LOW,
    ACT_WALK,
    ACT_VIG
} activity_t;

static const char *act_str(activity_t a)
{
    switch (a) {
    case ACT_REST: return "REST";
    case ACT_LOW:  return "LOW";
    case ACT_WALK: return "WALK";
    default:       return "VIG";
    }
}

/* =========================
 * Public setters
 * ========================= */
void algo_v0_set_eda_mv_to_uS(float scale)
{
    g_eda_mv_to_uS = (scale > 0.0f) ? scale : 1.0f;
}

void algo_v0_set_minute_callback(algo_v0_minute_cb_t cb)
{
    g_minute_cb = cb;
}



static void ppg_reset_filter_state(float x, int64_t t_ms)
{
    g_ppg_hp = 0.0f;
    g_ppg_lp = 0.0f;
    g_ppg_mean_ewma = 0.0f;
    g_ppg_var_ewma = 1.0f;
    g_ppg_x_prev = x;

    g_ppg_y_prev1 = 0.0f;
    g_ppg_y_prev2 = 0.0f;
    g_ppg_t_prev1 = -1;
    g_ppg_t_prev2 = -1;

    g_last_peak_t = -1000000;
    g_last5_n = 0;
    memset(g_last5_ibi, 0, sizeof(g_last5_ibi));

    /* Do not let old/invalid IBIs remain in the current minute after a contact event. */
    g_ibi_n = 0;
    memset(g_ibi_ms, 0, sizeof(g_ibi_ms));
    memset(g_ibi_t_ms, 0, sizeof(g_ibi_t_ms));

    /* Clear PPG-only 5-second quality accumulators after the transient. */
    g_ppg_saturation_5s = false;
    g_ppg_clip_cnt_5s = 0;
    g_ppg_total_cnt_5s = 0;
    g_ppg_energy_band_5s = 0.0f;
    g_ppg_energy_total_5s = 0.0f;

    /* Force quality gate closed until the next clean completed window. */
    g_last_ppg_quality_ok = false;
    g_last_ppg_sqi_live = NAN;
    g_last_ppg_artifact_live = true;

    g_ppg_ignore_until_ms = t_ms + PPG_SETTLE_AFTER_RESET_MS;
}

#if PPG_VALIDATION_MODE || PPG_FULLFW_PPG_STREAM_ENABLE
static void ppg_stream_send_sample(int64_t t_ms,
                                   uint32_t seq,
                                   uint32_t ppg_raw,
                                   uint32_t green_raw,
                                   float filt,
                                   float th,
                                   int peak,
                                   int32_t ibi_ms,
                                   float hr_inst,
                                   float fs_hz)
{
    if (!ble_log_is_ready()) {
        return;
    }

#if PPG_VALIDATION_MODE
    const uint32_t stream_div = PPG_VAL_STREAM_DIV;
#else
    const uint32_t stream_div = PPG_FULLFW_PPG_STREAM_DIV;
#endif

    /* Keep bandwidth manageable, but always send accepted peak samples. */
    if (((seq % stream_div) != 0U) && (peak == 0)) {
        return;
    }

    int settling = (t_ms < g_ppg_ignore_until_ms) ? 1 : 0;

    char line[320];
    int n = snprintk(line, sizeof(line),
#if PPG_VALIDATION_MODE
        "PV,t=%lld,seq=%u,raw=%u,green=%u,filt=%.2f,th=%.2f,peak=%d,ibi=%ld,hr=%.1f,fs=%.1f,qok=%d,sqi=%.3f,art=%d,settle=%d\r\n",
#else
        "PPG_STREAM,t=%lld,seq=%u,raw=%u,green=%u,filt=%.2f,th=%.2f,peak=%d,ibi=%ld,hr=%.1f,fs=%.1f,qok=%d,sqi=%.3f,art=%d,settle=%d\r\n",
#endif
        t_ms,
        seq,
        ppg_raw,
        green_raw,
        (double)filt,
        (double)th,
        peak,
        (long)ibi_ms,
        (double)hr_inst,
        (double)fs_hz,
        g_last_ppg_quality_ok ? 1 : 0,
        isnan(g_last_ppg_sqi_live) ? -1.0 : (double)g_last_ppg_sqi_live,
        g_last_ppg_artifact_live ? 1 : 0,
        settling);

    if (n > 0) {
        (void)ble_log_send_as((const uint8_t *)line, (size_t)n);
    }
}
#endif

/* =========================
 * Push functions (called from sensor tasks)
 * ========================= */

void algo_v0_push_temp_c(float temp_c, int64_t t_ms)
{
    if (g_last_temp_t > 0) {
        int64_t dt = t_ms - g_last_temp_t;
        if (dt > 0) {
            float fs = 1000.0f / (float)dt;
            g_temp_fs_hz = 0.9f * g_temp_fs_hz + 0.1f * fs;
        }
    }
    g_last_temp_t = t_ms;

    if (g_temp_n < (int)ARRAY_SIZE(g_temp_c)) {
        g_temp_c[g_temp_n++] = temp_c;
    }
}

void algo_v0_push_imu(float ax_g, float ay_g, float az_g,
                      float gx_dps, float gy_dps, float gz_dps,
                      int64_t t_ms)
{
    ARG_UNUSED(gx_dps);
    ARG_UNUSED(gy_dps);
    ARG_UNUSED(gz_dps);

    g_seen_imu = true;

    if (g_last_acc_t > 0) {
        int64_t dt_ms = t_ms - g_last_acc_t;
        if (dt_ms > 0) {
            float fs = 1000.0f / (float)dt_ms;
            g_acc_fs_hz = 0.9f * g_acc_fs_hz + 0.1f * fs;
        }
    }
    g_last_acc_t = t_ms;

    /* gravity estimate (LPF) and linear accel */
    float dt = 1.0f / MAX(g_acc_fs_hz, 10.0f);
    float alpha = dt / (GRAV_TAU_SEC + dt);

    g_gx = g_gx + alpha * (ax_g - g_gx);
    g_gy = g_gy + alpha * (ay_g - g_gy);
    g_gz = g_gz + alpha * (az_g - g_gz);

    float lx = ax_g - g_gx;
    float ly = ay_g - g_gy;
    float lz = az_g - g_gz;

    float mag_lin = sqrtf(lx * lx + ly * ly + lz * lz);
    wf_push(&g_mag_wf_5s, mag_lin);

    g_sma_sum_5s += (f_abs(lx) + f_abs(ly) + f_abs(lz));
    g_acc_count_5s++;
}

void algo_v0_push_ppg(uint32_t red, uint32_t ir, uint32_t green, int64_t t_ms)
{
    ARG_UNUSED(red);

    g_seen_ppg = true;

#if PPG_VALIDATION_MODE
    /* In validation mode, use the true MAX30101 green channel for PPG. */
    uint32_t ppg_raw = green;
    ARG_UNUSED(ir);
#else
    /* Normal firmware path: the selected PPG signal is passed in the IR argument
     * for backward compatibility with the existing task interface.
     */
    uint32_t ppg_raw = ir;
    ARG_UNUSED(green);
#endif

    g_ppg_total_cnt_5s++;
    if (ppg_raw >= (PPG_ADC_MAX - PPG_SAT_EPS) || ppg_raw <= (PPG_ADC_MIN + PPG_SAT_EPS)) {
        g_ppg_saturation_5s = true;
        g_ppg_clip_cnt_5s++;
    }

    if (g_last_ppg_t > 0) {
        int64_t dt_ms = t_ms - g_last_ppg_t;
        if (dt_ms > 0) {
            float fs = 1000.0f / (float)dt_ms;
            g_ppg_fs_hz = 0.9f * g_ppg_fs_hz + 0.1f * fs;
        }
    }
    g_last_ppg_t = t_ms;

    float x = (float)ppg_raw;

    /* Contact-placement protection:
     * Finger/wrist contact can cause a large raw DC step. Without this reset,
     * the HP/LP filters produce a huge transient that looks like a pulse and can
     * create false HR/IBI or BLE instability from noisy peak bursts.
     */
    if (isnan(g_ppg_last_raw)) {
        g_ppg_last_raw = x;
        ppg_reset_filter_state(x, t_ms);
    } else {
        float raw_step = f_abs(x - g_ppg_last_raw);
        g_ppg_last_raw = x;

        if (raw_step > PPG_RAW_STEP_RESET_ADC) {
            ppg_reset_filter_state(x, t_ms);
        }
    }

    float dt = 1.0f / MAX(g_ppg_fs_hz, 10.0f);

    /* Band-pass approximation: HP 0.5 Hz then LP 5 Hz. */
    float rc_hp = 1.0f / (2.0f * (float)M_PI * 0.5f);
    float a_hp = rc_hp / (rc_hp + dt);
    g_ppg_hp = a_hp * (g_ppg_hp + x - g_ppg_x_prev);
    g_ppg_x_prev = x;

    float rc_lp = 1.0f / (2.0f * (float)M_PI * 5.0f);
    float a_lp = dt / (rc_lp + dt);
    g_ppg_lp = g_ppg_lp + a_lp * (g_ppg_hp - g_ppg_lp);

    float y = g_ppg_lp;

    /* SQI proxy: band power / total power in filtered non-DC signal. */
    g_ppg_energy_band_5s  += (y * y);
    g_ppg_energy_total_5s += (g_ppg_hp * g_ppg_hp);

    /* Rolling ~3 s mean/std. */
    float alpha = dt / (3.0f + dt);
    float d = y - g_ppg_mean_ewma;
    g_ppg_mean_ewma += alpha * d;
    g_ppg_var_ewma  = (1.0f - alpha) * (g_ppg_var_ewma + alpha * d * d);
    float sigma = sqrtf(MAX(g_ppg_var_ewma, 1.0f));
    float th = g_ppg_mean_ewma + 0.82f * sigma;

    bool settling = (t_ms < g_ppg_ignore_until_ms);

    bool is_peak = !settling &&
                   (g_ppg_y_prev1 > th) &&
                   (g_ppg_y_prev1 > g_ppg_y_prev2) &&
                   (g_ppg_y_prev1 > y);

    int peak_accepted = 0;
    int32_t ibi_report_ms = 0;
    float hr_inst = -1.0f;

    if (is_peak) {
        /* The peak is the previous sample, so use the previous timestamp.
         * This is more accurate for IBI/PRV than using the current sample time.
         */
        int64_t peak_t = (g_ppg_t_prev1 > 0) ? g_ppg_t_prev1 : t_ms;
        int64_t since = peak_t - g_last_peak_t;

        if (since >= PEAK_REFRACTORY_MS) {
            if (g_last_peak_t > 0) {
                int64_t ibi = peak_t - g_last_peak_t;
                if (ibi >= IBI_MIN_MS && ibi <= IBI_MAX_MS) {
                    bool ok = true;

                    if (g_last5_n >= 3) {
                        uint16_t tmp[5];
                        int n = MIN(g_last5_n, 5);
                        for (int i = 0; i < n; i++) tmp[i] = g_last5_ibi[i];

                        for (int i = 0; i < n - 1; i++) {
                            for (int j = i + 1; j < n; j++) {
                                if (tmp[j] < tmp[i]) {
                                    uint16_t t = tmp[i];
                                    tmp[i] = tmp[j];
                                    tmp[j] = t;
                                }
                            }
                        }

                        uint16_t med = (n & 1)
                                     ? tmp[n / 2]
                                     : (uint16_t)((tmp[n / 2 - 1] + tmp[n / 2]) / 2);
                        float frac = f_abs((float)ibi - (float)med) / (float)MAX(med, 1);
                        if (frac > IBI_OUTLIER_FRAC) ok = false;
                    }

                    if (ok) {
                        if (g_ibi_n < IBI_MAX_PER_MIN) {
                            g_ibi_ms[g_ibi_n] = (uint16_t)ibi;
                            g_ibi_t_ms[g_ibi_n] = peak_t;
                            g_ibi_n++;
                        }

                        if (g_last5_n < 5) {
                            g_last5_ibi[g_last5_n++] = (uint16_t)ibi;
                        } else {
                            memmove(&g_last5_ibi[0], &g_last5_ibi[1], 4 * sizeof(uint16_t));
                            g_last5_ibi[4] = (uint16_t)ibi;
                        }

                        peak_accepted = 1;
                        ibi_report_ms = (int32_t)ibi;
                        hr_inst = 60000.0f / (float)ibi;
                    }
                }
            }

            g_last_peak_t = peak_t;
        }
    }

#if PPG_VALIDATION_MODE || PPG_FULLFW_PPG_STREAM_ENABLE
    int32_t ibi_report = ibi_report_ms;
    float hr_report = hr_inst;

    /* Do not present HR/IBI as valid unless the last completed 5-second
     * PPG quality window passed SQI/artifact checks. The waveform itself is
     * still streamed so GUI/mobile can show the final PPG shape.
     */
    if (!g_last_ppg_quality_ok) {
        ibi_report = 0;
        hr_report = -1.0f;
    }

    ppg_stream_send_sample(t_ms,
                           g_ppg_val_seq++,
                           ppg_raw,
                           green,
                           y,
                           th,
                           peak_accepted,
                           ibi_report,
                           hr_report,
                           g_ppg_fs_hz);
#endif

    g_ppg_y_prev2 = g_ppg_y_prev1;
    g_ppg_y_prev1 = y;
    g_ppg_t_prev2 = g_ppg_t_prev1;
    g_ppg_t_prev1 = t_ms;
}

void algo_v0_push_eda_mv(float eda_mv, int64_t t_ms)
{
    if (g_last_eda_t > 0) {
        int64_t dt = t_ms - g_last_eda_t;
        if (dt > 0) {
            float fs = 1000.0f / (float)dt;
            g_eda_fs_hz = 0.9f * g_eda_fs_hz + 0.1f * fs;
        }
    }
    g_last_eda_t = t_ms;

    float uS = eda_mv * g_eda_mv_to_uS;

    if (g_eda_n < (int)ARRAY_SIZE(g_eda_uS)) {
        g_eda_uS[g_eda_n++] = uS;
    }

    if (!isnan(g_last_eda_uS)) {
        float d = f_abs(uS - g_last_eda_uS);
        if (d < EDA_FLAT_DELTA_US) {
            g_eda_flat_cnt++;
        } else {
            g_eda_flat_cnt = 0;
        }

        int need = (int)(EDA_FLAT_TIME_SEC * MAX(g_eda_fs_hz, 1.0f));
        if (g_eda_flat_cnt >= need) {
            g_eda_flatline = true;
        }
    }
    g_last_eda_uS = uS;
}

/* =========================
 * Fast loop (5 seconds)
 * ========================= */
static void reset_5s_accumulators(void)
{
    wf_reset(&g_mag_wf_5s);
    g_sma_sum_5s = 0.0f;
    g_acc_count_5s = 0;

    g_ppg_saturation_5s = false;
    g_ppg_clip_cnt_5s = 0;
    g_ppg_total_cnt_5s = 0;
    g_ppg_energy_band_5s = 0.0f;
    g_ppg_energy_total_5s = 0.0f;
}

static void run_fast_5s(int64_t now_ms)
{
    bool acc_present = (g_acc_count_5s >= ACC_MIN_SAMPLES_5S);
    bool ppg_present = (g_ppg_total_cnt_5s >= PPG_MIN_SAMPLES_5S);

    float accel_var = NAN;
    float sma = NAN;
    if (acc_present) {
        accel_var = wf_var(&g_mag_wf_5s);
        sma = g_sma_sum_5s / (float)MAX(g_acc_count_5s, 1);
    }

    float clip_frac = NAN;
    float ppg_sqi = NAN;
    if (ppg_present) {
        clip_frac = (float)g_ppg_clip_cnt_5s / (float)MAX(g_ppg_total_cnt_5s, 1);
        if (g_ppg_energy_total_5s > 0.0f) {
            ppg_sqi = clamp01(g_ppg_energy_band_5s / g_ppg_energy_total_5s);
        } else {
            ppg_sqi = 0.0f;
        }
    }

    bool artifact = false;

    if (!acc_present || !ppg_present) {
        artifact = true;
    } else {
        artifact =
            (accel_var > TH_ACCEL_VAR_G2) ||
            (sma > TH_SMA_G) ||
            (ppg_sqi < TH_PPG_SQI) ||
            g_ppg_saturation_5s ||
            (clip_frac > TH_CLIP_FRAC);
    }

    if (now_ms < g_ppg_ignore_until_ms) {
        artifact = true;
    }

    g_last_ppg_sqi_live = ppg_present ? ppg_sqi : NAN;
    g_last_ppg_artifact_live = artifact;
    g_last_ppg_quality_ok =
        ppg_present &&
        (now_ms >= g_ppg_ignore_until_ms) &&
        !artifact &&
        !isnan(ppg_sqi) &&
        (ppg_sqi >= TH_PPG_SQI);

    if (g_5s_idx < FAST_WINDOWS_PER_MIN) {
        g_artifact_5s[g_5s_idx] = artifact ? 1u : 0u;
        g_acc_valid_5s[g_5s_idx] = acc_present ? 1u : 0u;
        g_ppg_valid_5s[g_5s_idx] = ppg_present ? 1u : 0u;
        g_sma_5s[g_5s_idx] = acc_present ? sma : NAN;
        g_ppg_sqi_5s[g_5s_idx] = ppg_present ? ppg_sqi : NAN;
        g_ppg_sat_5s[g_5s_idx] = (ppg_present && g_ppg_saturation_5s) ? 1u : 0u;
        g_clip_frac_5s[g_5s_idx] = ppg_present ? clip_frac : NAN;
        g_5s_idx++;
    }

#if PPG_VALIDATION_MODE
    if (ble_log_is_ready()) {
        char line[192];
        int n = snprintk(line, sizeof(line),
            "PV_WIN,t=%lld,ppg_n=%u,sqi=%.3f,clip=%.3f,sat=%d,art=%d,qok=%d,settle=%d,sma=%.3f,acc_var=%.4f\r\n",
            now_ms,
            g_ppg_total_cnt_5s,
            isnan(ppg_sqi) ? -1.0 : (double)ppg_sqi,
            isnan(clip_frac) ? -1.0 : (double)clip_frac,
            (int)g_ppg_saturation_5s,
            (int)artifact,
            g_last_ppg_quality_ok ? 1 : 0,
            (now_ms < g_ppg_ignore_until_ms) ? 1 : 0,
            isnan(sma) ? -1.0 : (double)sma,
            isnan(accel_var) ? -1.0 : (double)accel_var);
        if (n > 0) {
            (void)ble_log_send_as((const uint8_t *)line, (size_t)n);
        }
    }
#else
    LOG_INF("[V0][5s] t=%lld acc_n=%d ppg_n=%u accel_var=%.4f sma=%.3f ppg_sqi=%.3f sat=%d clip=%.3f ART=%d",
            now_ms,
            g_acc_count_5s,
            g_ppg_total_cnt_5s,
            isnan(accel_var) ? -1.0f : accel_var,
            isnan(sma) ? -1.0f : sma,
            isnan(ppg_sqi) ? -1.0f : ppg_sqi,
            (int)g_ppg_saturation_5s,
            isnan(clip_frac) ? -1.0f : clip_frac,
            (int)artifact);
#endif

    reset_5s_accumulators();
}

/* =========================
 * Slow loop (60 seconds)
 * ========================= */
static float median_valid_sma(void)
{
    float tmp[FAST_WINDOWS_PER_MIN];
    int n = 0;

    for (int i = 0; i < FAST_WINDOWS_PER_MIN; i++) {
        if (g_acc_valid_5s[i] && !isnan(g_sma_5s[i])) {
            tmp[n++] = g_sma_5s[i];
        }
    }

    if (n == 0) return NAN;
    return median_float(tmp, n);
}

static activity_t classify_activity(float sma_med)
{
    if (sma_med < SMA_REST_MAX_G) return ACT_REST;
    if (sma_med < SMA_LOW_MAX_G)  return ACT_LOW;
    if (sma_med < SMA_WALK_MAX_G) return ACT_WALK;
    return ACT_VIG;
}

static float activity_confidence(activity_t act)
{
    int in_range = 0;

    for (int i = 0; i < FAST_WINDOWS_PER_MIN; i++) {
        if (!g_acc_valid_5s[i] || isnan(g_sma_5s[i])) {
            continue;
        }

        float s = g_sma_5s[i];
        bool ok = false;

        switch (act) {
        case ACT_REST: ok = (s < SMA_REST_MAX_G); break;
        case ACT_LOW:  ok = (s >= SMA_REST_MAX_G && s < SMA_LOW_MAX_G); break;
        case ACT_WALK: ok = (s >= SMA_LOW_MAX_G && s < SMA_WALK_MAX_G); break;
        default:       ok = (s >= SMA_WALK_MAX_G); break;
        }

        if (ok) in_range++;
    }

    return (float)in_range / (float)FAST_WINDOWS_PER_MIN;
}

static int clean_seconds_in_minute(void)
{
    int clean = 0;
    for (int i = 0; i < FAST_WINDOWS_PER_MIN; i++) {
        if (g_artifact_5s[i] == 0) clean += FAST_LOOP_SEC;
    }
    return clean;
}

static int longest_clean_run_seconds(void)
{
    int best = 0;
    int cur = 0;

    for (int i = 0; i < FAST_WINDOWS_PER_MIN; i++) {
        if (g_artifact_5s[i] == 0) {
            cur += FAST_LOOP_SEC;
            best = MAX(best, cur);
        } else {
            cur = 0;
        }
    }
    return best;
}

static const char *hr_quality_str(int clean_sec)
{
    if (clean_sec >= 30) return "GOOD";
    if (clean_sec >= 20) return "LOW";
    return "INVALID";
}

static void compute_hr_10s_blocks(void)
{
    memset(g_hr10, 0, sizeof(g_hr10));
    memset(g_hr10_valid, 0, sizeof(g_hr10_valid));

    for (int b = 0; b < HR_BLOCKS_PER_MIN; b++) {
        int a0 = b * 2;
        int a1 = a0 + 1;
        bool invalid = false;

        if (a1 >= FAST_WINDOWS_PER_MIN) {
            g_hr10_valid[b] = false;
            continue;
        }

        if (!g_ppg_valid_5s[a0] || !g_ppg_valid_5s[a1]) invalid = true;
        if (g_artifact_5s[a0] || g_artifact_5s[a1]) invalid = true;
        if (g_ppg_sat_5s[a0] || g_ppg_sat_5s[a1]) invalid = true;
        if (isnan(g_ppg_sqi_5s[a0]) || isnan(g_ppg_sqi_5s[a1])) invalid = true;
        if (!isnan(g_ppg_sqi_5s[a0]) && g_ppg_sqi_5s[a0] < TH_PPG_SQI) invalid = true;
        if (!isnan(g_ppg_sqi_5s[a1]) && g_ppg_sqi_5s[a1] < TH_PPG_SQI) invalid = true;

        if (invalid) {
            g_hr10_valid[b] = false;
            continue;
        }

        int64_t block_start = g_minute_start_t + ((int64_t)b * HR_BLOCK_SEC * 1000);
        int64_t block_end   = block_start + ((int64_t)HR_BLOCK_SEC * 1000);

        float block_ibis[IBI_MAX_PER_10S];
        int block_n = 0;

        for (int i = 0; i < g_ibi_n; i++) {
            if (g_ibi_t_ms[i] >= block_start && g_ibi_t_ms[i] < block_end) {
                if (block_n < IBI_MAX_PER_10S) {
                    block_ibis[block_n++] = (float)g_ibi_ms[i];
                }
            }
        }

        if (block_n <= 0) {
            g_hr10_valid[b] = false;
            continue;
        }

        float med_ibi = median_float(block_ibis, block_n);
        if (isnan(med_ibi) || med_ibi <= 0.0f) {
            g_hr10_valid[b] = false;
            continue;
        }

        g_hr10[b] = 60000.0f / med_ibi;
        g_hr10_valid[b] = true;
    }
}

static float median_valid_hr10(int *valid_blocks)
{
    float v[HR_BLOCKS_PER_MIN];
    int n = 0;

    for (int i = 0; i < HR_BLOCKS_PER_MIN; i++) {
        if (g_hr10_valid[i]) v[n++] = g_hr10[i];
    }

    if (valid_blocks) *valid_blocks = n;
    if (n == 0) return NAN;
    return median_float(v, n);
}

static float compute_rmssd_ms(void)
{
    if (g_ibi_n < 2) return NAN;

    float sum = 0.0f;
    int cnt = 0;
    for (int i = 1; i < g_ibi_n; i++) {
        float d = (float)g_ibi_ms[i] - (float)g_ibi_ms[i - 1];
        sum += d * d;
        cnt++;
    }

    if (cnt <= 0) return NAN;
    return sqrtf(sum / (float)cnt);
}

static void compute_eda_minute(float *muSCL,
                               float *sigmaSCR,
                               const char **eda_quality,
                               const char **eda_conf)
{
    *muSCL = NAN;
    *sigmaSCR = NAN;
    *eda_quality = "INVALID";
    *eda_conf = "LOW";

    if (g_eda_n < 10) return;

    int bad = 0;
    int clean_n = 0;

    for (int i = 0; i < g_eda_n; i++) {
        float x = g_eda_uS[i];
        bool bad_sample = false;

        if (x <= EDA_MIN_US || x >= EDA_MAX_US) {
            bad_sample = true;
        }
        if (i > 0) {
            float d = f_abs(x - g_eda_uS[i - 1]);
            if (d > EDA_MAX_DELTA_US) bad_sample = true;
        }

        if (bad_sample) {
            bad++;
        } else {
            g_eda_clean[clean_n++] = x;
        }
    }

    float bad_frac = (float)bad / (float)g_eda_n;
    if (bad_frac > 0.20f || g_eda_flatline || clean_n < 10) {
        *eda_quality = "INVALID";
        *eda_conf = "LOW";
        return;
    }

    lowpass_sequence(g_eda_clean, g_eda_1hz, clean_n, 1.0f, MAX(g_eda_fs_hz, 1.0f));
    lowpass_sequence(g_eda_1hz, g_eda_tonic, clean_n, 0.03f, MAX(g_eda_fs_hz, 1.0f));

    for (int i = 0; i < clean_n; i++) {
        g_eda_phasic[i] = g_eda_1hz[i] - g_eda_tonic[i];
    }

    *muSCL = mean_float(g_eda_tonic, clean_n);
    *sigmaSCR = std_float(g_eda_phasic, clean_n);
    *eda_quality = (isnan(*muSCL) || isnan(*sigmaSCR)) ? "INVALID" : "OK";

    int clean_sec = clean_seconds_in_minute();
    float art_frac = (float)(SLOW_LOOP_SEC - clean_sec) / (float)SLOW_LOOP_SEC;
    *eda_conf = (art_frac > 0.50f) ? "LOW" : "HIGH";
}

static void temp_hist_push(float temp60, int64_t t_ms)
{
    if (g_temp60_hist_n < TEMP_SLOPE_HIST_MIN) {
        g_temp60_hist[g_temp60_hist_n] = temp60;
        g_temp60_hist_t[g_temp60_hist_n] = t_ms;
        g_temp60_hist_n++;
        return;
    }

    memmove(&g_temp60_hist[0], &g_temp60_hist[1], (TEMP_SLOPE_HIST_MIN - 1) * sizeof(g_temp60_hist[0]));
    memmove(&g_temp60_hist_t[0], &g_temp60_hist_t[1], (TEMP_SLOPE_HIST_MIN - 1) * sizeof(g_temp60_hist_t[0]));
    g_temp60_hist[TEMP_SLOPE_HIST_MIN - 1] = temp60;
    g_temp60_hist_t[TEMP_SLOPE_HIST_MIN - 1] = t_ms;
}

static float compute_temp_slope_5min_c_per_min(void)
{
    if (g_temp60_hist_n < 2) return NAN;

    float x_mean = 0.0f;
    float y_mean = 0.0f;

    int64_t t0 = g_temp60_hist_t[0];
    for (int i = 0; i < g_temp60_hist_n; i++) {
        float x_min = (float)(g_temp60_hist_t[i] - t0) / 60000.0f;
        x_mean += x_min;
        y_mean += g_temp60_hist[i];
    }
    x_mean /= (float)g_temp60_hist_n;
    y_mean /= (float)g_temp60_hist_n;

    float num = 0.0f;
    float den = 0.0f;
    for (int i = 0; i < g_temp60_hist_n; i++) {
        float x_min = (float)(g_temp60_hist_t[i] - t0) / 60000.0f;
        float dx = x_min - x_mean;
        float dy = g_temp60_hist[i] - y_mean;
        num += dx * dy;
        den += dx * dx;
    }

    if (den <= 0.0f) return NAN;
    return num / den;
}

static void compute_temp_minute(float *temp60,
                                float *temp_slope_5min,
                                const char **temp_quality,
                                int64_t now_ms)
{
    *temp60 = NAN;
    *temp_slope_5min = NAN;
    *temp_quality = "INVALID";

    if (g_temp_n < 2) return;

    bool ok_range = true;
    for (int i = 0; i < g_temp_n; i++) {
        if (g_temp_c[i] < TEMP_MIN_C || g_temp_c[i] > TEMP_MAX_C) {
            ok_range = false;
            break;
        }
    }

    int n = MIN(g_temp_n, (int)ARRAY_SIZE(g_temp_scratch));
    for (int i = 0; i < n; i++) g_temp_scratch[i] = g_temp_c[i];
    float med = median_float(g_temp_scratch, n);

    bool ok_slope = true;
    if (!isnan(g_last_temp_c)) {
        float d = f_abs(med - g_last_temp_c);
        if (d > TEMP_MAX_DPERMIN_C) ok_slope = false;
    }

    if (!ok_range || !ok_slope) {
        *temp_quality = "INVALID";
        return;
    }

    *temp60 = med;
    *temp_quality = "OK";
    g_last_temp_c = med;

    temp_hist_push(med, now_ms);
    *temp_slope_5min = compute_temp_slope_5min_c_per_min();
}

static void reset_minute_buffers(void)
{
    memset(g_artifact_5s, 0, sizeof(g_artifact_5s));
    memset(g_acc_valid_5s, 0, sizeof(g_acc_valid_5s));
    memset(g_ppg_valid_5s, 0, sizeof(g_ppg_valid_5s));
    memset(g_sma_5s, 0, sizeof(g_sma_5s));
    memset(g_ppg_sqi_5s, 0, sizeof(g_ppg_sqi_5s));
    memset(g_ppg_sat_5s, 0, sizeof(g_ppg_sat_5s));
    memset(g_clip_frac_5s, 0, sizeof(g_clip_frac_5s));
    g_5s_idx = 0;

    g_ibi_n = 0;

    g_eda_n = 0;
    g_eda_flatline = false;
    g_eda_flat_cnt = 0;

    g_temp_n = 0;
}

static void run_slow_60s(int64_t now_ms)
{
    int clean_sec = clean_seconds_in_minute();
    float art_frac = (float)(SLOW_LOOP_SEC - clean_sec) / (float)SLOW_LOOP_SEC;

    float sma_med = median_valid_sma();

    activity_t act = isnan(sma_med) ? ACT_REST : classify_activity(sma_med);
    float act_conf = isnan(sma_med) ? 0.0f : activity_confidence(act);

    compute_hr_10s_blocks();
    int valid_hr_blocks = 0;
    float hr60 = median_valid_hr10(&valid_hr_blocks);
    int hr_cov = valid_hr_blocks * HR_BLOCK_SEC;
    const char *hr_quality = hr_quality_str(hr_cov);
    if (valid_hr_blocks == 0 || isnan(hr60)) {
        hr_quality = "INVALID";
        hr_cov = 0;
        hr60 = NAN;
    }

    float rmssd = NAN;
    const char *hrv_quality = "INVALID";
    int longest_clean = longest_clean_run_seconds();
    if (strcmp(hr_quality, "GOOD") == 0 && longest_clean >= HRV_MIN_CONT_CLEAN_SEC) {
        rmssd = compute_rmssd_ms();
        hrv_quality = isnan(rmssd) ? "INVALID" : "OK";
    }

    float muSCL, sigmaSCR;
    const char *eda_q, *eda_c;
    compute_eda_minute(&muSCL, &sigmaSCR, &eda_q, &eda_c);

    float temp60, temp_slope_5min;
    const char *temp_q;
    compute_temp_minute(&temp60, &temp_slope_5min, &temp_q, now_ms);

    bool restlow = (act == ACT_REST || act == ACT_LOW);
    bool wake_motion = (act == ACT_WALK || act == ACT_VIG);

    if (restlow && strcmp(hr_quality, "INVALID") != 0) {
        g_sleep_restlow_streak_min++;
    } else {
        g_sleep_restlow_streak_min = 0;
    }

    if (wake_motion) {
        g_sleep_wake_streak_min++;
    } else {
        g_sleep_wake_streak_min = 0;
    }

    if (g_sleep_restlow_streak_min >= 10) {
        g_sleep_likely = true;
    }
    if (g_sleep_wake_streak_min >= 2) {
        g_sleep_likely = false;
        g_sleep_restlow_streak_min = 0;
    }
    if (!restlow && !wake_motion) {
        g_sleep_likely = false;
    }

    const char *sleep_state = g_sleep_likely ? "SLEEP" : "WAKE";
    float sleep_conf = g_sleep_likely
                     ? clamp01((float)g_sleep_restlow_streak_min / 10.0f)
                     : clamp01((float)g_sleep_restlow_streak_min / 10.0f);

    char line[320];
    int n = snprintk(line, sizeof(line),
        "V0_MIN t=%lld "
        "ACT=%s act_conf=%.2f "
        "ART_frac=%.2f "
        "HR=%.1f HR_cov=%ds HR_q=%s "
        "HRV_RMSSD=%.1f HRV_q=%s "
        "EDA_muSCL=%.3f EDA_sigmaSCR=%.3f EDA_q=%s EDA_conf=%s "
        "TEMP=%.2f TEMP_q=%s TEMP_slope5m=%.4f "
        "SLEEP=%s sleep_conf=%.2f",
        now_ms,
        act_str(act), act_conf,
        art_frac,
        isnan(hr60) ? -1.0f : hr60, hr_cov, hr_quality,
        isnan(rmssd) ? -1.0f : rmssd, hrv_quality,
        isnan(muSCL) ? -1.0f : muSCL,
        isnan(sigmaSCR) ? -1.0f : sigmaSCR,
        eda_q, eda_c,
        isnan(temp60) ? -1.0f : temp60,
        temp_q,
        isnan(temp_slope_5min) ? -1.0f : temp_slope_5min,
        sleep_state,
        sleep_conf);

    LOG_INF("%s", line);
    if (g_minute_cb && n > 0) g_minute_cb(line);

    reset_minute_buffers();
    g_minute_start_t += ((int64_t)SLOW_LOOP_SEC * 1000);
}

/* =========================
 * Algo thread: schedules 5s + 60s
 * ========================= */
#define ALGO_STACK_SIZE 4096
#define ALGO_PRIORITY   4

K_THREAD_STACK_DEFINE(algo_stack, ALGO_STACK_SIZE);
static struct k_thread algo_tcb;
static bool started;

static void algo_thread(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);

    int64_t now = k_uptime_get();

    g_minute_start_t = -1;
    g_next_fast_t = -1;
    g_next_slow_t = -1;
    g_algo_armed = false;

    reset_5s_accumulators();

    LOG_INF("Algo V0 started: waiting for IMU + PPG before arming fast=%ds slow=%ds",
            FAST_LOOP_SEC, SLOW_LOOP_SEC);

    while (1) {
        now = k_uptime_get();

        if (!g_algo_armed) {
            if (g_seen_imu && g_seen_ppg) {
                g_minute_start_t = now;
                g_next_fast_t = now + ((int64_t)FAST_LOOP_SEC * 1000);
                g_next_slow_t = now + ((int64_t)SLOW_LOOP_SEC * 1000);
                reset_minute_buffers();
                reset_5s_accumulators();
                g_algo_armed = true;
                LOG_INF("Algo V0 armed at t=%lld after receiving IMU + PPG", now);
            } else {
                k_sleep(K_MSEC(20));
                continue;
            }
        }

        while (now >= g_next_fast_t) {
            run_fast_5s(g_next_fast_t);
            g_next_fast_t += ((int64_t)FAST_LOOP_SEC * 1000);
        }

        while (now >= g_next_slow_t) {
            run_slow_60s(g_next_slow_t);
            g_next_slow_t += ((int64_t)SLOW_LOOP_SEC * 1000);
        }

        k_sleep(K_MSEC(20));
    }
}

void algo_v0_init(void)
{
    LOG_INF("Algo V0 initialization started");

    memset(g_artifact_5s, 0, sizeof(g_artifact_5s));
    memset(g_acc_valid_5s, 0, sizeof(g_acc_valid_5s));
    memset(g_ppg_valid_5s, 0, sizeof(g_ppg_valid_5s));
    memset(g_sma_5s, 0, sizeof(g_sma_5s));
    memset(g_ppg_sqi_5s, 0, sizeof(g_ppg_sqi_5s));
    memset(g_ppg_sat_5s, 0, sizeof(g_ppg_sat_5s));
    memset(g_clip_frac_5s, 0, sizeof(g_clip_frac_5s));
    g_5s_idx = 0;

    wf_reset(&g_mag_wf_5s);
    g_sma_sum_5s = 0.0f;
    g_acc_count_5s = 0;

    g_ppg_saturation_5s = false;
    g_ppg_clip_cnt_5s = 0;
    g_ppg_total_cnt_5s = 0;
    g_ppg_energy_band_5s = 0.0f;
    g_ppg_energy_total_5s = 0.0f;

    g_last_ppg_t = -1;
    g_ppg_hp = 0.0f;
    g_ppg_lp = 0.0f;
    g_ppg_mean_ewma = 0.0f;
    g_ppg_var_ewma = 1.0f;
    g_ppg_x_prev = 0.0f;
    g_ppg_y_prev1 = 0.0f;
    g_ppg_y_prev2 = 0.0f;
    g_ppg_t_prev1 = -1;
    g_ppg_t_prev2 = -1;
    g_last_peak_t = -1000000;
#if PPG_VALIDATION_MODE || PPG_FULLFW_PPG_STREAM_ENABLE
    g_ppg_val_seq = 0;
#endif
    g_last_ppg_quality_ok = false;
    g_last_ppg_sqi_live = NAN;
    g_last_ppg_artifact_live = true;

    g_ibi_n = 0;
    memset(g_ibi_ms, 0, sizeof(g_ibi_ms));
    memset(g_ibi_t_ms, 0, sizeof(g_ibi_t_ms));
    memset(g_last5_ibi, 0, sizeof(g_last5_ibi));
    g_last5_n = 0;

    g_eda_n = 0;
    g_eda_flatline = false;
    g_eda_flat_cnt = 0;
    g_last_eda_t = -1;
    g_last_eda_uS = NAN;

    g_temp_n = 0;
    g_last_temp_c = NAN;
    g_last_temp_t = -1;
    memset(g_temp60_hist, 0, sizeof(g_temp60_hist));
    memset(g_temp60_hist_t, 0, sizeof(g_temp60_hist_t));
    g_temp60_hist_n = 0;

    g_sleep_restlow_streak_min = 0;
    g_sleep_wake_streak_min = 0;
    g_sleep_likely = false;

    g_seen_imu = false;
    g_seen_ppg = false;
    g_algo_armed = false;

    g_minute_start_t = -1;
    g_next_fast_t = -1;
    g_next_slow_t = -1;

    g_acc_fs_hz = ACC_FS_HZ_DEFAULT;
    g_ppg_fs_hz = PPG_FS_HZ_DEFAULT;
    g_eda_fs_hz = EDA_FS_HZ_DEFAULT;
    g_temp_fs_hz = TEMP_FS_HZ_DEFAULT;

    g_last_acc_t = -1;

    g_gx = 0.0f;
    g_gy = 0.0f;
    g_gz = 1.0f;
}

void algo_v0_start(void)
{
    if (started) return;
    started = true;

    k_thread_create(&algo_tcb, algo_stack, K_THREAD_STACK_SIZEOF(algo_stack),
                    algo_thread, NULL, NULL, NULL,
                    ALGO_PRIORITY, 0, K_NO_WAIT);

    k_thread_name_set(&algo_tcb, "algo_v0");
}
