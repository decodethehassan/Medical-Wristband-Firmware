#pragma once

/*
 * PPG validation mode switch.
 *
 * 1 = build a focused validation firmware path:
 *     - MAX30101 forced to green-only PPG
 *     - PPG + IMU + algo stay active
 *     - NAND/EDA/TEMP startup/logging is skipped or quiet
 *     - algo_v0 streams clean PV/PV_WIN lines over BLE
 *
 * 0 = restore normal full firmware behavior.
 */
#define PPG_VALIDATION_MODE        0

/* Stream every N PPG samples. At 64 Hz:
 * 1 = ~64 PV lines/sec, 2 = ~32 PV lines/sec, 4 = ~16 PV lines/sec.
 * Accepted peak samples are always streamed.
 */
#define PPG_VAL_STREAM_DIV         8U

/*
 * Lightweight final PPG waveform stream for COMPLETE FW mode.
 *
 * This keeps PPG_VALIDATION_MODE = 0, so NAND/EDA/TEMP/full FW still run,
 * but it also exports the final filtered PPG waveform used by Algo V0.
 *
 * BLE/mobile/GUI can plot:
 *   filt = final filtered PPG waveform
 *   th   = adaptive peak threshold/debug line
 *   peak = accepted beat peak flag
 *   ibi  = accepted pulse interval in ms
 *   sqi/art/qok = quality gate information
 *
 * At 64 Hz, div=8 gives about 8 waveform points/sec plus accepted peak samples.
 * Increase to 16 if BLE bandwidth is tight; decrease to 4 for smoother debug plots.
 */
#define PPG_FULLFW_PPG_STREAM_ENABLE 1
#define PPG_FULLFW_PPG_STREAM_DIV    8U

#define PPG_VAL_GREEN_ONLY         1
#define PPG_VAL_DISABLE_NAND       1
#define PPG_VAL_DISABLE_EDA        1
#define PPG_VAL_DISABLE_TEMP       1
#define PPG_VAL_DISABLE_DEBUG_LOGS 1
