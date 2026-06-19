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

/* Stream every N native 100 Hz PPG samples.
 * 1 = ~100 PV lines/sec, 2 = ~50 PV lines/sec, 4 = ~25 PV lines/sec.
 * Accepted peak samples are always streamed.
 */
#define PPG_VAL_STREAM_DIV         2U

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
 * At native 100 Hz, div=2 gives about 50 waveform points/sec plus accepted peak samples.
 * Use div=1 for the smoothest plot; use div=4 if BLE bandwidth is tight.
 */
#define PPG_FULLFW_PPG_STREAM_ENABLE 1
#define PPG_FULLFW_PPG_STREAM_DIV    2U

#define PPG_VAL_GREEN_ONLY         1
#define PPG_VAL_DISABLE_NAND       1
#define PPG_VAL_DISABLE_EDA        1
#define PPG_VAL_DISABLE_TEMP       1
#define PPG_VAL_DISABLE_DEBUG_LOGS 1
