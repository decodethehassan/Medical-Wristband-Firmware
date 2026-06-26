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
#define PPG_VALIDATION_MODE        1

/* Stream every N native 100 Hz PPG samples.
 * 1 = ~100 PV lines/sec, 2 = ~50 PV lines/sec, 4 = ~25 PV lines/sec.
 * Accepted peak samples are always streamed.
 */
#define PPG_VAL_STREAM_DIV         2U

/* Compact GUI morphology stream.
 * Best current setting for BLE text streaming:
 *   PC div=4  -> compact clean PPG at ~25 Hz. This is the most stable
 *                setting seen so far for the current BLE text path and GUI.
 *   PV div=64 -> full diagnostics at ~1.6 Hz to keep BLE free for waveform.
 *
 * Internal algorithm still processes every native 100 Hz MAX30101 sample.
 */
#define PPG_PC_STREAM_DIV          4U
#define PPG_PV_DEBUG_STREAM_DIV    64U
#define PPG_VAL_LED_CURRENT       0x80U
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
