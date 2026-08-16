/*
 * dac_audio.h - DAC audio / CTCSS tone driver for AT32F403A on the RT-950 Pro
 *
 * DAC channel 1 output on PA4, driven by DMA2 channel 3 with TIM6 trigger.
 * V0.27 OEM DAC/DMA evidence:
 *   DAC init        @ fw 0x0800A548  (CR bit-set/clear @ fw 0x0800A508)
 *   DMA2_CH3 ISR    @ fw 0x0800D1D5  (active in vector table, confirmed)
 *   TIM6 clock ref  @ fw 0x0800697C  (enable), fw 0x0801BF70 (audio area)
 *
 * OEM DAC config: BOFF1=1 (output buffer OFF), TIM6 TRGO trigger (no ISR).
 * DMA2_CH3 runs in circular mode for continuous DAC playback.
 * OEM may use DAC channel 2 (EN2 = bit 16 in DAC_CR) - open question.
 *
 * TIM6 sample rates (V0.27):
 *   PSC=3, ARR=781   -> ~1200 Hz sample rate (CTCSS generation)
 *   PSC=119, ARR=125 -> ~248 Hz sample rate (low-rate audio)
 *
 * CTCSS tones: 50 standard tones, stored as frequency x 10 values.
 * Generated via 32-sample sine LUT at TIM6-controlled sample rate.
 */

#ifndef DRIVERS_DAC_AUDIO_H
#define DRIVERS_DAC_AUDIO_H

#include "at32f403a.h"

/* DAC Control Register (CR) bits ------------------------------------- */
#define DAC_CR_EN1          (1UL << 0)      /* Channel 1 enable */
#define DAC_CR_BOFF1        (1UL << 1)      /* Channel 1 output buffer disable */
#define DAC_CR_TEN1         (1UL << 2)      /* Channel 1 trigger enable */
#define DAC_CR_TSEL1_MASK   (7UL << 3)      /* Channel 1 trigger selection */
#define DAC_CR_TSEL1_TIM6   (0UL << 3)      /* TIM6 TRGO */
#define DAC_CR_TSEL1_TIM3   (1UL << 3)      /* TIM3 TRGO */
#define DAC_CR_TSEL1_TIM7   (2UL << 3)      /* TIM7 TRGO */
#define DAC_CR_TSEL1_TIM5   (3UL << 3)      /* TIM5 TRGO */
#define DAC_CR_TSEL1_TIM2   (4UL << 3)      /* TIM2 TRGO */
#define DAC_CR_TSEL1_TIM4   (5UL << 3)      /* TIM4 TRGO */
#define DAC_CR_TSEL1_EXT9   (6UL << 3)      /* External line 9 */
#define DAC_CR_TSEL1_SW     (7UL << 3)      /* Software trigger */
#define DAC_CR_WAVE1_MASK   (3UL << 6)      /* Wave generation mode */
#define DAC_CR_MAMP1_MASK   (0xFUL << 8)    /* Mask/amplitude selector */
#define DAC_CR_DMAEN1       (1UL << 12)     /* Channel 1 DMA enable */
#define DAC_CR_EN2          (1UL << 16)     /* Channel 2 enable */

/* CTCSS constants ---------------------------------------------------- */
#define CTCSS_TONE_COUNT    50      /* Standard CTCSS tone table size */

/* DMA buffer sizing - OEM uses 2×2048 halfword double-buffers.
 * We use a single 2048-sample buffer (circular DMA, no TCIE needed).
 * At 7936 Hz sample rate, 2048 samples = 258ms of audio - long enough
 * for the DMA to loop cleanly without audible glitches. */
#define DAC_BUFFER_SAMPLES  2048    /* Samples per DMA buffer */

/* OEM sine LUT is 256 entries (12-bit unsigned, centered at 0x0800).
 * DDS phase accumulator indexes into this table via top 8 bits. */
#define SINE_LUT_SIZE       256     /* Entries in sine lookup table */

/* Functions ---------------------------------------------------------- */

/*
 * dac_audio_init - Enable DAC and TIM6 clocks, configure DAC1 for
 *                  TIM6-triggered DMA output on PA4.
 */
void dac_audio_init(void);

/*
 * dac_audio_play_tone - Generate a continuous CTCSS sine tone.
 * @param freq_hz_x10  Tone frequency x 10 (e.g., 670 = 67.0 Hz)
 *
 * Configures TIM6 for the required sample rate and starts circular
 * DMA from the internal 32-sample sine LUT to DAC1.
 */
void dac_audio_play_tone(uint16_t freq_hz_x10);

/*
 * dac_audio_stop - Stop tone/audio playback.
 * Disables TIM6 and DMA2 CH3.
 */
void dac_audio_stop(void);

/* Output volume, 0-100 %. Scales the DAC waveform about its midpoint.
 * Independent of the radio's volume knob, which is an analog pot the MCU
 * cannot read. */
void dac_audio_set_volume(uint8_t percent);
uint8_t dac_audio_get_volume(void);

/*
 * dac_audio_play_buffer - Play an arbitrary waveform via DMA.
 * @param samples   Pointer to 12-bit sample buffer
 * @param count     Number of samples
 * @param circular  Non-zero for continuous looping, 0 for one-shot
 */
void dac_audio_play_buffer(const uint16_t *samples, uint16_t count,
                           int circular);

/*
 * dac_audio_is_playing - Check if DMA playback is active.
 * Returns non-zero if DMA2 CH3 is still enabled.
 */
int dac_audio_is_playing(void);

#endif /* DRIVERS_DAC_AUDIO_H */
