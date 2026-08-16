/*
 * dac_audio.c - DAC audio / CTCSS tone driver for AT32F403A on the RT-950 Pro
 *
 * DAC output fed by DMA2 channel 3, triggered by TIM6 TRGO.
 *
 * OEM V0.27 function addresses:
 *   dac_init               @ 0x0800A4F6 (18B)  - DAC peripheral init
 *   dac_enable              @ 0x0800A508 (26B)  - DAC CR enable bits
 *   dac_dma_enable          @ 0x0800A528 (26B)  - Enable DMA trigger on DAC
 *   dac_dma_timer_config    @ 0x0800A548 (36B)  - Configure trigger/DMA bits in CR
 *   tim6_dac_dma_init       @ 0x08018EC0 (174B) - Complete TIM6+DMA2_CH3+DAC setup
 *   beep_play               @ 0x08003808 (360B) - Single-tone via TIM6+DMA+DAC
 *
 * OEM TIM6 configurations (verified):
 *   Beep:  PSC=3, ARR=781  → sample rate ~38.4 kHz (~1200 Hz with 32 samples)
 *   Init:  PSC=119, ARR=125 → sample rate ~7.9 kHz (~248 Hz with 32 samples)
 *   CR2 MMS = 0x20 (update event triggers TRGO → DMA2_CH3 → DAC_DHR12R1)
 *
 * OEM hardware registers:
 *   DAC base = 0x40007400, DHR12R1 = 0x40007408
 *   TIM6 base = 0x40001000
 *   DMA2_CH3 target = 0x40007408 (DAC data reg)
 *   Waveform buffer = 0x2000E87C (16 halfwords in RAM)
 *
 * DAC config: BOFF1=1 (buffer OFF), TSEL1=0 (TIM6 TRGO trigger), DMA enabled.
 *
 * TIM6 clock = 120 MHz (APB1 bus = 60 MHz, timer clock doubled because
 * APB1 prescaler > 1).
 */

#include "drivers/dac_audio.h"

/* TIM6 timer instance ------------------------------------------------ */
#define TIM6    ((TIM_TypeDef *)TIM6_BASE)

/* TIM6 clock frequency ----------------------------------------------- */
#define TIM6_CLOCK_HZ   120000000UL

/* TIM CR1 / CR2 bits used for basic timer ---------------------------- */
#define TIM_CR1_CEN     (1UL << 0)      /* Counter enable */
#define TIM_CR2_MMS_UPDATE (2UL << 4)   /* MMS = 010: Update event as TRGO */

/* TIM EGR bits ------------------------------------------------------- */
#define TIM_EGR_UG      (1UL << 0)      /* Update generation */

/* DMA Channel Configuration Register (CCR) bits ---------------------- */
#define DMA_CCR_EN      (1UL << 0)      /* Channel enable */
#define DMA_CCR_DIR     (1UL << 4)      /* Direction: 1 = read from memory */
#define DMA_CCR_CIRC    (1UL << 5)      /* Circular mode */
#define DMA_CCR_MINC    (1UL << 7)      /* Memory increment mode */
#define DMA_CCR_PSIZE_16 (1UL << 8)     /* Peripheral size: 16-bit */
#define DMA_CCR_MSIZE_16 (1UL << 10)    /* Memory size: 16-bit */
#define DMA_CCR_PL_HIGH (2UL << 12)     /* Priority: high */

/* DMA2 ISR/IFCR bits for channel 3 (flags at bits 8-11) */
#define DMA2_IFCR_CGIF3 (1UL << 8)     /* Clear channel 3 global flag */

/*
 * 256-entry sine lookup table, 12-bit values (0-4095), centered at 2048.
 * Values: 2048 + 2047 * sin(2pi * i / 256), rounded to nearest integer.
 *
 * OEM uses identical 256×16-bit sine LUT in flash @ 0x0802FC84.
 * beep_tone_generate() indexes this via DDS phase accumulator:
 *   idx = (phase_acc >> 24) & 0xFF  (top 8 bits of 32-bit accumulator)
 */
/* Output volume, 0-100 %.
 *
 * The radio's volume knob is an ANALOG POT in the audio path -- there is no ADC
 * channel for it (the only analog inputs are PA0/PA1, both battery sense), so
 * the MCU cannot read its position. Software volume is therefore a separate
 * control: it scales the DAC waveform about its 2048 midpoint before the signal
 * ever reaches the pot.
 *
 * Scaling about the midpoint rather than multiplying the raw sample matters --
 * the LUT is 2048 + 2047*sin(), so a naive multiply would shift the DC offset
 * and thump the speaker instead of quietening it. */
static uint8_t tone_volume = 100;

static const uint16_t sine_lut[SINE_LUT_SIZE] = {
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2397,
    2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
    2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143,
    3185, 3227, 3267, 3307, 3347, 3385, 3423, 3460,
    3496, 3531, 3565, 3598, 3630, 3662, 3692, 3722,
    3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3975, 3992, 4007, 4021, 4034, 4046,
    4056, 4065, 4073, 4080, 4086, 4090, 4093, 4095,
    4095, 4095, 4093, 4090, 4086, 4080, 4073, 4065,
    4056, 4046, 4034, 4021, 4007, 3992, 3975, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777,
    3750, 3722, 3692, 3662, 3630, 3598, 3565, 3531,
    3496, 3460, 3423, 3385, 3347, 3307, 3267, 3227,
    3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
    2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496,
    2447, 2397, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1998, 1948, 1898, 1848, 1798, 1748, 1699,
    1649, 1600, 1551, 1502, 1454, 1406, 1359, 1312,
    1265, 1219, 1173, 1128, 1083, 1039,  996,  953,
     911,  869,  829,  789,  749,  711,  673,  636,
     600,  565,  531,  498,  466,  434,  404,  374,
     346,  319,  292,  267,  243,  220,  198,  177,
     157,  138,  121,  104,   89,   75,   62,   50,
      40,   31,   23,   16,   10,    6,    3,    1,
       1,    1,    3,    6,   10,   16,   23,   31,
      40,   50,   62,   75,   89,  104,  121,  138,
     157,  177,  198,  220,  243,  267,  292,  319,
     346,  374,  404,  434,  466,  498,  531,  565,
     600,  636,  673,  711,  749,  789,  829,  869,
     911,  953,  996, 1039, 1083, 1128, 1173, 1219,
    1265, 1312, 1359, 1406, 1454, 1502, 1551, 1600,
    1649, 1699, 1748, 1798, 1848, 1898, 1948, 1998
};

/* 2048-sample DMA buffer in RAM - OEM uses 0x2000E87C (2048 halfwords).
 * Filled by DDS phase accumulator before DMA start. */
static uint16_t dma_buf[DAC_BUFFER_SAMPLES];

/* Internal helper: configure DMA2 CH3 for DAC1 transfer -------------- */

static void dma2_ch3_setup(const uint16_t *src, uint16_t count, int circular)
{
    DMA_Channel_TypeDef *ch = DMA2_CH(3);

    /* Disable channel before reconfiguring */
    ch->CCR = 0;

    /* Clear all channel 3 interrupt flags */
    DMA2->IFCR = DMA2_IFCR_CGIF3;

    /* Number of data items to transfer */
    ch->CNDTR = count;

    /* Peripheral address: DAC channel 1, 12-bit right-aligned data */
    ch->CPAR = (uint32_t)&DAC->DHR12R1;

    /* Memory address: source buffer */
    ch->CMAR = (uint32_t)src;

    /* Configure: mem-to-periph, mem-increment, 16-bit, high priority */
    uint32_t ccr = DMA_CCR_DIR | DMA_CCR_MINC
                 | DMA_CCR_PSIZE_16 | DMA_CCR_MSIZE_16
                 | DMA_CCR_PL_HIGH;

    if (circular) {
        ccr |= DMA_CCR_CIRC;
    }

    /* Enable channel */
    ch->CCR = ccr | DMA_CCR_EN;
}

/* ========================================================================
 *  dac_audio_init - Enable DAC/TIM6/DMA2 clocks, configure DAC1 for
 *                   TIM6-triggered DMA output on PA4.
 *
 *  OEM tim6_dac_dma_init @ 0x08018EC0:
 *    - Enables TIM6+DAC clocks, configures DMA2_CH3 for 2048-sample circular
 *    - Sets TIM6 PSC=119, ARR=125 (7936 Hz playback rate)
 *    - LEAVES DAC CH1 DISABLED (EN1=0, DMAEN1=0) - beep_play enables later
 *    - Enables TIM6 (runs continuously)
 *    - DAC CH2 (PA5) enabled separately for amp bias
 *
 *  PA4 must be set to analog mode (MODE=00, CNF=00) before enabling DAC.
 * ======================================================================== */

void dac_audio_init(void)
{
    /* Enable peripheral clocks (OEM APB1EN includes BKP for DAC operation) */
    CRM->APB1EN |= CRM_APB1EN_DACEN | CRM_APB1EN_TIM6EN | CRM_APB1EN_BKPEN;
    CRM->AHBEN  |= CRM_AHBEN_DMA2EN;

    /* PA4 + PA5 = analog mode (disable digital input circuitry).
     * Required for clean DAC output per reference manual. */
    {
        volatile uint32_t *crl = &((GPIO_TypeDef *)GPIOA_BASE)->CRL;
        uint32_t v = *crl;
        v &= ~(0xFFUL << 16);  /* PA4+PA5: bits [23:16] = MODE=00 CNF=00 */
        *crl = v;               /* analog mode for both DAC channels */
    }

    /* Configure DAC - OEM leaves EN1 DISABLED at boot.
     * Only EN2 (CH2 bias on PA5) + BOFF1 + TEN1 + TSEL1=TIM6 set here.
     * EN1 + DMAEN1 are enabled later in dac_audio_play_tone(). */
    DAC->CR = DAC_CR_EN2 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_TSEL1_TIM6;

    /* TIM6: set to OEM idle rate (7936 Hz).
     * OEM runs TIM6 continuously - DMA triggers are harmless when DAC
     * CH1 is disabled (EN1=0). */
    TIM6->CR1 = 0;
    TIM6->CR2 = TIM_CR2_MMS_UPDATE;    /* Update event -> TRGO */
    TIM6->PSC = 119;                    /* 120MHz / 120 = 1MHz tick */
    TIM6->ARR = 125;                    /* 1MHz / 126 = 7936 Hz */
    TIM6->EGR = TIM_EGR_UG;            /* Load shadow registers */
    TIM6->CR1 = TIM_CR1_CEN;           /* Start TIM6 (runs continuously) */
}

/* ========================================================================
 *  dac_audio_play_tone - Generate continuous tone via DMA (OEM-matching).
 *
 *  OEM beep_play sequence (@ 0x08003808):
 *    Phase 1: TIM6 PSC=3, ARR=781 → 38.3kHz (fast fill rate - not used here)
 *    Phase 2-3: Fill 2×2048 buffers via DDS phase accumulator
 *    Phase 4: SET PC12 (done by caller audio_path_enable)
 *    Phase 5: Configure DMA2_CH3 circular 2048 samples
 *    Phase 6: Enable DAC EN1 + DMAEN1
 *    Phase 7: Double-buffer refill loop (we skip - single circular buf)
 *    Phase 8: TIM6 PSC=119, ARR=125 → 7936Hz playback
 *    Phase 9: CLR PC12 (done by caller audio_path_disable)
 *
 *  We simplify: fill 2048 samples in RAM, configure DMA circular,
 *  enable DAC, set TIM6 to 7936Hz. The 2048-sample buffer contains
 *  enough complete sine cycles for clean circular playback.
 *
 *  DDS frequency generation (OEM beep_tone_generate @ 0x08003780):
 *    freq_step = target_freq * 2^32 / sample_rate
 *    For each sample: phase_acc += freq_step; idx = (phase_acc >> 24) & 0xFF
 *    buf[i] = sine_lut[idx]
 *
 *  At 7936 Hz sample rate with 256-entry LUT:
 *    freq_step = target_freq * 4294967296 / 7936
 * ======================================================================== */

/* DDS sample rate - matches OEM TIM6 playback config */
#define DAC_SAMPLE_RATE  7936UL

void dac_audio_play_tone(uint16_t freq_hz_x10)
{
    if (freq_hz_x10 == 0) {
        dac_audio_stop();
        return;
    }

    /* Stop any current playback */
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA2_CH(3)->CCR = 0;

    /* Clear DMA underrun flag (DAC_SR bit 13 = DMAUDR1) */
    DAC->SR = (1UL << 13);

    /* === Fill 2048-sample buffer via DDS phase accumulator ===
     *
     * OEM beep_tone_generate @ 0x08003780:
     *   phase_acc += freq_step  (32-bit accumulator)
     *   idx = (phase_acc >> 24) & 0xFF  → top 8 bits as 256-entry LUT index
     *
     * With >>24, the accumulator wraps at 2^32, and one full LUT cycle (256
     * entries) corresponds to one full 2^32 accumulator cycle.
     *   output_freq = freq_step * sample_rate / 2^32
     *   freq_step   = output_freq * 2^32 / sample_rate
     *               = (freq_hz_x10 * 2^32) / (10 * 7936)
     *               = (freq_hz_x10 * 4294967296) / 79360
     */
    {
        uint32_t freq_step = (uint32_t)(((uint64_t)freq_hz_x10 * 4294967296ULL) / 79360);

        /* Quantise so the buffer holds a WHOLE number of sine cycles.
         *
         * The DMA buffer is circular and the accumulator restarts at phase 0
         * every time it is refilled. Unless freq_step * DAC_BUFFER_SAMPLES is
         * an exact multiple of 2^32, the phase at the end of the buffer does
         * not line up with the phase at its start -- so every wrap produces a
         * step discontinuity in the waveform, heard as a click.
         *
         * The buffer repeats at 7936 / 2048 = 3.875 Hz, which is exactly the
         * "about four wobbles a second" this produced.
         *
         * Rounding freq_step to a multiple of 2^32 / DAC_BUFFER_SAMPLES = 2^21
         * makes the ends meet and the loop seamless. The cost is frequency
         * resolution of 3.875 Hz -- inaudible for beeps, and far preferable to
         * a click on every wrap. */
        {
            const uint32_t quantum = 4294967296ULL / DAC_BUFFER_SAMPLES;  /* 2^21 */
            uint32_t rounded = ((freq_step + quantum / 2u) / quantum) * quantum;
            if (rounded == 0u)
                rounded = quantum;          /* never round a tone away entirely */
            freq_step = rounded;
        }
        uint32_t phase_acc = 0;
        int i;
        for (i = 0; i < DAC_BUFFER_SAMPLES; i++) {
            phase_acc += freq_step;
            uint8_t idx = (phase_acc >> 24) & 0xFF;
            {
                int32_t v = (int32_t)sine_lut[idx] - 2048;
                v = (v * (int32_t)tone_volume) / 100;
                dma_buf[i] = (uint16_t)(2048 + v);
            }
        }
    }

    /* === Configure DMA2_CH3: RAM buffer → DAC_DHR12R1, circular ===
     * OEM beep_play phase 5: CCR = MEM_TO_PERIPH | MEM_INC | HALFWORD | CIRC | TCIE | EN
     * We omit TCIE (no double-buffer ISR needed with 2048 circular). */
    dma2_ch3_setup(dma_buf, DAC_BUFFER_SAMPLES, 1);

    /* === Enable DAC CH1 + DMA - OEM beep_play phase 6 ===
     * Preserve EN2 for amp bias. */
    DAC->CR = DAC_CR_EN2 | DAC_CR_EN1 | DAC_CR_BOFF1 | DAC_CR_TEN1
            | DAC_CR_TSEL1_TIM6 | DAC_CR_DMAEN1;

    /* === Set TIM6 to OEM playback rate - phase 8 ===
     * PSC=119, ARR=125 → 120MHz/120/126 = 7936 Hz */
    TIM6->PSC = 119;
    TIM6->ARR = 125;
    TIM6->EGR = TIM_EGR_UG;            /* Load shadow registers */
    TIM6->CR1 = TIM_CR1_CEN;           /* Start playback */
}

/* ========================================================================
 *  dac_audio_stop - Stop tone/audio playback.
 *
 *  OEM behavior: DAC CH1 disabled (EN1=0, DMAEN1=0), but TIM6 keeps
 *  running at idle rate. DMA channel disabled.
 * ======================================================================== */

void dac_audio_stop(void)
{
    /* Disable DMA2 channel 3 first */
    DMA2_CH(3)->CCR = 0;

    /* Clear DMA flags */
    DMA2->IFCR = DMA2_IFCR_CGIF3;

    /* Clear DMAUDR1 in case it got set */
    DAC->SR = (1UL << 13);

    /* Disable DAC CH1 + DMA - keep EN2 for amp bias.
     * OEM leaves TIM6 running (harmless when DAC disabled). */
    DAC->CR = DAC_CR_EN2 | DAC_CR_BOFF1 | DAC_CR_TEN1 | DAC_CR_TSEL1_TIM6;

    /* Set DAC output to mid-scale (silence) */
    DAC->DHR12R1 = 2048;
}

/* ========================================================================
 *  dac_audio_play_buffer - Play arbitrary waveform via DMA.
 *
 *  Caller must configure TIM6 timing beforehand (or call after
 *  dac_audio_play_tone which sets up TIM6).  If the caller wants a
 *  specific sample rate, set TIM6->ARR = (TIM6_CLOCK / rate) - 1.
 * ======================================================================== */

void dac_audio_play_buffer(const uint16_t *samples, uint16_t count,
                           int circular)
{
    /* Stop current transfer */
    TIM6->CR1 &= ~TIM_CR1_CEN;
    DMA2_CH(3)->CCR = 0;

    /* Arm DMA with the provided buffer */
    dma2_ch3_setup(samples, count, circular);

    /* Start TIM6 */
    TIM6->CR1 = TIM_CR1_CEN;
}

/* ========================================================================
 *  dac_audio_is_playing - Returns non-zero if DMA2 CH3 is still enabled.
 * ======================================================================== */

int dac_audio_is_playing(void)
{
    return (DMA2_CH(3)->CCR & DMA_CCR_EN) ? 1 : 0;
}

/* Set output volume, 0-100 %. Takes effect on the next tone: the DMA buffer is
 * filled when a tone starts, so changing this mid-tone does nothing until the
 * next one. */
void dac_audio_set_volume(uint8_t percent)
{
    tone_volume = (percent > 100u) ? 100u : percent;
}

uint8_t dac_audio_get_volume(void)
{
    return tone_volume;
}
