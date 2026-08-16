/*
 * si4732.h - SI4732 AM/FM/SW broadcast receiver driver for the RT-950 Pro
 *
 * Communication via bit-bang I2C on GPIOB:
 *   PB6 = SCL (mask 0x40),  PB7 = SDA (mask 0x80)
 *   I2C address 0x22 write / 0x23 read (7-bit: 0x11, SEN pin = low)
 *
 * WARNING: XOSCEN must match PCB crystal configuration. If a 32.768 kHz
 *   crystal is on RCLK/GPO3 pins, XOSCEN=1 is required in POWER_UP ARG1
 *   with a 500ms delay before first tune. OEM sends 0x31 (XOSCEN=1) for
 *   AM+SSB mode.
 *
 * Reference: AN332 Si47xx Programming Guide (docs/pdf/Si47xx_ProgrammingGuide.pdf)
 *
 * V0.27 binary addresses (r2 / fw):
 *   i2c_read_byte:           r2 0x080289B4 / fw 0x080259B4
 *   i2c_start_send_address:  r2 0x08028A7C / fw 0x08025A7C
 *   i2c_stop:                r2 0x08028B5C / fw 0x08025B5C
 *   i2c_write_byte:          r2 0x08028BC0 / fw 0x08025BC0
 *   i2c_read_resp:           r2 0x080294F0 / fw 0x080264F0
 *   i2c_send_cmd:            r2 0x08029584 / fw 0x08026584
 *   send_cmd_and_read:       r2 0x080295AA / fw 0x080265AA
 *   read_status:             r2 0x080295F4 / fw 0x080265F4
 *   set_property:            r2 0x08029604 / fw 0x08026604
 *   wait_cts:                r2 0x0802962C / fw 0x0802662C
 *   power_up (FM/AM):        r2 0x08029460 / fw 0x08026460
 *   power_up_am_patch:       r2 0x080293E4 / fw 0x080263E4
 *   power_down:              r2 0x0801194C / fw 0x0800E94C
 *   ssb_patch_load:          r2 0x080295D0 / fw 0x080265D0
 *   set_mode:                r2 0x08011BE0 / fw 0x0800EBE0
 *   am_tune:                 r2 0x08006C34 / fw 0x08003C34
 *   set_audio_config:        r2 0x08011920 / fw 0x0800E920
 *   GPIOB literal pool:      r2 0x08028A78, 0x08028B58, 0x08028BBC, 0x08028C5C
 *
 * No hardware I2C peripheral used - purely bit-bang.
 * OEM does NOT check I2C ACK bits on write operations.
 */

#ifndef DRIVERS_SI4732_H
#define DRIVERS_SI4732_H

#include "at32f403a.h"

/* ---- SI4732 command opcodes (Silicon Labs standard) ---- */
#define SI4732_CMD_POWER_UP       0x01
#define SI4732_CMD_GET_REV        0x10
#define SI4732_CMD_POWER_DOWN     0x11
#define SI4732_CMD_SET_PROPERTY   0x12
#define SI4732_CMD_GET_PROPERTY   0x13
#define SI4732_CMD_GET_INT_STATUS 0x14
#define SI4732_CMD_FM_TUNE_FREQ   0x20
#define SI4732_CMD_FM_SEEK_START  0x21
#define SI4732_CMD_FM_TUNE_STATUS 0x22
#define SI4732_CMD_FM_RSQ_STATUS  0x23  /* OEM @ fw 0x0800E970 */
#define SI4732_CMD_AM_TUNE_FREQ   0x40
#define SI4732_CMD_AM_SEEK_START  0x41
#define SI4732_CMD_AM_TUNE_STATUS 0x42
#define SI4732_CMD_AM_RSQ_STATUS  0x43  /* OEM @ fw 0x0800E970 */
#define SI4732_CMD_SSB_TUNE_FREQ  0x40  /* Same opcode, different power-up mode */
#define SI4732_CMD_WB_TUNE_FREQ   0x50  /* Weather band tune (162.4-162.55 MHz) */
#define SI4732_CMD_WB_TUNE_STATUS 0x52  /* Weather band tune status */

/* ---- POWER_UP ARG1 bit fields (per AN332) ---- */
#define SI4732_FUNC_FM_RECV       0x00  /* FUNC bits[3:0] = FM receive */
#define SI4732_FUNC_AM_RECV       0x01  /* FUNC bits[3:0] = AM/SW/LW receive */
#define SI4732_FUNC_AM_SW_LW      0x01  /* Alias - AM/SW/LW share mode 0x01 */
#define SI4732_FUNC_WB_RECV       0x03  /* FUNC bits[3:0] = Weather band receive */
/* POWER_UP ARG1 bits, per the Si47xx datasheet:
 *   bit7 CTSIEN  bit6 GPO2OEN  bit5 PATCH  bit4 XOSCEN  bits3-0 FUNC
 *
 * These two were SWAPPED here: PATCH was defined as bit 4 and XOSCEN as bit 5.
 * The FM/AM power-up happens to send the right value (0x10 = XOSCEN, matching
 * the OEM's [0x01, 0x10, 0x05]) only because it used the mis-named constant.
 * Anyone reading the names would have concluded the driver requests a patch
 * boot, which it does not -- and setting the real PATCH bit without then
 * uploading a patch leaves the part waiting forever and never asserting CTS. */
#define SI4732_POWERUP_XOSCEN     (1U << 4)  /* Crystal oscillator enable */
#define SI4732_POWERUP_PATCH      (1U << 5)  /* Boot from an uploaded patch */

/*
 * OEM POWER_UP ARG1 values from V0.27 binary:
 *   FM mode:       0x10 = FUNC_FM  | PATCH           @ fw 0x08026472
 *   AM mode:       0x11 = FUNC_AM  | PATCH           @ fw 0x0802649C
 *   AM+SSB init:   0x31 = FUNC_AM  | PATCH | XOSCEN  @ fw 0x080263F2
 * OEM ALWAYS sets PATCH bit (needed for SSB support).
 */

/* ---- POWER_UP output modes ---- */
#define SI4732_OUT_ANALOG         0x05

/* ---- Status byte bits ---- */
#define SI4732_STATUS_CTS         (1U << 7)

/* ---- Tune status response flags (byte 0 of response) ---- */
#define SI4732_TUNE_VALID         (1U << 0)
#define SI4732_TUNE_AFCRL         (1U << 1)
#define SI4732_TUNE_STC           (1U << 6)

/* ---- SI4732 property IDs (per AN332 + OEM V0.27 usage) ---- */
#define SI4732_PROP_GPO_IEN               0x0001  /* GPO interrupt enable */
#define SI4732_PROP_REFCLK_FREQ           0x0201  /* Ref clock freq (default 32768) */
#define SI4732_PROP_REFCLK_PRESCALE       0x0202  /* Ref clock prescaler (default 1) */
#define SI4732_PROP_FM_DEEMPHASIS         0x1100  /* FM de-emphasis: 1=50us, 2=75us */
#define SI4732_PROP_FM_BLEND_STEREO_TH    0x1400  /* FM stereo blend RSSI threshold */
#define SI4732_PROP_FM_BLEND_MONO_TH      0x1401  /* FM mono blend RSSI threshold */
#define SI4732_PROP_AM_CHANNEL_FILTER     0x3102  /* AM bandwidth filter */
#define SI4732_PROP_AM_NB_RATE            0x3103  /* AM noise blanker rate */
#define SI4732_PROP_AM_SOFT_MUTE_RATE     0x3202  /* AM soft mute rate */
#define SI4732_PROP_AM_SOFT_MUTE_SNR_TH   0x3302  /* AM soft mute SNR threshold */
#define SI4732_PROP_AM_SOFT_MUTE_MAX_ATT  0x3400  /* AM soft mute max attenuation */
#define SI4732_PROP_AM_SOFT_MUTE_SNR_TH2  0x3401  /* AM soft mute SNR threshold 2 */
#define SI4732_PROP_AM_SOFT_MUTE_RELEASE  0x3402  /* AM soft mute release rate */
#define SI4732_PROP_AM_AGC_ATTACK_RATE    0x3702
#define SI4732_PROP_RX_VOLUME             0x4000  /* Output volume (0-63, default 63) */
#define SI4732_PROP_RX_HARD_MUTE          0x4001  /* L/R mute (default 0 = unmuted) */

/*
 * SSB patch properties - NOT in AN332. These are injected by the community
 * SSB firmware patch and only exist AFTER the patch is uploaded via
 * PATCH_ARGS (0x15) / PATCH_DATA (0x16). Writing to these without the patch
 * loaded is undefined behavior.
 */
#define SI4732_PROP_SSB_MODE            0x0101  /* SSB sideband: 1=LSB, 2=USB */
#define SI4732_PROP_SSB_BFO             0x0100  /* BFO offset in Hz (signed) */
#define SI4732_PROP_SSB_IF_BW           0x3102  /* SSB IF bandwidth (same reg as AM) */
/*
 * 0x3104 has DUAL meaning:
 *   Standard AM mode: AM_MODE_AFC_SW_PULL_IN_RANGE (default 0x21F7)
 *   After SSB patch:  SSB AGC override (bit0=disable, bits[7:1]=gain)
 */
#define SI4732_PROP_SSB_AGC_OVERRIDE    0x3104

/* ---- AM bandwidth filter values (for PROP 0x3102) ---- */
#define SI4732_AM_BW_6K    0x0001
#define SI4732_AM_BW_4K    0x0002
#define SI4732_AM_BW_3K    0x0003
#define SI4732_AM_BW_2K    0x0004
#define SI4732_AM_BW_1K    0x0005

/*
 * Tune status response layout (FM_TUNE_STATUS 0x22 / AM_TUNE_STATUS 0x42):
 *   Byte 0: STATUS  [CTS:7] [ERR:6] ... [STCINT:0]
 *   Byte 1: RESP1   [VALID:0] [AFCRL:1]
 *   Byte 2: RESP2   READFREQ[15:8]
 *   Byte 3: RESP3   READFREQ[7:0]
 *   Byte 4: RESP4   RSSI (dBuV)
 *   Byte 5: RESP5   SNR (dB)
 *   Byte 6: RESP6   FM: multipath / AM: reserved
 *   Byte 7: RESP7   READANTCAP
 *
 * Frequency units: FM = 10 kHz (8750 = 87.5 MHz), AM = 1 kHz
 */
struct si4732_tune_status {
    uint16_t freq;    /* FM: 10kHz units, AM: kHz, WB: kHz (may wrap) */
    uint8_t  rssi;
    uint8_t  snr;
    uint8_t  valid;
};

/* ---- API ---- */

/* Configure GPIO pins for bit-bang I2C (SCL push-pull, SDA input pull-up) */
void si4732_init(void);

/* Power up in FM or AM receive mode (analog audio output) */
int si4732_power_up_fm(void);
int si4732_power_up_am(void);

/* Power down the chip */
void si4732_power_down(void);

/* Tune FM frequency. freq_10khz is in 10 kHz units (e.g. 8750 = 87.5 MHz) */
int si4732_fm_tune(uint16_t freq_10khz);

/* Tune AM frequency in kHz (e.g. 540, 1000) */
int si4732_am_tune(uint16_t freq_khz);

/* Set AM channel bandwidth filter */
int si4732_am_set_bandwidth(uint16_t bw_value);

/* Read FM tune status into *st. Returns 0 on success, -1 on error. */
int si4732_fm_tune_status(struct si4732_tune_status *st);

/* Read AM tune status into *st. Returns 0 on success, -1 on error. */
int si4732_am_tune_status(struct si4732_tune_status *st);

/* Set a SI4732 property. Returns 0 on success, -1 on timeout. */
int si4732_set_property(uint16_t prop, uint16_t value);

/* Get chip revision. Writes part number to *part_number. Returns 0 on success. */
int si4732_get_rev(uint8_t *part_number);

/* Diagnostic: probe every 7-bit I2C address, store those that ACK, return the
 * count. Used to confirm which address the receiver actually sits at. */
uint8_t si4732_i2c_scan(uint8_t *found, uint8_t max);

/* Read single status byte from SI4732 */
uint8_t si4732_get_status(void);

/* ---- Weather band (NOAA) support ---- */

/* Power up in weather band receive mode */
int si4732_power_up_wb(void);

/* Tune WB frequency. freq_khz in kHz (e.g. 162400 = 162.400 MHz)
 * NOTE: NOAA frequencies (162400-162550) exceed uint16_t range.
 * SI4732 WB_TUNE_FREQ command uses the lower 16 bits only. */
int si4732_wb_tune(uint32_t freq_khz);

/* Read WB tune status. Returns 0 on success, -1 on error. */
int si4732_wb_tune_status(struct si4732_tune_status *st);

/* ---- SSB support (requires patch ROM upload) ---- */

/*
 * Load SSB patch into SI4732. Must be called after power_up_am() with
 * PATCH and XOSCEN bits set.
 *
 * SSB patch upload sequence (per AN332 S7.2):
 *   1. POWER_UP with PATCH=1 (ARG1 bit 5), FUNC=AM
 *   2. Wait for CTS
 *   3. For each 8-byte block in patch data:
 *        PATCH_ARGS (0x15) + 7 arg bytes -> wait CTS
 *      or PATCH_DATA (0x16) + 7 data bytes -> wait CTS
 *   4. POWER_DOWN
 *   5. POWER_UP with PATCH=0, FUNC=AM (normal boot with patch active)
 *
 * OEM V0.27 @ fw 0x080265D0: uploads 15,832 bytes (0x3DD8) from flash
 * address fw 0x08028BA0 in 8-byte blocks via I2C, with wait_cts() between
 * each block. Some SI4732 modules have the patch in internal ROM; others
 * require the ~8 KB binary embedded in firmware or loaded from SPI flash.
 *
 * TODO: Implement actual patch upload. Current stub only sets properties.
 * TODO: Extract patch binary from OEM flash image at fw 0x08028BA0.
 * TODO: Verify if RT-950 SI4732 has patch in ROM or needs upload.
 *
 * Returns 0 on success, -1 on error.
 */
int si4732_ssb_patch_load(void);

/* Tune SSB frequency in kHz. Uses AM_TUNE_FREQ under the hood. */
int si4732_ssb_tune(uint16_t freq_khz);

/* Set SSB sideband mode: 1=LSB, 2=USB */
int si4732_ssb_set_mode(uint8_t mode);

/* Set BFO (Beat Frequency Oscillator) offset in Hz. Signed, +/-16383. */
int si4732_ssb_set_bfo(int16_t offset_hz);

/* Set SSB IF bandwidth (same filter values as AM) */
int si4732_ssb_set_bandwidth(uint16_t bw_value);

/* Set SSB AGC: 0=auto, 1=manual (with gain value in bits[7:1]) */
int si4732_ssb_set_agc(uint8_t agc_disable, uint8_t gain);

/*
 * Future work / gaps:
 *   - FM_RSQ_STATUS (0x23) / AM_RSQ_STATUS (0x43): RSSI/SNR/multipath
 *     for S-meter and signal quality display
 *   - FM_RDS_STATUS (0x24): RDS station name, program type, radiotext
 *   - FM_CHANNEL_FILTER (0x1102): adjustable FM bandwidth
 *   - GPIO_CTL (0x80) / GPIO_SET (0x81): SI4732 GPO pins may be wired
 *     to antenna switch or LED on RT-950 PCB
 *   - REFCLK_FREQ (0x0201): verify RT-950 PCB ref clock source and freq
 *
 * Reference: AN332 Si47xx Programming Guide (Skyworks, Rev 207003A)
 *   available at docs/pdf/Si47xx_ProgrammingGuide.pdf
 */

#endif /* DRIVERS_SI4732_H */
